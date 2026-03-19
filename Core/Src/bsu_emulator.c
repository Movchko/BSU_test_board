/**
 * bsu_emulator.c - Эмуляция PPKY, 3 МКУ и их исполнителей
 *
 * На шине:
 * - ППКУ: h_adr из UID, d_type=10 (DEVICE_PPKY_TYPE)
 * - МКУ 1 (h_adr=1): d_type=13 (DEVICE_MCU_IGN_TYPE) - шлёт пакет
 * - МКУ 2 (h_adr=2): d_type=13 (DEVICE_MCU_IGN_TYPE) - шлёт пакет
 * - МКУ 3 (h_adr=3): d_type=14 (DEVICE_MCU_TC_TYPE) - шлёт пакет
 * - Igniter 1 (h=1, l=1): d_type=11
 * - Igniter 2 (h=2, l=1): d_type=11
 * - DPT (h=3, l=1): d_type=12
 */

#include "bsu_emulator.h"
#include "bsu_protocol.h"
#include "bsu_backend.h"
#include "main.h"
#include <string.h>

static can_ext_id_t ppky_can_id;
static uint8_t ppky_status_sec_cnt = 0;
/* 9 МКУ на шине (3 зоны × 3 МКУ) */
static can_ext_id_t mcu_can_id[9];
static uint32_t ppky_last_tick = 0;
static uint32_t mcu_last_tick[9]   = {0};
static uint32_t igniter_last_tick[6] = {0};
static uint32_t dpt_last_tick[3]   = {0};
#define PPKY_INTERVAL_MS    1000
#define MCU_INTERVAL_MS     1000
#define IGNITER_INTERVAL_MS 1000
#define DPT_INTERVAL_MS     1000
#define DPT_STATUS_MAX      5

typedef enum {
    IGNITER_STATUS_IDLE = 0,
    IGNITER_STATUS_RUN  = 1,
    IGNITER_STATUS_ERR  = 2
} IgniterStatus_t;

typedef enum {
    IGNITER_LINE_NORMAL = 0,
    IGNITER_LINE_BREAK  = 1,
    IGNITER_LINE_SHORT  = 2
} IgniterLineState_t;

static struct {
    IgniterStatus_t    status;
    IgniterLineState_t line_state;
    uint8_t            flags;
    uint8_t            disable_sc_check;
    uint16_t           start_duration_ms;
} vdev_igniter[6];

static struct {
    uint16_t speed;
    uint8_t  direction;
} vdev_dpt[3];

static can_ext_id_t igniter_id[6];
static can_ext_id_t dpt_id[3];
static uint8_t dpt_status_idx[3] = {0};

static volatile uint32_t emulator_pause_until = 0;

void BSU_Emulator_PauseFor(uint32_t ms)
{
    emulator_pause_until = HAL_GetTick() + ms;
}

static void build_ppky_id(void)
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();

    uint8_t hadr = (uint8_t)(uid0 & 0xFF);
    if (hadr == 0) {
        hadr = (uint8_t)(uid1 & 0xFF);
        if (hadr == 0)
            hadr = 1;
    }

    ppky_can_id.field.dir    = 1;
    ppky_can_id.field.zone   = 0;
    ppky_can_id.field.l_adr  = 0;
    ppky_can_id.field.h_adr  = hadr;
    ppky_can_id.field.d_type = DEVICE_PPKY_TYPE;
}

static void send_ppky_packet(void)
{
    /* Эмуляция статуса ППКУ в формате боевой прошивки (см. AppSetStatus):
     * data[0] = Code (статус, пока 0)
     * data[1] = status_sec_cnt (секунды с запуска, modulo 256)
     * data[2] = power   (шаг 100 мВ, code * 0.1 В)
     * data[3] = Rpower  (шаг 100 мВ)
     * data[4] = current1 (шаг 50 мА, code * 0.05 А)
     * data[5] = current2 (шаг 50 мА)
     */
    uint8_t data[8] = {0};

    ppky_status_sec_cnt++;  /* раз в секунду, переполнение по uint8_t нас устраивает */

    uint8_t power_code  = 200; /* 20.0 В */
    uint8_t rpower_code = 195; /* 19.5 В */
    uint8_t cur1_code   = 10;  /* 0.5 А */
    uint8_t cur2_code   = 4;   /* 0.2 А */

    data[0] = 0;                    /* Code */
    data[1] = ppky_status_sec_cnt;  /* секунды работы */
    data[2] = power_code;
    data[3] = rpower_code;
    data[4] = cur1_code;
    data[5] = cur2_code;
    data[6] = 0;
    data[7] = 0;

    BSU_Protocol_SendCan(ppky_can_id.ID, data, 8);
}

static void send_mcu_packet(uint8_t mcu_idx)
{
    /* Формат статуса МКУ, совместимый с парсером GUI (bus_monitor.py):
     * data[0] = Code (пока 0)
     * data[1..4] = tick (LE, uint32_t)
     * data[5] = CAN flags: bit0 = CAN1, bit1 = CAN2
     */
    uint8_t data[8] = {0};
    can_ext_id_t id = mcu_can_id[mcu_idx];
    id.field.dir = 1;

    uint32_t tick = HAL_GetTick();
    data[0] = 0;
    data[1] = (uint8_t)(tick & 0xFF);
    data[2] = (uint8_t)((tick >> 8) & 0xFF);
    data[3] = (uint8_t)((tick >> 16) & 0xFF);
    data[4] = (uint8_t)((tick >> 24) & 0xFF);
    data[5] = 0x03; /* обе шины "активны" */

    BSU_Protocol_SendCan(id.ID, data, 8);
}

static void send_igniter_status(uint8_t vdev_idx)
{
    uint8_t data[8];
    can_ext_id_t id = igniter_id[vdev_idx];
    id.field.dir = 1;

    data[0] = 0;
    data[1] = (uint8_t)vdev_igniter[vdev_idx].status;
    data[2] = (uint8_t)vdev_igniter[vdev_idx].line_state;
    data[3] = vdev_igniter[vdev_idx].flags;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    BSU_Protocol_SendCan(id.ID, data, 8);
}

static void send_dpt_status(uint8_t dpt_idx)
{
    uint8_t data[8];
    can_ext_id_t id = dpt_id[dpt_idx];
    id.field.dir = 1;

    /* Эмуляция статуса ДПТ в "backend"-формате:
     * data[0] = Code (DeviceDPTStatus), сейчас 0
     * data[1] = LineState
     * data[2..3] = measured_resistance_ohm (LE, 16 бит)
     * data[4] = max_temp_c
     * data[5] = max_fault (0/1)
     */
    uint16_t r_ohm = 3000;  /* условная "норма" */
    uint8_t line_state = 1; /* например, NORMAL */
    int8_t max_temp_c = 25;
    uint8_t max_fault = 0;

    data[0] = 0;
    data[1] = line_state;
    data[2] = (uint8_t)(r_ohm & 0xFF);
    data[3] = (uint8_t)((r_ohm >> 8) & 0xFF);
    data[4] = (uint8_t)max_temp_c;
    data[5] = max_fault;
    data[6] = 0;
    data[7] = 0;

    BSU_Protocol_SendCan(id.ID, data, 8);

    dpt_status_idx[dpt_idx]++;
    if (dpt_status_idx[dpt_idx] >= DPT_STATUS_MAX)
        dpt_status_idx[dpt_idx] = 0;
}

void BSU_Emulator_Init(void)
{
    build_ppky_id();
    ppky_last_tick = HAL_GetTick();
    for (int i = 0; i < 9; i++) {
        mcu_last_tick[i] = HAL_GetTick();
    }
    for (int i = 0; i < 6; i++) {
        igniter_last_tick[i] = HAL_GetTick();
    }
    for (int i = 0; i < 3; i++) {
        dpt_last_tick[i] = HAL_GetTick();
    }

    memset(vdev_igniter, 0, sizeof(vdev_igniter));
    memset(vdev_dpt, 0, sizeof(vdev_dpt));

    /* Топология, согласованная с fill_default_config:
     * 9 МКУ:
     *   Зона 1: h=1 (MCU_TC), h=2,3 (MCU_IGN)
     *   Зона 2: h=4 (MCU_TC), h=5,6 (MCU_IGN)
     *   Зона 3: h=7 (MCU_TC), h=8,9 (MCU_IGN)
     */
    const uint8_t zone_map[9]  = {1,1,1, 2,2,2, 3,3,3};
    const uint8_t hadr_map[9]  = {1,2,3, 4,5,6, 7,8,9};
    const uint8_t mcu_type[9]  = {
        DEVICE_MCU_TC_TYPE, DEVICE_MCU_IGN_TYPE, DEVICE_MCU_IGN_TYPE,
        DEVICE_MCU_TC_TYPE, DEVICE_MCU_IGN_TYPE, DEVICE_MCU_IGN_TYPE,
        DEVICE_MCU_TC_TYPE, DEVICE_MCU_IGN_TYPE, DEVICE_MCU_IGN_TYPE
    };

    for (int i = 0; i < 9; i++) {
        mcu_can_id[i].field.dir    = 0;
        mcu_can_id[i].field.zone   = zone_map[i];
        mcu_can_id[i].field.l_adr  = 0;
        mcu_can_id[i].field.h_adr  = hadr_map[i];
        mcu_can_id[i].field.d_type = mcu_type[i];
    }

    /* 6 Igniter: по одному на каждый MCU_IGN (l_adr = 1) */
    uint8_t ign_idx = 0;
    for (int i = 0; i < 9 && ign_idx < 6; i++) {
        if (mcu_type[i] != DEVICE_MCU_IGN_TYPE)
            continue;

        vdev_igniter[ign_idx].status     = IGNITER_STATUS_IDLE;
        vdev_igniter[ign_idx].line_state = IGNITER_LINE_NORMAL;
        vdev_igniter[ign_idx].flags      = 0;

        igniter_id[ign_idx].field.dir    = 0;
        igniter_id[ign_idx].field.zone   = zone_map[i];
        igniter_id[ign_idx].field.l_adr  = 1;
        igniter_id[ign_idx].field.h_adr  = hadr_map[i];
        igniter_id[ign_idx].field.d_type = DEVICE_IGNITER_TYPE;

        ign_idx++;
    }

    /* 3 DPT: по одному на каждый MCU_TC (l_adr = 1) */
    uint8_t dpt_idx = 0;
    for (int i = 0; i < 9 && dpt_idx < 3; i++) {
        if (mcu_type[i] != DEVICE_MCU_TC_TYPE)
            continue;

        dpt_id[dpt_idx].field.dir    = 0;
        dpt_id[dpt_idx].field.zone   = zone_map[i];
        dpt_id[dpt_idx].field.l_adr  = 1;
        dpt_id[dpt_idx].field.h_adr  = hadr_map[i];
        dpt_id[dpt_idx].field.d_type = DEVICE_DPT_TYPE;

        dpt_idx++;
    }
}

void BSU_Emulator_Tick1ms(void)
{
    (void)0;
}

void BSU_Emulator_Process(void)
{
    uint32_t now = HAL_GetTick();
    if (now < emulator_pause_until)
        return;

    if (now - ppky_last_tick >= PPKY_INTERVAL_MS) {
        ppky_last_tick = now;
        send_ppky_packet();
    }

    /* 9 МКУ */
    for (int i = 0; i < 9; i++) {
        if (now - mcu_last_tick[i] >= MCU_INTERVAL_MS) {
            mcu_last_tick[i] = now;
            send_mcu_packet(i);
        }
    }

    /* 6 Igniter */
    for (int i = 0; i < 6; i++) {
        if (now - igniter_last_tick[i] >= IGNITER_INTERVAL_MS) {
            igniter_last_tick[i] = now;
            send_igniter_status(i);
        }
    }

    /* 3 DPT */
    for (int i = 0; i < 3; i++) {
        if (now - dpt_last_tick[i] >= DPT_INTERVAL_MS) {
            dpt_last_tick[i] = now;
            send_dpt_status(i);
        }
    }
}

void BSU_Emulator_SetIgniterConfig(uint8_t vdev_idx, uint8_t disable_sc_check, uint16_t start_duration_ms)
{
    if (vdev_idx < 6) {
        vdev_igniter[vdev_idx].disable_sc_check   = disable_sc_check;
        vdev_igniter[vdev_idx].start_duration_ms = start_duration_ms;
    }
}

void BSU_Emulator_SetDPTConfig(uint8_t vdev_idx, uint16_t speed, uint8_t direction)
{
    if (vdev_idx < 3) {
        vdev_dpt[vdev_idx].speed     = speed;
        vdev_dpt[vdev_idx].direction = direction;
    }
}
