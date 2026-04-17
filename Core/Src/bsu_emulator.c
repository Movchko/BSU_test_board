/**
 * bsu_emulator.c - Эмулятор PPKY и набора МКУ/виртуальных устройств.
 *
 * Целевая топология:
 * - MCU_k1 x3 : (h=1,z=1), (h=2,z=2), (h=3,z=3)
 * - MCU_k2 x1 : (h=4,z=1)
 * - MCU_k3 x1 : (h=5,z=2)
 * - MCU_kr x1 : (h=6,z=3)
 * - PPKY      : h=1
 */

#include "bsu_emulator.h"
#include "bsu_protocol.h"
#include "bsu_backend.h"
#include "main.h"
#include <string.h>

static can_ext_id_t ppky_can_id;
static uint8_t ppky_status_sec_cnt = 0;
/* 6 МКУ на шине */
#define MCU_COUNT 6
#define IGNITER_COUNT 10
#define DPT_COUNT 3
#define LSWITCH_COUNT 2
#define RELAY_COUNT 2

static can_ext_id_t mcu_can_id[MCU_COUNT];
static uint32_t ppky_last_tick = 0;
static uint32_t mcu_last_tick[MCU_COUNT] = {0};
static uint32_t igniter_last_tick[IGNITER_COUNT] = {0};
static uint32_t dpt_last_tick[DPT_COUNT] = {0};
static uint32_t lswitch_last_tick[LSWITCH_COUNT] = {0};
static uint32_t relay_last_tick[RELAY_COUNT] = {0};
#define PPKY_INTERVAL_MS    1000
#define MCU_INTERVAL_MS     1000
#define IGNITER_INTERVAL_MS 1000
#define DPT_INTERVAL_MS     1000
#define LSWITCH_INTERVAL_MS 1000
#define RELAY_INTERVAL_MS   1000

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

typedef struct {
    IgniterStatus_t    status;
    IgniterLineState_t line_state;
    uint8_t            flags;
    uint8_t            disable_sc_check;
    uint16_t           start_duration_ms;
    uint16_t           measured_resistance_ohm;
} IgniterState_t;

typedef struct {
    uint8_t  line_state;
    uint16_t measured_resistance_ohm;
    int8_t   max_temp_c;
    uint8_t  max_fault;
    int8_t   max_internal_temp_c;
    uint16_t speed;
    uint8_t  direction;
} DPTState_t;

typedef struct {
    uint8_t line_state;
    uint16_t measured_resistance_ohm;
    int8_t max_temp_c;
    uint8_t max_fault;
    int8_t max_internal_temp_c;
} LSwitchState_t;

typedef struct {
    uint8_t desired_state;
    uint8_t actual_state;
    uint8_t error_flag;
} RelayState_t;

typedef struct {
    uint8_t zone;
    uint8_t h_adr;
    uint8_t d_type;
} McuMap_t;

static const McuMap_t mcu_map[MCU_COUNT] = {
    {1u, 1u, DEVICE_MCU_K1},
    {2u, 2u, DEVICE_MCU_K1},
    {3u, 3u, DEVICE_MCU_K1},
    {1u, 4u, DEVICE_MCU_K2},
    {2u, 5u, DEVICE_MCU_K3},
    {3u, 6u, DEVICE_MCU_KR}
};

static IgniterState_t vdev_igniter[IGNITER_COUNT];
static DPTState_t vdev_dpt[DPT_COUNT];
static LSwitchState_t vdev_lswitch[LSWITCH_COUNT];
static RelayState_t vdev_relay[RELAY_COUNT];

static can_ext_id_t igniter_id[IGNITER_COUNT];
static can_ext_id_t dpt_id[DPT_COUNT];
static can_ext_id_t lswitch_id[LSWITCH_COUNT];
static can_ext_id_t relay_id[RELAY_COUNT];

static volatile uint32_t emulator_pause_until = 0;

void BSU_Emulator_PauseFor(uint32_t ms)
{
    emulator_pause_until = HAL_GetTick() + ms;
}

static void build_ppky_id(void)
{
    ppky_can_id.field.dir    = 1;
    ppky_can_id.field.zone   = 0;
    ppky_can_id.field.l_adr  = 0;
    ppky_can_id.field.h_adr  = 1;
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

static void send_igniter_status(uint8_t ign_idx)
{
    uint8_t data[8];
    can_ext_id_t id = igniter_id[ign_idx];
    id.field.dir = 1;

    /* Формат как в device_lib::VDeviceIgniter::SetStatus:
     * data[0] = status_cmd
     * data[1] = line_state
     * data[2] = ack_flags
     * data[3..4] = measured_resistance_ohm (LE) */
    data[0] = (uint8_t)vdev_igniter[ign_idx].status;
    data[1] = (uint8_t)vdev_igniter[ign_idx].line_state;
    data[2] = vdev_igniter[ign_idx].flags;
    data[3] = (uint8_t)(vdev_igniter[ign_idx].measured_resistance_ohm & 0xFFu);
    data[4] = (uint8_t)((vdev_igniter[ign_idx].measured_resistance_ohm >> 8) & 0xFFu);
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

    /* Формат как в device_lib::VDeviceDPT::SetStatus. */
    data[0] = 0; /* DeviceDPTStatus_Idle */
    data[1] = vdev_dpt[dpt_idx].line_state;
    data[2] = (uint8_t)(vdev_dpt[dpt_idx].measured_resistance_ohm & 0xFFu);
    data[3] = (uint8_t)((vdev_dpt[dpt_idx].measured_resistance_ohm >> 8) & 0xFFu);
    data[4] = (uint8_t)vdev_dpt[dpt_idx].max_temp_c;
    data[5] = vdev_dpt[dpt_idx].max_fault;
    data[6] = (uint8_t)vdev_dpt[dpt_idx].max_internal_temp_c;
    data[7] = 0;

    BSU_Protocol_SendCan(id.ID, data, 8);
}

static void send_lswitch_status(uint8_t lsw_idx)
{
    uint8_t data[8];
    can_ext_id_t id = lswitch_id[lsw_idx];
    id.field.dir = 1;

    /* Для LSWITCH используется DPT-совместимый формат. */
    data[0] = 0; /* статус idle */
    data[1] = vdev_lswitch[lsw_idx].line_state;
    data[2] = (uint8_t)(vdev_lswitch[lsw_idx].measured_resistance_ohm & 0xFFu);
    data[3] = (uint8_t)((vdev_lswitch[lsw_idx].measured_resistance_ohm >> 8) & 0xFFu);
    data[4] = (uint8_t)vdev_lswitch[lsw_idx].max_temp_c;
    data[5] = vdev_lswitch[lsw_idx].max_fault;
    data[6] = (uint8_t)vdev_lswitch[lsw_idx].max_internal_temp_c;
    data[7] = 0;

    BSU_Protocol_SendCan(id.ID, data, 8);
}

static void send_relay_status(uint8_t relay_idx)
{
    uint8_t data[8];
    can_ext_id_t id = relay_id[relay_idx];
    id.field.dir = 1;

    /* Формат как в device_lib::VDeviceRelay::SetStatus. */
    data[0] = vdev_relay[relay_idx].error_flag ? 1u : 0u; /* DeviceRelayStatus */
    data[1] = vdev_relay[relay_idx].actual_state;
    data[2] = vdev_relay[relay_idx].error_flag;
    data[3] = vdev_relay[relay_idx].desired_state;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    BSU_Protocol_SendCan(id.ID, data, 8);
}

static int find_vdev_by_addr(const can_ext_id_t *ids, uint8_t count, uint8_t h_adr, uint8_t l_adr)
{
    uint8_t i;
    for (i = 0; i < count; i++) {
        if (ids[i].field.h_adr == h_adr && ids[i].field.l_adr == l_adr) {
            return (int)i;
        }
    }
    return -1;
}

void BSU_Emulator_Init(void)
{
    uint8_t i;
    uint8_t ign_idx = 0;
    uint8_t dpt_idx = 0;
    uint8_t lsw_idx = 0;
    uint8_t rel_idx = 0;

    build_ppky_id();
    ppky_last_tick = HAL_GetTick();
    for (i = 0; i < MCU_COUNT; i++) {
        mcu_last_tick[i] = HAL_GetTick();
    }
    for (i = 0; i < IGNITER_COUNT; i++) {
        igniter_last_tick[i] = HAL_GetTick();
    }
    for (i = 0; i < DPT_COUNT; i++) {
        dpt_last_tick[i] = HAL_GetTick();
    }
    for (i = 0; i < LSWITCH_COUNT; i++) {
        lswitch_last_tick[i] = HAL_GetTick();
    }
    for (i = 0; i < RELAY_COUNT; i++) {
        relay_last_tick[i] = HAL_GetTick();
    }

    memset(vdev_igniter, 0, sizeof(vdev_igniter));
    memset(vdev_dpt, 0, sizeof(vdev_dpt));
    memset(vdev_lswitch, 0, sizeof(vdev_lswitch));
    memset(vdev_relay, 0, sizeof(vdev_relay));

    for (i = 0; i < DPT_COUNT; i++) {
        vdev_dpt[i].line_state = 0;               /* normal */
        vdev_dpt[i].measured_resistance_ohm = 3000;
        vdev_dpt[i].max_temp_c = 25;
        vdev_dpt[i].max_fault = 0;
        vdev_dpt[i].max_internal_temp_c = 24;
    }
    for (i = 0; i < LSWITCH_COUNT; i++) {
        vdev_lswitch[i].line_state = 0;
        vdev_lswitch[i].measured_resistance_ohm = 3000;
        vdev_lswitch[i].max_temp_c = 25;
        vdev_lswitch[i].max_fault = 0;
        vdev_lswitch[i].max_internal_temp_c = 24;
    }
    for (i = 0; i < RELAY_COUNT; i++) {
        vdev_relay[i].desired_state = 0;
        vdev_relay[i].actual_state = 0;
        vdev_relay[i].error_flag = 0;
    }

    for (i = 0; i < IGNITER_COUNT; i++) {
        vdev_igniter[i].status = IGNITER_STATUS_IDLE;
        vdev_igniter[i].line_state = IGNITER_LINE_NORMAL;
        vdev_igniter[i].flags = 0;
        vdev_igniter[i].disable_sc_check = 0;
        vdev_igniter[i].start_duration_ms = 1000;
        vdev_igniter[i].measured_resistance_ohm = 900;
    }

    for (i = 0; i < MCU_COUNT; i++) {
        mcu_can_id[i].field.dir    = 0;
        mcu_can_id[i].field.zone   = mcu_map[i].zone;
        mcu_can_id[i].field.l_adr  = 0;
        mcu_can_id[i].field.h_adr  = mcu_map[i].h_adr;
        mcu_can_id[i].field.d_type = mcu_map[i].d_type;

        if (mcu_map[i].d_type == DEVICE_MCU_K1) {
            if (dpt_idx < DPT_COUNT) {
                dpt_id[dpt_idx].field.dir = 0;
                dpt_id[dpt_idx].field.zone = mcu_map[i].zone;
                dpt_id[dpt_idx].field.h_adr = mcu_map[i].h_adr;
                dpt_id[dpt_idx].field.l_adr = 1;
                dpt_id[dpt_idx].field.d_type = DEVICE_DPT_TYPE;
                dpt_idx++;
            }
            if (ign_idx < IGNITER_COUNT) {
                igniter_id[ign_idx].field.dir = 0;
                igniter_id[ign_idx].field.zone = mcu_map[i].zone;
                igniter_id[ign_idx].field.h_adr = mcu_map[i].h_adr;
                igniter_id[ign_idx].field.l_adr = 2;
                igniter_id[ign_idx].field.d_type = DEVICE_IGNITER_TYPE;
                ign_idx++;
            }
            if (ign_idx < IGNITER_COUNT) {
                igniter_id[ign_idx].field.dir = 0;
                igniter_id[ign_idx].field.zone = mcu_map[i].zone;
                igniter_id[ign_idx].field.h_adr = mcu_map[i].h_adr;
                igniter_id[ign_idx].field.l_adr = 3;
                igniter_id[ign_idx].field.d_type = DEVICE_IGNITER_TYPE;
                ign_idx++;
            }
        } else if (mcu_map[i].d_type == DEVICE_MCU_K2) {
            if (ign_idx < IGNITER_COUNT) {
                igniter_id[ign_idx].field.dir = 0;
                igniter_id[ign_idx].field.zone = mcu_map[i].zone;
                igniter_id[ign_idx].field.h_adr = mcu_map[i].h_adr;
                igniter_id[ign_idx].field.l_adr = 1;
                igniter_id[ign_idx].field.d_type = DEVICE_IGNITER_TYPE;
                ign_idx++;
            }
            if (ign_idx < IGNITER_COUNT) {
                igniter_id[ign_idx].field.dir = 0;
                igniter_id[ign_idx].field.zone = mcu_map[i].zone;
                igniter_id[ign_idx].field.h_adr = mcu_map[i].h_adr;
                igniter_id[ign_idx].field.l_adr = 2;
                igniter_id[ign_idx].field.d_type = DEVICE_IGNITER_TYPE;
                ign_idx++;
            }
            if (ign_idx < IGNITER_COUNT) {
                igniter_id[ign_idx].field.dir = 0;
                igniter_id[ign_idx].field.zone = mcu_map[i].zone;
                igniter_id[ign_idx].field.h_adr = mcu_map[i].h_adr;
                igniter_id[ign_idx].field.l_adr = 3;
                igniter_id[ign_idx].field.d_type = DEVICE_IGNITER_TYPE;
                ign_idx++;
            }
        } else if (mcu_map[i].d_type == DEVICE_MCU_K3) {
            if (lsw_idx < LSWITCH_COUNT) {
                lswitch_id[lsw_idx].field.dir = 0;
                lswitch_id[lsw_idx].field.zone = mcu_map[i].zone;
                lswitch_id[lsw_idx].field.h_adr = mcu_map[i].h_adr;
                lswitch_id[lsw_idx].field.l_adr = 1;
                lswitch_id[lsw_idx].field.d_type = DEVICE_LSWITCH_TYPE;
                lsw_idx++;
            }
            if (lsw_idx < LSWITCH_COUNT) {
                lswitch_id[lsw_idx].field.dir = 0;
                lswitch_id[lsw_idx].field.zone = mcu_map[i].zone;
                lswitch_id[lsw_idx].field.h_adr = mcu_map[i].h_adr;
                lswitch_id[lsw_idx].field.l_adr = 2;
                lswitch_id[lsw_idx].field.d_type = DEVICE_LSWITCH_TYPE;
                lsw_idx++;
            }
            if (ign_idx < IGNITER_COUNT) {
                igniter_id[ign_idx].field.dir = 0;
                igniter_id[ign_idx].field.zone = mcu_map[i].zone;
                igniter_id[ign_idx].field.h_adr = mcu_map[i].h_adr;
                igniter_id[ign_idx].field.l_adr = 3;
                igniter_id[ign_idx].field.d_type = DEVICE_IGNITER_TYPE;
                ign_idx++;
            }
        } else if (mcu_map[i].d_type == DEVICE_MCU_KR) {
            if (rel_idx < RELAY_COUNT) {
                relay_id[rel_idx].field.dir = 0;
                relay_id[rel_idx].field.zone = mcu_map[i].zone;
                relay_id[rel_idx].field.h_adr = mcu_map[i].h_adr;
                relay_id[rel_idx].field.l_adr = 1;
                relay_id[rel_idx].field.d_type = DEVICE_RELAY_TYPE;
                rel_idx++;
            }
            if (rel_idx < RELAY_COUNT) {
                relay_id[rel_idx].field.dir = 0;
                relay_id[rel_idx].field.zone = mcu_map[i].zone;
                relay_id[rel_idx].field.h_adr = mcu_map[i].h_adr;
                relay_id[rel_idx].field.l_adr = 2;
                relay_id[rel_idx].field.d_type = DEVICE_RELAY_TYPE;
                rel_idx++;
            }
        }
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

    for (int i = 0; i < MCU_COUNT; i++) {
        if (now - mcu_last_tick[i] >= MCU_INTERVAL_MS) {
            mcu_last_tick[i] = now;
            send_mcu_packet(i);
        }
    }

    for (int i = 0; i < IGNITER_COUNT; i++) {
        if (now - igniter_last_tick[i] >= IGNITER_INTERVAL_MS) {
            igniter_last_tick[i] = now;
            send_igniter_status(i);
        }
    }

    for (int i = 0; i < DPT_COUNT; i++) {
        if (now - dpt_last_tick[i] >= DPT_INTERVAL_MS) {
            dpt_last_tick[i] = now;
            send_dpt_status(i);
        }
    }

    for (int i = 0; i < LSWITCH_COUNT; i++) {
        if (now - lswitch_last_tick[i] >= LSWITCH_INTERVAL_MS) {
            lswitch_last_tick[i] = now;
            send_lswitch_status(i);
        }
    }

    for (int i = 0; i < RELAY_COUNT; i++) {
        if (now - relay_last_tick[i] >= RELAY_INTERVAL_MS) {
            relay_last_tick[i] = now;
            send_relay_status(i);
        }
    }
}

void BSU_Emulator_SetIgniterConfigByAddr(uint8_t h_adr, uint8_t l_adr, uint8_t disable_sc_check, uint16_t start_duration_ms)
{
    int idx = find_vdev_by_addr(igniter_id, IGNITER_COUNT, h_adr, l_adr);
    if (idx >= 0) {
        vdev_igniter[idx].disable_sc_check = disable_sc_check;
        vdev_igniter[idx].start_duration_ms = start_duration_ms;
    }
}

void BSU_Emulator_SetDPTConfigByAddr(uint8_t h_adr, uint8_t l_adr, uint16_t speed, uint8_t direction)
{
    int idx = find_vdev_by_addr(dpt_id, DPT_COUNT, h_adr, l_adr);
    if (idx >= 0) {
        vdev_dpt[idx].speed = speed;
        vdev_dpt[idx].direction = direction;
    }
}

void BSU_Emulator_SetRelayStateByAddr(uint8_t h_adr, uint8_t l_adr, uint8_t desired_state)
{
    int idx = find_vdev_by_addr(relay_id, RELAY_COUNT, h_adr, l_adr);
    if (idx >= 0) {
        vdev_relay[idx].desired_state = desired_state ? 1u : 0u;
        vdev_relay[idx].actual_state = vdev_relay[idx].desired_state;
        vdev_relay[idx].error_flag = 0u;
    }
}
