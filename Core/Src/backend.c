/**
 * backend.c - BSU_test_board: использует device_lib (backend.h, device_config.h, service.h).
 * Чтение/запись настроек только через ППКУ. Главный массив в ППКУ.
 */

#include "bsu_backend.h"
#include "bsu_protocol.h"
#include "bsu_emulator.h"
#include "main.h"
#include <string.h>
#include "device_config.h"

#define PPKY_CONFIG_SIZE  sizeof(PPKYCfg)

static uint8_t LocalConfig[PPKY_CONFIG_SIZE];
static uint8_t SavedConfig[PPKY_CONFIG_SIZE];

static uint8_t ppky_h_adr;
static uint8_t ppky_l_adr;
static uint8_t ppky_zone;

static void send_ppky_response(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    can_ext_id_t id;
    id.field.dir    = 1;
    id.field.zone   = ppky_zone;
    id.field.l_adr  = ppky_l_adr;
    id.field.h_adr  = ppky_h_adr;
    id.field.d_type = DEVICE_PPKY_TYPE;

    uint8_t buf[8];
    buf[0] = cmd;
    if (len > 7) len = 7;
    memcpy(&buf[1], data, len);
    if (len < 7)
        memset(&buf[1 + len], 0, 7 - len);

    BSU_Protocol_SendCanNow(id.ID, buf, 8);  /* приоритет: сразу, минуя очередь */
}

static void fill_default_config(void)
{
    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    memset(cfg, 0, sizeof(PPKYCfg));

    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    /* ППКУ (DEVICE_PPKY_TYPE) */
    cfg->UId.UId0 = uid0;
    cfg->UId.UId1 = uid1;
    cfg->UId.UId2 = uid2;
    cfg->UId.UId3 = HAL_GetDEVID();
    cfg->UId.UId4 = 1;
    cfg->UId.devId.zone   = 0;
    cfg->UId.devId.l_adr  = 0;
    cfg->UId.devId.h_adr  = (uint8_t)(uid0 & 0xFF);
    if (cfg->UId.devId.h_adr == 0) {
        cfg->UId.devId.h_adr = (uint8_t)(uid1 & 0xFF);
        if (cfg->UId.devId.h_adr == 0)
            cfg->UId.devId.h_adr = 1;
    }
    cfg->UId.devId.d_type = DEVICE_PPKY_TYPE;

    cfg->beep = 1;

    /* Имена зон */
    strncpy((char *)cfg->zone_name[0], "Моторный отсек", ZONE_NAME_SIZE - 1);
    strncpy((char *)cfg->zone_name[1], "Кондиционер", ZONE_NAME_SIZE - 1);
    strncpy((char *)cfg->zone_name[2], "Е-панель", ZONE_NAME_SIZE - 1);

    /* Карта МКУ: 3 зоны × (1 MCU_TC + 2 MCU_IGN) = 9 устройств
     * Зона 1: h=1,2,3
     * Зона 2: h=4,5,6
     * Зона 3: h=7,8,9
     * В каждой зоне: сначала MCU_TC, затем два MCU_IGN.
     */
    const uint8_t zone_map[9]  = {1,1,1, 2,2,2, 3,3,3};
    const uint8_t hadr_map[9]  = {1,2,3, 4,5,6, 7,8,9};
    const uint8_t mcu_type[9]  = {
        DEVICE_MCU_TC_TYPE, DEVICE_MCU_IGN_TYPE, DEVICE_MCU_IGN_TYPE,
        DEVICE_MCU_TC_TYPE, DEVICE_MCU_IGN_TYPE, DEVICE_MCU_IGN_TYPE,
        DEVICE_MCU_TC_TYPE, DEVICE_MCU_IGN_TYPE, DEVICE_MCU_IGN_TYPE
    };

    for (uint8_t i = 0; i < 9u; i++) {
        MKUCfg *m = &cfg->CfgDevices[i];
        memset(m, 0, sizeof(MKUCfg));

        /* Заполняем UniqId произвольными, но отличающимися значениями */
        m->UId.UId0 = 0x10000000u + i;
        m->UId.UId1 = 0x20000000u + i;
        m->UId.UId2 = 0x30000000u + i;
        m->UId.UId3 = uid0 ^ uid1 ^ uid2 ^ i;
        m->UId.UId4 = (uint32_t)(i + 1u);

        m->UId.devId.zone  = zone_map[i] & 0x7Fu;
        m->UId.devId.h_adr = hadr_map[i];
        m->UId.devId.l_adr = 0;
        m->UId.devId.d_type = mcu_type[i];

        /* Задержки зоны/модулей — параметры для боевой прошивки (в секундах) */
        if (zone_map[i] == 1)
            m->zone_delay = 5;   /* 5 с */
        else if (zone_map[i] == 2)
            m->zone_delay = 10;  /* 10 с */
        else if (zone_map[i] == 3)
            m->zone_delay = 15;  /* 15 с */
        else
            m->zone_delay = 0;

        for (uint8_t j = 0; j < NUM_DEV_IN_MCU; j++) {
            m->module_delay[j] = 0;
        }
        m->module_delay[0] = 30;  /* delay для первого модуля, 30 с */

        /* Один виртуальный модуль на МКУ: либо Igniter, либо DPT */
        if (mcu_type[i] == DEVICE_MCU_IGN_TYPE) {
            m->VDtype[0] = DEVICE_IGNITER_TYPE;
            m->Devices[0].type = DEVICE_IGNITER_TYPE;
            DeviceIgniterConfig *ign = (DeviceIgniterConfig *)m->Devices[0].reserv;
            memset(ign, 0, sizeof(DeviceIgniterConfig));
            ign->disable_sc_check   = 0;
            ign->threshold_break_low  = 1000;  /* мВ */
            ign->threshold_break_high = 3000;  /* мВ */
            ign->burn_retry_count   = 1;
        } else if (mcu_type[i] == DEVICE_MCU_TC_TYPE) {
            m->VDtype[0] = DEVICE_DPT_TYPE;
            m->Devices[0].type = DEVICE_DPT_TYPE;
            DeviceDPTConfig *dpt = (DeviceDPTConfig *)m->Devices[0].reserv;
            memset(dpt, 0, sizeof(DeviceDPTConfig));
            dpt->mode                 = 0;   /* ДПТ */
            dpt->use_max              = 1;   /* использовать MAX */
            dpt->max_fire_threshold_c = 60;  /* °C */
            dpt->state_change_delay_ms = 100;
        }
    }
}

static void config_service_cmd(uint8_t cmd, const uint8_t *msg_data)
{
    uint8_t data[7] = {0, 0, 0, 0, 0, 0, 0};

    switch (cmd) {
    case ServiceCmd_GetConfigSize: {
        /* Возвращаем размер конфигурации в байтах в том же формате,
         * что и реальный backend: 4 байта, big-endian (MSB first). */
        uint32_t sz = BSU_GetConfigSize();
        data[0] = (sz >> 24) & 0xFF;
        data[1] = (sz >> 16) & 0xFF;
        data[2] = (sz >> 8)  & 0xFF;
        data[3] = (sz >> 0)  & 0xFF;
        send_ppky_response(cmd, data, 7);
        //BSU_Emulator_PauseFor(20);
    } break;

    case ServiceCmd_GetConfigCRC: {
        uint32_t crc;
        if (msg_data[0] == 0)
            crc = crc32(0, SavedConfig, BSU_GetConfigSize());
        else
            crc = crc32(0, LocalConfig, BSU_GetConfigSize());
        data[0] = (crc >> 24) & 0xFF;
        data[1] = (crc >> 16) & 0xFF;
        data[2] = (crc >> 8) & 0xFF;
        data[3] = (crc >> 0) & 0xFF;
        send_ppky_response(cmd, data, 7);
    } break;

    case ServiceCmd_GetConfigWord: {
        uint16_t num_word = ((uint16_t)msg_data[0] << 8) | msg_data[1];
        uint32_t word = BSU_GetConfigWord(num_word);
        data[0] = msg_data[0];
        data[1] = msg_data[1];
        data[2] = (word >> 24) & 0xFF;
        data[3] = (word >> 16) & 0xFF;
        data[4] = (word >> 8) & 0xFF;
        data[5] = (word >> 0) & 0xFF;
        send_ppky_response(cmd, data, 7);
        //BSU_Emulator_PauseFor(20);
    } break;

    case ServiceCmd_SetConfigWord: {
        uint16_t num_word = ((uint16_t)msg_data[0] << 8) | msg_data[1];
        uint32_t word = ((uint32_t)msg_data[2] << 24) | ((uint32_t)msg_data[3] << 16) |
                        ((uint32_t)msg_data[4] << 8) | msg_data[5];
        BSU_SetConfigWord(num_word, word);
        data[0] = msg_data[0];
        data[1] = msg_data[1];
        data[2] = (word >> 24) & 0xFF;
        data[3] = (word >> 16) & 0xFF;
        data[4] = (word >> 8) & 0xFF;
        data[5] = (word >> 0) & 0xFF;
        send_ppky_response(cmd, data, 7);
    } break;

    case ServiceCmd_SaveConfig:
        BSU_SaveConfig();
        break;

    case ServiceCmd_DefaultConfig:
        BSU_DefaultConfig();
        break;

    default:
        break;
    }
}

static void handle_igniter_command(uint8_t h_adr, uint8_t l_adr, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    /* В эмуляторе 6 igniter'ов на 6 MCU_IGN (см. bsu_emulator.c),
     * здесь для примера обрабатываем только первые два, как было. */
    uint8_t idx;
    if (h_adr == 1) idx = 0;
    else if (h_adr == 2) idx = 1;
    else return;
    if (l_adr != 1) return;

    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    DeviceIgniterConfig *ic = (DeviceIgniterConfig *)cfg->CfgDevices[idx].Devices[0].reserv;

    switch (cmd) {
    case 0: break;
    case 1:
        /* В новом DeviceIgniterConfig нет поля start_duration_ms,
         * поэтому просто обновляем disable_sc_check и передаём
         * какое‑то фиксированное "duration" в эмулятор. */
        if (len >= 1) {
            ic->disable_sc_check = payload[0] ? 1 : 0;
            BSU_Emulator_SetIgniterConfig(idx, ic->disable_sc_check, 1000);
        }
        break;
    case 2:
        /* Раньше тут настраивалась start_duration_ms, сейчас игнорируем
         * и просто транслируем disable_sc_check в эмулятор. */
        if (len >= 2) {
            (void)payload;
            BSU_Emulator_SetIgniterConfig(idx, ic->disable_sc_check, 1000);
        }
        break;
    default: break;
    }
}

static void handle_dpt_command(uint8_t h_adr, uint8_t l_adr, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (l_adr != 1) return;

    /* В новой схеме у нас 3 MCU_TC (h=1,4,7), по одному DPT в каждой зоне.
     * Здесь для простоты считаем, что команды приходят на первый DPT
     * (CfgDevices[2] в fill_default_config соответствует MCU_TC в зоне 1).
     * При необходимости можно будет расширить по h_adr. */
    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    DeviceDPTConfig *dc = (DeviceDPTConfig *)cfg->CfgDevices[2].Devices[0].reserv;

    switch (cmd) {
    case 2: /* Настройка порога пожара MAX в градусах */
        if (len >= 2) {
            uint16_t v = payload[0] | ((uint16_t)payload[1] << 8);
            if (v > 0) {
                dc->max_fire_threshold_c = v;
            }
        }
        break;
    case 3: /* Включить/выключить использование MAX */
        if (len >= 1) {
            dc->use_max = payload[0] ? 1 : 0;
        }
        break;
    case 4: /* Время фильтрации состояния линии, мс */
        if (len >= 2) {
            uint16_t v = payload[0] | ((uint16_t)payload[1] << 8);
            if (v > 0) {
                dc->state_change_delay_ms = v;
            }
        }
        break;
    default:
        break;
    }
}

void BSU_Backend_Init(void)
{
    fill_default_config();
    memcpy(SavedConfig, LocalConfig, PPKY_CONFIG_SIZE);

    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    ppky_h_adr = cfg->UId.devId.h_adr;
    ppky_l_adr = cfg->UId.devId.l_adr;
    ppky_zone  = cfg->UId.devId.zone;
}

void BSU_Backend_ProcessConfig(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    can_ext_id_t id;
    id.ID = can_id;

    if (id.field.dir != 0)
        return;

    uint8_t cmd = (len > 0) ? data[0] : 0;
    const uint8_t *payload = (len > 1) ? &data[1] : NULL;
    uint8_t payload_len = (len > 1) ? (len - 1) : 0;

    if (id.field.d_type == DEVICE_PPKY_TYPE && id.field.h_adr == ppky_h_adr) {
        ppky_h_adr = id.field.h_adr;
        ppky_l_adr = id.field.l_adr;
        ppky_zone  = id.field.zone;
        if (cmd >= ServiceCmd_GetConfigSize && cmd <= ServiceCmd_DefaultConfig) {
            config_service_cmd(cmd, payload);
        }
        return;
    }

    if (id.field.d_type == DEVICE_IGNITER_TYPE && (id.field.h_adr == 1 || id.field.h_adr == 2)) {
        handle_igniter_command(id.field.h_adr, id.field.l_adr, cmd, payload, payload_len);
        return;
    }
    if (id.field.d_type == DEVICE_DPT_TYPE && id.field.h_adr == 3) {
        handle_dpt_command(id.field.h_adr, id.field.l_adr, cmd, payload, payload_len);
        return;
    }
}

uint32_t BSU_GetConfigSize(void)
{
    return (uint32_t)PPKY_CONFIG_SIZE;
}

uint32_t BSU_GetConfigWord(uint16_t num_word)
{
    uint32_t offset = (uint32_t)num_word * 4;
    if (offset + 4 > PPKY_CONFIG_SIZE)
        return 0;
    return (uint32_t)LocalConfig[offset] << 24 |
           (uint32_t)LocalConfig[offset + 1] << 16 |
           (uint32_t)LocalConfig[offset + 2] << 8 |
           LocalConfig[offset + 3];
}

void BSU_SetConfigWord(uint16_t num_word, uint32_t word)
{
    uint32_t offset = (uint32_t)num_word * 4;
    if (offset + 4 > PPKY_CONFIG_SIZE)
        return;
    LocalConfig[offset]     = (uint8_t)(word >> 24);
    LocalConfig[offset + 1] = (uint8_t)(word >> 16);
    LocalConfig[offset + 2] = (uint8_t)(word >> 8);
    LocalConfig[offset + 3] = (uint8_t)(word >> 0);
}

void BSU_SaveConfig(void)
{
    memcpy(SavedConfig, LocalConfig, PPKY_CONFIG_SIZE);
}

void BSU_DefaultConfig(void)
{
    fill_default_config();
}
