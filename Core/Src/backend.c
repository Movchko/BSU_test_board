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
#define CFG_FLASH_BASE_ADDR  (0x08060000u)      /* последний сектор 128 КБ */
#define CFG_FLASH_SECTOR     FLASH_SECTOR_7
#define CFG_FLASH_BYTES      (128u * 1024u)
#define CFG_FLASH_VERSION    2u

static uint8_t LocalConfig[PPKY_CONFIG_SIZE];

static uint8_t ppky_h_adr;
static uint8_t ppky_l_adr;
static uint8_t ppky_zone;

static const uint32_t cfg_payload_addr = CFG_FLASH_BASE_ADDR + sizeof(PPKYConfigHeader);
/* SavedConfig физически лежит в flash начиная с cfg_payload_addr. */
static const uint8_t * const SavedConfig = (const uint8_t *)cfg_payload_addr;

static uint8_t cfg_flash_header_valid(const PPKYConfigHeader *hdr)
{
    if (hdr == NULL) {
        return 0u;
    }
    if (hdr->magic != PPKY_CFG_HEADER_MAGIC) {
        return 0u;
    }
    if (hdr->version != (uint16_t)CFG_FLASH_VERSION) {
        return 0u;
    }
    if (hdr->size != (uint32_t)PPKY_CONFIG_SIZE) {
        return 0u;
    }
    if ((sizeof(PPKYConfigHeader) + hdr->size) > CFG_FLASH_BYTES) {
        return 0u;
    }
    return 1u;
}

static uint8_t cfg_flash_load_saved(uint8_t *dst)
{
    const PPKYConfigHeader *hdr = (const PPKYConfigHeader *)CFG_FLASH_BASE_ADDR;
    if (!cfg_flash_header_valid(hdr)) {
        return 0u;
    }

    memcpy(dst, (const void *)cfg_payload_addr, PPKY_CONFIG_SIZE);
    return 1u;
}

static HAL_StatusTypeDef cfg_flash_program_bytes(uint32_t dst_addr, const uint8_t *src, uint32_t len)
{
    uint32_t i = 0u;
    while (i < len) {
        uint32_t w = 0xFFFFFFFFu;
        uint8_t b0 = (i + 0u < len) ? src[i + 0u] : 0xFFu;
        uint8_t b1 = (i + 1u < len) ? src[i + 1u] : 0xFFu;
        uint8_t b2 = (i + 2u < len) ? src[i + 2u] : 0xFFu;
        uint8_t b3 = (i + 3u < len) ? src[i + 3u] : 0xFFu;
        w = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dst_addr + i, w) != HAL_OK) {
            return HAL_ERROR;
        }
        i += 4u;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef cfg_flash_store_local(void)
{
    HAL_StatusTypeDef st = HAL_OK;
    PPKYConfigHeader hdr;
    FLASH_EraseInitTypeDef er = {0};
    uint32_t erase_err = 0u;

    memset(&hdr, 0, sizeof(hdr));
    hdr.magic = PPKY_CFG_HEADER_MAGIC;
    hdr.version = (uint16_t)CFG_FLASH_VERSION;
    hdr.size = (uint32_t)PPKY_CONFIG_SIZE;

    HAL_FLASH_Unlock();

    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Sector = CFG_FLASH_SECTOR;
    er.NbSectors = 1u;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase(&er, &erase_err) != HAL_OK) {
        st = HAL_ERROR;
    }

    if (st == HAL_OK) {
        st = cfg_flash_program_bytes(CFG_FLASH_BASE_ADDR, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr));
    }
    if (st == HAL_OK) {
        st = cfg_flash_program_bytes(cfg_payload_addr, LocalConfig, (uint32_t)PPKY_CONFIG_SIZE);
    }

    HAL_FLASH_Lock();
    return st;
}

static uint32_t cfg_flash_saved_crc(void)
{
    const PPKYConfigHeader *hdr = (const PPKYConfigHeader *)CFG_FLASH_BASE_ADDR;
    if (!cfg_flash_header_valid(hdr)) {
        return crc32(0, LocalConfig, (uint32_t)PPKY_CONFIG_SIZE);
    }
    return crc32(0, SavedConfig, hdr->size);
}

static void backend_apply_runtime_cfg(void)
{
    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    ppky_h_adr = cfg->UId.devId.h_adr;
    ppky_l_adr = cfg->UId.devId.l_adr;
    ppky_zone  = cfg->UId.devId.zone;
    BSU_Emulator_ApplyConfig(LocalConfig, (uint32_t)PPKY_CONFIG_SIZE);
}

#define EMU_MCU_COUNT 7u

typedef struct {
    uint8_t zone;
    uint8_t h_adr;
    uint8_t d_type;
} EmuMcuMap_t;

static const EmuMcuMap_t emu_mcu_map[EMU_MCU_COUNT] = {
    {1u, 1u, DEVICE_MCU_K1},
    {2u, 2u, DEVICE_MCU_K1},
    {3u, 3u, DEVICE_MCU_K1},
    {1u, 4u, DEVICE_MCU_K2},
    {2u, 5u, DEVICE_MCU_K3},
    {3u, 6u, DEVICE_MCU_KR},
    {3u, 7u, DEVICE_MCU_K3}
};

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
    cfg->UId.devId.h_adr  = 1; /* По ТЗ: ППКУ адрес 1 */
    cfg->UId.devId.d_type = DEVICE_PPKY_TYPE;

    cfg->beep = 1;
    cfg->fire_mode = 0;
    cfg->power_input = 0;
    cfg->power_value = 24;
    cfg->rs485_on = 0;
    cfg->ex_can_on = 0;
    cfg->ex_can_protocol = 0;
    cfg->isBRP = 0;

    /* Имена зон */
    strncpy((char *)cfg->zone_name[0], "Моторный отсек", ZONE_NAME_SIZE - 1);
    strncpy((char *)cfg->zone_name[1], "Кондиционер", ZONE_NAME_SIZE - 1);
    strncpy((char *)cfg->zone_name[2], "Е-панель", ZONE_NAME_SIZE - 1);
    cfg->fire_and[0] = 1; /* первая зона: режим И */

    for (uint8_t i = 0; i < EMU_MCU_COUNT; i++) {
        MKUCfg *m = &cfg->CfgDevices[i];
        memset(m, 0, sizeof(MKUCfg));

        /* Заполняем UniqId произвольными, но отличающимися значениями */
        m->UId.UId0 = 0x10000000u + i;
        m->UId.UId1 = 0x20000000u + i;
        m->UId.UId2 = 0x30000000u + i;
        m->UId.UId3 = uid0 ^ uid1 ^ uid2 ^ i;
        m->UId.UId4 = (uint32_t)(i + 1u);

        m->UId.devId.zone  = emu_mcu_map[i].zone & 0x7Fu;
        m->UId.devId.h_adr = emu_mcu_map[i].h_adr;
        m->UId.devId.l_adr = 0;
        m->UId.devId.d_type = emu_mcu_map[i].d_type;

        /* Задержки зоны/модулей — параметры для боевой прошивки (в секундах) */
        if (emu_mcu_map[i].zone == 1)
            m->zone_delay = 25;  /* по ТЗ для первой зоны */
        else if (emu_mcu_map[i].zone == 2)
            m->zone_delay = 10;  /* 10 с */
        else if (emu_mcu_map[i].zone == 3)
            m->zone_delay = 15;  /* 15 с */
        else
            m->zone_delay = 0;

        for (uint8_t j = 0; j < NUM_DEV_IN_MCU; j++) {
            m->module_delay[j] = 0;
        }
        m->module_delay[0] = 30;  /* delay для первого модуля, 30 с */

        if (emu_mcu_map[i].d_type == DEVICE_MCU_K1) {
            /* K1: l1=DPT, l2=IGN, l3=IGN */
            m->VDtype[0] = DEVICE_DPT_TYPE;
            DeviceDPTConfig *dpt = (DeviceDPTConfig *)m->Devices[0].reserv;
            memset(dpt, 0, sizeof(DeviceDPTConfig));
            dpt->mode = 0;
            dpt->use_max = 1;
            dpt->max_fire_threshold_c = 60;
            dpt->state_change_delay_ms = 100;

            m->VDtype[1] = DEVICE_IGNITER_TYPE;
            DeviceIgniterConfig *ign1 = (DeviceIgniterConfig *)m->Devices[1].reserv;
            memset(ign1, 0, sizeof(DeviceIgniterConfig));
            ign1->disable_sc_check = 1;
            ign1->threshold_break_low = 1000;
            ign1->threshold_break_high = 3000;
            ign1->burn_retry_count = 0;

            m->VDtype[2] = DEVICE_IGNITER_TYPE;
            DeviceIgniterConfig *ign2 = (DeviceIgniterConfig *)m->Devices[2].reserv;
            memset(ign2, 0, sizeof(DeviceIgniterConfig));
            ign2->disable_sc_check = 1;
            ign2->threshold_break_low = 1000;
            ign2->threshold_break_high = 3000;
            ign2->burn_retry_count = 0;
        } else if (emu_mcu_map[i].d_type == DEVICE_MCU_K2) {
            /* K2: l1=IGN, l2=IGN, l3=IGN */
            for (uint8_t s = 0; s < 3u; s++) {
                m->VDtype[s] = DEVICE_IGNITER_TYPE;
                DeviceIgniterConfig *ign = (DeviceIgniterConfig *)m->Devices[s].reserv;
                memset(ign, 0, sizeof(DeviceIgniterConfig));
                ign->disable_sc_check = 1;
                ign->threshold_break_low = 1000;
                ign->threshold_break_high = 3000;
                ign->burn_retry_count = 0;
            }
        } else if (emu_mcu_map[i].d_type == DEVICE_MCU_K3) {
            if (emu_mcu_map[i].h_adr == 7u) {
                /* Дополнительный K3: l1=BUTTON, l2=LSWITCH, l3=IGN */
                m->VDtype[0] = DEVICE_BUTTON_TYPE;
                DeviceButtonConfig *btn = (DeviceButtonConfig *)m->Devices[0].reserv;
                memset(btn, 0, sizeof(DeviceButtonConfig));
                btn->mode = 2;
                btn->use_max = 0;
                btn->max_fire_threshold_c = 60;
                btn->state_change_delay_ms = 100;
                btn->button_kind = DeviceButtonKind_StartSP;
                btn->normal_closed = 0;

                m->VDtype[1] = DEVICE_LSWITCH_TYPE;
                DeviceDPTConfig *lsw = (DeviceDPTConfig *)m->Devices[1].reserv;
                memset(lsw, 0, sizeof(DeviceDPTConfig));
                lsw->mode = 1;
                lsw->use_max = 0;
                lsw->max_fire_threshold_c = 60;
                lsw->state_change_delay_ms = 100;
                DeviceLimitSwitchConfig *lsc = (DeviceLimitSwitchConfig *)m->Devices[1].reserv;
                lsc->trigger_delay_s = 0;
                lsc->function = DeviceLimitSwitchFunction_SetFault;
                lsc->normal_closed = 0;

                m->VDtype[2] = DEVICE_IGNITER_TYPE;
                DeviceIgniterConfig *ign = (DeviceIgniterConfig *)m->Devices[2].reserv;
                memset(ign, 0, sizeof(DeviceIgniterConfig));
                ign->disable_sc_check = 1;
                ign->threshold_break_low = 1000;
                ign->threshold_break_high = 3000;
                ign->burn_retry_count = 0;
            } else {
                /* Базовый K3: l1=LSWITCH, l2=LSWITCH, l3=IGN */
                m->VDtype[0] = DEVICE_LSWITCH_TYPE;
                DeviceDPTConfig *lsw1 = (DeviceDPTConfig *)m->Devices[0].reserv;
                memset(lsw1, 0, sizeof(DeviceDPTConfig));
                lsw1->mode = 1;
                lsw1->use_max = 0;
                lsw1->max_fire_threshold_c = 60;
                lsw1->state_change_delay_ms = 100;
                DeviceLimitSwitchConfig *lsc1 = (DeviceLimitSwitchConfig *)m->Devices[0].reserv;
                lsc1->trigger_delay_s = 0;
                lsc1->function = DeviceLimitSwitchFunction_SetFault;
                lsc1->normal_closed = 0;

                m->VDtype[1] = DEVICE_LSWITCH_TYPE;
                DeviceDPTConfig *lsw2 = (DeviceDPTConfig *)m->Devices[1].reserv;
                memset(lsw2, 0, sizeof(DeviceDPTConfig));
                lsw2->mode = 1;
                lsw2->use_max = 0;
                lsw2->max_fire_threshold_c = 60;
                lsw2->state_change_delay_ms = 100;
                DeviceLimitSwitchConfig *lsc2 = (DeviceLimitSwitchConfig *)m->Devices[1].reserv;
                lsc2->trigger_delay_s = 0;
                lsc2->function = DeviceLimitSwitchFunction_SetFault;
                lsc2->normal_closed = 0;

                m->VDtype[2] = DEVICE_IGNITER_TYPE;
                DeviceIgniterConfig *ign = (DeviceIgniterConfig *)m->Devices[2].reserv;
                memset(ign, 0, sizeof(DeviceIgniterConfig));
                ign->disable_sc_check = 1;
                ign->threshold_break_low = 1000;
                ign->threshold_break_high = 3000;
                ign->burn_retry_count = 0;
            }
        } else if (emu_mcu_map[i].d_type == DEVICE_MCU_KR) {
            /* KR: l1=RELAY, l2=RELAY */
            m->VDtype[0] = DEVICE_RELAY_TYPE;
            DeviceRelayConfig *r1 = (DeviceRelayConfig *)m->Devices[0].reserv;
            memset(r1, 0, sizeof(DeviceRelayConfig));
            r1->initial_state = 0;
            r1->persist_state_enabled = 0;
            r1->feedback_inverted = 0;
            r1->switch_delay_s = 0;
            r1->settle_time_ms = 100;

            m->VDtype[1] = DEVICE_RELAY_TYPE;
            DeviceRelayConfig *r2 = (DeviceRelayConfig *)m->Devices[1].reserv;
            memset(r2, 0, sizeof(DeviceRelayConfig));
            r2->initial_state = 0;
            r2->persist_state_enabled = 0;
            r2->feedback_inverted = 0;
            r2->switch_delay_s = 0;
            r2->settle_time_ms = 100;
        }
    }
}

static int find_cfg_device_index_by_hadr(uint8_t h_adr)
{
    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    for (uint8_t i = 0; i < 32u; i++) {
        if (cfg->CfgDevices[i].UId.devId.h_adr == h_adr &&
            cfg->CfgDevices[i].UId.devId.l_adr == 0 &&
            cfg->CfgDevices[i].UId.devId.d_type != 0u) {
            return (int)i;
        }
    }
    return -1;
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
            crc = cfg_flash_saved_crc();
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
        send_ppky_response(cmd, data, 7);
        /* Как ППКУ после APPLY на МКУ: подтверждение в ПО (cmd=168). */
        {
            uint8_t apply_done[7] = {0};
            apply_done[0] = (uint8_t)EMU_MCU_COUNT; /* ok_count */
            apply_done[1] = (uint8_t)EMU_MCU_COUNT; /* target_count */
            send_ppky_response(ServiceCmd_ApplyConfigDone, apply_done, 7);
        }
        break;

    case ServiceCmd_StartSetConfig:
        PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
        memset(cfg, 0, sizeof(PPKYCfg));
        send_ppky_response(cmd, data, 7);
        break;

    default:
        break;
    }
}

static void handle_igniter_command(uint8_t h_adr, uint8_t l_adr, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (l_adr == 0u)
        return;

    int cfg_idx = find_cfg_device_index_by_hadr(h_adr);
    if (cfg_idx < 0)
        return;

    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    MKUCfg *m = &cfg->CfgDevices[cfg_idx];
    uint8_t slot = (uint8_t)(l_adr - 1u);
    if (slot >= NUM_DEV_IN_MCU || m->VDtype[slot] != DEVICE_IGNITER_TYPE)
        return;

    DeviceIgniterConfig *ic = (DeviceIgniterConfig *)m->Devices[slot].reserv;

    switch (cmd) {
    case 11: /* VDeviceIgniter::CommandCB -> disable_sc_check */
    case 1:  /* совместимость со старой командой */
        if (len >= 1) {
            ic->disable_sc_check = payload[0] ? 1 : 0;
            BSU_Emulator_SetIgniterConfigByAddr(h_adr, l_adr, ic->disable_sc_check, 1000);
        }
        break;
    case 12: /* пороги + retries (device_lib) */
        if (len >= 5) {
            uint16_t bl = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            uint16_t bh = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
            if (bl > 0u)
                ic->threshold_break_low = bl;
            if (bh > 0u)
                ic->threshold_break_high = bh;
            ic->burn_retry_count = (payload[4] > 1u) ? 1u : payload[4];
            BSU_Emulator_SetIgniterConfigByAddr(h_adr, l_adr, ic->disable_sc_check, 1000);
        }
        break;
    case 2: /* legacy */
        if (len >= 2) {
            BSU_Emulator_SetIgniterConfigByAddr(h_adr, l_adr, ic->disable_sc_check, 1000);
        }
        break;
    default: break;
    }
}

static void handle_dpt_command(uint8_t h_adr, uint8_t l_adr, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (l_adr == 0u)
        return;

    int cfg_idx = find_cfg_device_index_by_hadr(h_adr);
    if (cfg_idx < 0)
        return;

    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    MKUCfg *m = &cfg->CfgDevices[cfg_idx];
    uint8_t slot = (uint8_t)(l_adr - 1u);
    if (slot >= NUM_DEV_IN_MCU)
        return;
    if (m->VDtype[slot] != DEVICE_DPT_TYPE &&
        m->VDtype[slot] != DEVICE_LSWITCH_TYPE &&
        m->VDtype[slot] != DEVICE_BUTTON_TYPE)
        return;

    DeviceDPTConfig *dc = (DeviceDPTConfig *)m->Devices[slot].reserv;
    DeviceButtonConfig *bc = (DeviceButtonConfig *)m->Devices[slot].reserv;

    switch (cmd) {
    case 12: /* Настройка порога MAX в градусах (device_lib) */
    case 2:  /* legacy */
        if (len >= 2) {
            uint16_t v = payload[0] | ((uint16_t)payload[1] << 8);
            if (v > 0) {
                dc->max_fire_threshold_c = v;
            }
        }
        break;
    case 3: /* legacy: use_max */
        if (len >= 1) {
            dc->use_max = payload[0] ? 1 : 0;
        }
        break;
    case 13: /* Время фильтрации состояния линии, мс (device_lib) */
    case 4:  /* legacy */
        if (len >= 2) {
            uint16_t v = payload[0] | ((uint16_t)payload[1] << 8);
            if (v > 0) {
                dc->state_change_delay_ms = v;
            }
        }
        break;
    case 14: /* mode (для DPT/LSWITCH) */
        if (len >= 1) {
            dc->mode = payload[0];
        }
        break;
    case 15: /* button_kind (для BUTTON) */
        if (m->VDtype[slot] == DEVICE_BUTTON_TYPE && len >= 1 && payload[0] <= DeviceButtonKind_FireZone) {
            bc->button_kind = payload[0];
        }
        break;
    case 16: /* zones[7] (для BUTTON) */
        if (m->VDtype[slot] == DEVICE_BUTTON_TYPE && len >= 7) {
            memcpy(bc->zones, payload, 7u);
        }
        break;
    case 17: /* normal_closed (для BUTTON) */
        if (m->VDtype[slot] == DEVICE_BUTTON_TYPE && len >= 1) {
            bc->normal_closed = payload[0] ? 1u : 0u;
        }
        break;
    default:
        break;
    }

    BSU_Emulator_SetDPTConfigByAddr(h_adr, l_adr, dc->state_change_delay_ms, dc->mode);
}

static void handle_relay_command(uint8_t h_adr, uint8_t l_adr, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (l_adr == 0u)
        return;

    int cfg_idx = find_cfg_device_index_by_hadr(h_adr);
    if (cfg_idx < 0)
        return;

    PPKYCfg *cfg = (PPKYCfg *)LocalConfig;
    MKUCfg *m = &cfg->CfgDevices[cfg_idx];
    uint8_t slot = (uint8_t)(l_adr - 1u);
    if (slot >= NUM_DEV_IN_MCU || m->VDtype[slot] != DEVICE_RELAY_TYPE)
        return;

    DeviceRelayConfig *rc = (DeviceRelayConfig *)m->Devices[slot].reserv;

    if (cmd == 10u) {
        uint8_t desired = 0u;
        if (len >= 1u && (payload[0] == 0u || payload[0] == 1u)) {
            desired = payload[0];
        } else {
            desired = rc->initial_state ? 0u : 1u;
        }
        rc->initial_state = desired;
        BSU_Emulator_SetRelayStateByAddr(h_adr, l_adr, desired);
    } else if (cmd == 11u) {
        if (len >= 1u) {
            rc->mode = (payload[0] > 3u) ? 3u : payload[0];
        }
    } else if (cmd == 12u) {
        if (len >= 1u) {
            rc->initial_state = (payload[0] != 0u) ? 1u : 0u;
            BSU_Emulator_SetRelayStateByAddr(h_adr, l_adr, rc->initial_state);
        }
    } else if (cmd == 13u) {
        if (len >= 1u) {
            rc->persist_state_enabled = (payload[0] != 0u) ? 1u : 0u;
        }
    }
}

void BSU_Backend_Init(void)
{
    if (cfg_flash_load_saved(LocalConfig)) {
        /* валидная сохраненная копия уже загружена в LocalConfig */
    } else {
        fill_default_config();
        (void)cfg_flash_store_local();
    }
    backend_apply_runtime_cfg();
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
        if (cmd == ServiceCmd_SetSystemTime && payload_len >= 6u) {
            BSU_Emulator_SetSystemTimeBcd(payload);
            return;
        }
        if (cmd >= ServiceCmd_GetConfigSize && cmd <= ServiceCmd_StartSetConfig) {
            config_service_cmd(cmd, payload);
        }
        return;
    }

    if (id.field.d_type == DEVICE_IGNITER_TYPE) {
        handle_igniter_command(id.field.h_adr, id.field.l_adr, cmd, payload, payload_len);
        return;
    }
    if (id.field.d_type == DEVICE_DPT_TYPE || id.field.d_type == DEVICE_LSWITCH_TYPE) {
        handle_dpt_command(id.field.h_adr, id.field.l_adr, cmd, payload, payload_len);
        return;
    }
    if (id.field.d_type == DEVICE_RELAY_TYPE) {
        handle_relay_command(id.field.h_adr, id.field.l_adr, cmd, payload, payload_len);
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
    if (cfg_flash_store_local() == HAL_OK) {
        backend_apply_runtime_cfg();
    }
}

void BSU_DefaultConfig(void)
{
    fill_default_config();
}

const uint8_t *BSU_Backend_GetLocalConfig(void)
{
    return LocalConfig;
}
