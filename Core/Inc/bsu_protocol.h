/**
 * bsu_protocol.h - USB протокол BSU_test_board
 *
 * Формат пакета:
 *   2 байта - преамбула (0xAA55)
 *   2 байта - размер всего пакета (little-endian)
 *   2 байта - тип пакета (0 = CAN)
 *   2 байта - номер пакета
 *   ... полезные данные ...
 *   2 байта - контрольная сумма (16-bit sum)
 *
 * Тип 0 = CAN пакет: 4 байта ID + 8 байт данных
 */

#ifndef BSU_PROTOCOL_H_
#define BSU_PROTOCOL_H_

#include <stdint.h>

#define BSU_PREAMBLE_LO    0x55
#define BSU_PREAMBLE_HI    0xAA

#define BSU_HEADER_SIZE    (2+2+2+2)
#define BSU_CAN_PAYLOAD    (4+8)
#define BSU_CHECKSUM_SIZE  2
#define BSU_MIN_PKT_SIZE   (BSU_HEADER_SIZE + BSU_CHECKSUM_SIZE)
#define BSU_CAN_PKT_SIZE   (BSU_HEADER_SIZE + BSU_CAN_PAYLOAD + BSU_CHECKSUM_SIZE)
#define BSU_RX_BUF_SIZE    256
#define BSU_TX_BUF_SIZE    256

#ifdef __cplusplus
extern "C" {
#endif

void BSU_Protocol_Init(void);
void BSU_Protocol_Rx(uint8_t *buf, uint32_t len);
void BSU_Protocol_SendCan(uint32_t can_id, const uint8_t *data, uint8_t dlc);
/** Вызывать из CDC_TransmitCplt_FS при завершении передачи USB */
void BSU_Protocol_UsbTxComplete(void);
/** Отправка сразу, минуя очередь (для ответов конфигурации — приоритет) */
void BSU_Protocol_SendCanNow(uint32_t can_id, const uint8_t *data, uint8_t dlc);
void BSU_Protocol_Process(void);
uint16_t BSU_Checksum(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSU_PROTOCOL_H_ */
