/**
 * bsu_protocol.c - Реализация USB протокола BSU_test_board
 * Очереди приёма/отправки по аналогии с конвертером USB-CAN.
 */

#include "bsu_protocol.h"
#include "bsu_backend.h"
#include "main.h"
#include <string.h>

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
#define USBD_OK   0
#define BSU_PKT_TYPE_CAN  0

typedef enum {
    BSU_RX_WAIT_PREAMBLE_0,
    BSU_RX_WAIT_PREAMBLE_1,
    BSU_RX_WAIT_SIZE_LO,
    BSU_RX_WAIT_SIZE_HI,
    BSU_RX_WAIT_TYPE_LO,
    BSU_RX_WAIT_TYPE_HI,
    BSU_RX_WAIT_SEQ_LO,
    BSU_RX_WAIT_SEQ_HI,
    BSU_RX_BODY,
    BSU_RX_CHECKSUM_LO,
    BSU_RX_CHECKSUM_HI
} BSU_RxState_t;

static BSU_RxState_t rx_state = BSU_RX_WAIT_PREAMBLE_0;
static uint8_t rx_buf[BSU_RX_BUF_SIZE];
static uint16_t rx_size;
static uint16_t rx_type;
static uint16_t rx_seq;
static uint16_t rx_pos;
static uint16_t rx_total;
static uint16_t rx_checksum_acc;
static uint8_t  rx_recv_crc_lo;

#define TX_QUEUE_SIZE  128
#define RX_QUEUE_SIZE  16
#define USB_TX_TIMEOUT_MS  50

typedef struct {
    uint32_t can_id;
    uint8_t  data[8];
    uint8_t  dlc;
} bsu_packet_t;

static bsu_packet_t tx_queue[TX_QUEUE_SIZE];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;

/* Очередь приёма USB: парсер кладёт пакеты, main loop обрабатывает */
static bsu_packet_t usb_rx_queue[RX_QUEUE_SIZE];
static volatile uint8_t usb_rx_head = 0;
static volatile uint8_t usb_rx_tail = 0;

/* Отправка USB: busy + таймаут 50 мс (как в конвертере) */
static volatile uint8_t usb_tx_busy = 0;
static uint32_t usb_tx_start_tick = 0;

/* Приоритетный слот: ответ конфигурации при занятом CDC — отправить первым в Process */
static uint8_t priority_has = 0;
static uint32_t priority_can_id;
static uint8_t priority_data[8];
static uint8_t priority_dlc;

uint16_t BSU_Checksum(const uint8_t *data, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

static void push_rx_packet(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    uint8_t next = (usb_rx_head + 1) % RX_QUEUE_SIZE;
    if (next == usb_rx_tail)
        return;
    usb_rx_queue[usb_rx_head].can_id = can_id;
    memcpy(usb_rx_queue[usb_rx_head].data, data, 8);
    usb_rx_queue[usb_rx_head].dlc = dlc > 8 ? 8 : dlc;
    usb_rx_head = next;
}

static void reset_parser(void)
{
    rx_state = BSU_RX_WAIT_PREAMBLE_0;
    rx_pos = 0;
}

void BSU_Protocol_Init(void)
{
    reset_parser();
    tx_head = tx_tail = 0;
    usb_rx_head = usb_rx_tail = 0;
    usb_tx_busy = 0;
    priority_has = 0;
}

void BSU_Protocol_Rx(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint8_t b = buf[i];

        switch (rx_state) {
        case BSU_RX_WAIT_PREAMBLE_0:
            if (b == BSU_PREAMBLE_LO)
                rx_state = BSU_RX_WAIT_PREAMBLE_1;
            break;

        case BSU_RX_WAIT_PREAMBLE_1:
            if (b == BSU_PREAMBLE_HI) {
                rx_state = BSU_RX_WAIT_SIZE_LO;
                rx_pos = 0;
                rx_checksum_acc = BSU_PREAMBLE_LO + BSU_PREAMBLE_HI;
            } else
                rx_state = BSU_RX_WAIT_PREAMBLE_0;
            break;

        case BSU_RX_WAIT_SIZE_LO:
            rx_size = b;
            rx_checksum_acc += b;
            rx_state = BSU_RX_WAIT_SIZE_HI;
            break;

        case BSU_RX_WAIT_SIZE_HI:
            rx_size |= (uint16_t)b << 8;
            rx_checksum_acc += b;
            rx_state = BSU_RX_WAIT_TYPE_LO;
            break;

        case BSU_RX_WAIT_TYPE_LO:
            rx_type = b;
            rx_checksum_acc += b;
            rx_state = BSU_RX_WAIT_TYPE_HI;
            break;

        case BSU_RX_WAIT_TYPE_HI:
            rx_type |= (uint16_t)b << 8;
            rx_checksum_acc += b;
            rx_state = BSU_RX_WAIT_SEQ_LO;
            break;

        case BSU_RX_WAIT_SEQ_LO:
            rx_seq = b;
            rx_checksum_acc += b;
            rx_state = BSU_RX_WAIT_SEQ_HI;
            break;

        case BSU_RX_WAIT_SEQ_HI:
            rx_seq |= (uint16_t)b << 8;
            rx_checksum_acc += b;
            rx_total = rx_size - BSU_HEADER_SIZE - BSU_CHECKSUM_SIZE;
            if (rx_total > BSU_RX_BUF_SIZE || rx_size < BSU_MIN_PKT_SIZE) {
                reset_parser();
                break;
            }
            rx_state = BSU_RX_BODY;
            rx_pos = 0;
            break;

        case BSU_RX_BODY:
            rx_buf[rx_pos++] = b;
            rx_checksum_acc += b;
            if (rx_pos >= rx_total) {
                rx_state = BSU_RX_CHECKSUM_LO;
            }
            break;

        case BSU_RX_CHECKSUM_LO:
            rx_recv_crc_lo = b;
            rx_state = BSU_RX_CHECKSUM_HI;
            break;

        case BSU_RX_CHECKSUM_HI: {
            uint16_t recv_crc = rx_recv_crc_lo | ((uint16_t)b << 8);
            uint16_t calc_crc = rx_checksum_acc & 0xFFFF;

            if (calc_crc == recv_crc && rx_type == BSU_PKT_TYPE_CAN && rx_total >= 12) {
                uint32_t can_id = rx_buf[0] | ((uint32_t)rx_buf[1] << 8) |
                                  ((uint32_t)rx_buf[2] << 16) | ((uint32_t)rx_buf[3] << 24);
                push_rx_packet(can_id, &rx_buf[4], 8);
            }
            reset_parser();
            break;
        }
        default:
            reset_parser();
            break;
        }
    }
}

void BSU_Protocol_SendCan(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > 8) dlc = 8;

    uint8_t next = (tx_head + 1) % TX_QUEUE_SIZE;
    if (next == tx_tail)
        return;

    tx_queue[tx_head].can_id = can_id;
    memcpy(tx_queue[tx_head].data, data, 8);
    tx_queue[tx_head].dlc = dlc;
    tx_head = next;
}

/* Статический буфер: CDC передаёт асинхронно, данные должны жить до завершения */
static uint8_t tx_pkt_buf[BSU_CAN_PKT_SIZE];

static uint16_t build_tx_packet(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > 8) dlc = 8;

    uint8_t *pkt = tx_pkt_buf;
    uint16_t pos = 0;

    pkt[pos++] = BSU_PREAMBLE_LO;
    pkt[pos++] = BSU_PREAMBLE_HI;

    uint16_t pkt_size = BSU_CAN_PKT_SIZE;
    pkt[pos++] = (uint8_t)(pkt_size & 0xFF);
    pkt[pos++] = (uint8_t)(pkt_size >> 8);

    pkt[pos++] = 0;  /* тип пакета: 0 = CAN */
    pkt[pos++] = 0;

    static uint16_t seq = 0;
    pkt[pos++] = (uint8_t)(seq & 0xFF);
    pkt[pos++] = (uint8_t)(seq >> 8);
    seq++;

    pkt[pos++] = (uint8_t)(can_id & 0xFF);
    pkt[pos++] = (uint8_t)((can_id >> 8) & 0xFF);
    pkt[pos++] = (uint8_t)((can_id >> 16) & 0xFF);
    pkt[pos++] = (uint8_t)((can_id >> 24) & 0xFF);

    memcpy(&pkt[pos], data, 8);
    pos += 8;

    uint16_t crc = BSU_Checksum(pkt, pos);
    pkt[pos++] = (uint8_t)(crc & 0xFF);
    pkt[pos++] = (uint8_t)(crc >> 8);

    return pos;
}

void BSU_Protocol_SendCanNow(uint32_t can_id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > 8) dlc = 8;

    uint16_t pos = build_tx_packet(can_id, data, dlc);
    if (CDC_Transmit_FS(tx_pkt_buf, pos) == USBD_OK) {
        usb_tx_busy = 1;
        usb_tx_start_tick = HAL_GetTick();
        return;
    }

    /* CDC занят — в приоритетный слот для Process */
    priority_has = 1;
    priority_can_id = can_id;
    memcpy(priority_data, data, 8);
    priority_dlc = dlc;
}

static void send_one_packet(void)
{
    if (tx_head == tx_tail)
        return;

    uint32_t id = tx_queue[tx_tail].can_id;
    const uint8_t *data = tx_queue[tx_tail].data;
    uint8_t dlc = tx_queue[tx_tail].dlc;

    uint16_t pos = build_tx_packet(id, data, dlc);

    if (CDC_Transmit_FS(tx_pkt_buf, pos) == USBD_OK) {
        tx_tail = (tx_tail + 1) % TX_QUEUE_SIZE;
        usb_tx_busy = 1;
        usb_tx_start_tick = HAL_GetTick();
    }
}

void BSU_Protocol_UsbTxComplete(void)
{
    usb_tx_busy = 0;
}

void BSU_Protocol_Process(void)
{
    /* 1. Обработка очереди приёма USB — по одному пакету за итерацию (чтобы не перезаписать приоритетный ответ) */
    if (usb_rx_head != usb_rx_tail) {
        bsu_packet_t *p = &usb_rx_queue[usb_rx_tail];
        BSU_Backend_ProcessConfig(p->can_id, p->data, p->dlc);
        usb_rx_tail = (usb_rx_tail + 1) % RX_QUEUE_SIZE;
    }

    /* 2. Таймаут: если CDC_TransmitCplt не вызвался за 50 мс — считаем передачу завершённой */
    if (usb_tx_busy && (HAL_GetTick() - usb_tx_start_tick) > USB_TX_TIMEOUT_MS)
        usb_tx_busy = 0;

    if (usb_tx_busy)
        return;

    /* 3. Приоритетный ответ конфигурации (если CDC был занят при SendCanNow) */
    if (priority_has) {
        uint16_t pos = build_tx_packet(priority_can_id, priority_data, priority_dlc);
        if (CDC_Transmit_FS(tx_pkt_buf, pos) == USBD_OK) {
            priority_has = 0;
            usb_tx_busy = 1;
            usb_tx_start_tick = HAL_GetTick();
            return;
        }
    }

    /* 4. Отправка из очереди TX */
    send_one_packet();
}
