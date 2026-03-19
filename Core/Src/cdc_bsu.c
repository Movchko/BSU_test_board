/**
 * cdc_bsu.c - CDC интерфейс для BSU_test_board
 */

#include "bsu_protocol.h"

void parseFromCDC(uint8_t *Buf, uint32_t len)
{
    BSU_Protocol_Rx(Buf, len);
}

void sendToCDC(void)
{
    BSU_Protocol_Process();
}

void usbTxComplete(void)
{
    BSU_Protocol_UsbTxComplete();
}
