#ifndef INC_CDC_H_
#define INC_CDC_H_

void sendToCDC(void);
void parseFromCDC(uint8_t *Buf, uint32_t len);
void usbTxComplete(void);

#endif /* INC_CDC_H_ */
