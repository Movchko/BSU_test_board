/**
 * bsu_emulator.h - Эмуляция PPKY, MKU и виртуальных устройств.
 */

#ifndef BSU_EMULATOR_H_
#define BSU_EMULATOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSU_Emulator_Init(void);
void BSU_Emulator_Tick1ms(void);
void BSU_Emulator_Process(void);
/** Пауза эмулятора на ms мс — при чтении конфига не забивать очередь */
void BSU_Emulator_PauseFor(uint32_t ms);
void BSU_Emulator_SetIgniterConfigByAddr(uint8_t h_adr, uint8_t l_adr, uint8_t disable_sc_check, uint16_t start_duration_ms);
void BSU_Emulator_SetDPTConfigByAddr(uint8_t h_adr, uint8_t l_adr, uint16_t speed, uint8_t direction);
void BSU_Emulator_SetRelayStateByAddr(uint8_t h_adr, uint8_t l_adr, uint8_t desired_state);

#ifdef __cplusplus
}
#endif

#endif /* BSU_EMULATOR_H_ */
