/**
 * bsu_emulator.h - Эмуляция PPKY и MCU Igniter для BSU_test_board
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
void BSU_Emulator_SetIgniterConfig(uint8_t vdev_idx, uint8_t disable_sc_check, uint16_t start_duration_ms);
void BSU_Emulator_SetDPTConfig(uint8_t vdev_idx, uint16_t speed, uint8_t direction);

#ifdef __cplusplus
}
#endif

#endif /* BSU_EMULATOR_H_ */
