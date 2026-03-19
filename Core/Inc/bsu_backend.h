/**
 * bsu_backend.h - BSU_test_board: инклюды device_lib + объявления BSU.
 * Использует backend.h, device_config.h, service.h из device_lib.
 */

#ifndef BSU_BACKEND_H_
#define BSU_BACKEND_H_

#include "backend.h"
#include "device_config.h"
#include "service.h"

#ifdef __cplusplus
extern "C" {
#endif

void BSU_Backend_Init(void);
void BSU_Backend_ProcessConfig(uint32_t can_id, const uint8_t *data, uint8_t len);

uint32_t BSU_GetConfigSize(void);
uint32_t BSU_GetConfigWord(uint16_t num_word);
void BSU_SetConfigWord(uint16_t num_word, uint32_t word);
void BSU_SaveConfig(void);
void BSU_DefaultConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* BSU_BACKEND_H_ */
