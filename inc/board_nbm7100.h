/**
 * @File Name: board_nbm7100.h
 * @brief  
 * @Author : 
 * @Version : 1.0
 * @Creat Date : 2023-12-04
 * 
 * @copyright Copyright (c) 
 * @par change log:
 * Date           Version     Author    Description
 * 2023-12-04     v1.0        wdfk-prog first version
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_NBM7100_H__
#define __BOARD_NBM7100_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "nbm7100_driver.h"
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ----------------------------------------------*/
void board_nbm7100_init(void);
nbm7100_status_e board_nbm7100_set_mode(nbm7100_mode_e mode);
bool board_nbm7100_get_rdy(void);
bool board_nbm7100_ready(void);
void board_nbm7100_lpm_exit(void);
uint8_t board_nbm7100_unit_debug(uint8_t *buffer, uint16_t size);
void nbm7100_printf(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_NBM7100_H__ */