/**
 * @File Name: board_nbm7100.h
 * @brief  
 * @Author : huangly@milesight.com
 * @Version : 1.0
 * @Creat Date : 2023-12-04
 * 
 * @copyright Copyright (c) 2023 星纵物联科技有限公司
 * @par 修改日志:
 * Date           Version     Author  Description
 * 2023-12-04     v1.0        huagnly 内容
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
void board_nbm7100_set_mode(nbm7100_mode_e mode);
bool board_nbm7100_get_rdy(void);
void board_nbm7100_poll(void);
void nbm7100_printf(void);
uint8_t board_nbm7100_unit_debug(uint8_t *buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_NBM7100_H__ */