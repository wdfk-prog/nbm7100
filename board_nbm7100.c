/**
 * @File Name: board_nbm7100.c
 * @brief  
 * @Author : 
 * @Version : 1.0
 * @Creat Date : 2023-12-04
 * 
 * @copyright Copyright (c) 
 * @par 修改日志:
 * Date           Version     Author  Description
 * 2023-12-04     v1.0        huagnly 内容
 */
/* Includes ------------------------------------------------------------------*/
#include "board_nbm7100.h"
/* Private includes ----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
#if NBM7100_TYPE == NBM7110A
/**
 * @brief  I2C device structure definition
 * @note   None
 */
typedef struct{
    I2c_t       obj;        //I2C对象
    I2cId_t     Id;         //I2C编号
    PinNames    scl;        //SCL引脚
    PinNames    sda;        //SDA引脚
    struct
    {
        Gpio_t          obj;
        PinNames        pin;
    }rdy;                   //RDY引脚
    struct
    {
        Gpio_t          obj;
        PinNames        pin;
        GPIO_PinState   enable;
    }start;                 //start引脚
}nbm7100_i2c_t;
#elif NBM7100_TYPE == NBM7110B
/**
 * @brief  SPI device structure definition
 * @note   None
 */
typedef struct 
{
    SPI_HandleTypeDef *hspi;
    struct
    {
        Gpio_t          obj;
        PinNames        pin;
        GPIO_PinState   enable;
    }nss;
    struct
    {
        Gpio_t          obj;
        PinNames        pin;
    }rdy;                   //RDY引脚
}nbm7100_spi_t;
#endif /* NBM7100_TYPE == NBM7110A */

typedef enum
{
    RDY_NOT_READY = 0,
    RDY_READY,
    RDY_FAILED,
    RDY_SUCCESS,
    RDY_MAX,
}nbm7100_rdy_t;
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *nbm7100_vcap_string[] = 
{
    [NBM7100_VCAP_1_40V] = "1.40V",
    [NBM7100_VCAP_1_51V] = "1.51V",
    [NBM7100_VCAP_1_80V] = "1.80V",
    [NBM7100_VCAP_2_19V] = "2.19V",
    [NBM7100_VCAP_2_40V] = "2.40V",
    [NBM7100_VCAP_2_60V] = "2.60V",
    [NBM7100_VCAP_2_79V] = "2.79V",
    [NBM7100_VCAP_3_01V] = "3.01V",
    [NBM7100_VCAP_3_20V] = "3.20V",
    [NBM7100_VCAP_3_41V] = "3.41V",
    [NBM7100_VCAP_3_61V] = "3.61V",
    [NBM7100_VCAP_3_99V] = "3.99V",
    [NBM7100_VCAP_4_39V] = "4.39V",
    [NBM7100_VCAP_4_80V] = "4.80V",
    [NBM7100_VCAP_5_21V] = "5.21V",
    [NBM7100_VCAP_5_59V] = "5.59V",
    [NBM7100_VCAP_5_91V] = "5.91V",
    [NBM7100_VCAP_6_02V] = "6.02V",
    [NBM7100_VCAP_6_40V] = "6.40V",
    [NBM7100_VCAP_6_55V] = "6.55V",
    [NBM7100_VCAP_6_80V] = "6.80V",
    [NBM7100_VCAP_7_14V] = "7.14V",
    [NBM7100_VCAP_7_23V] = "7.23V",
    [NBM7100_VCAP_7_68V] = "7.68V",
    [NBM7100_VCAP_8_19V] = "8.19V",
    [NBM7100_VCAP_8_65V] = "8.65V",
    [NBM7100_VCAP_9_10V] = "9.10V",
    [NBM7100_VCAP_9_53V] = "9.53V",
    [NBM7100_VCAP_9_91V] = "9.91V",
    [NBM7100_VCAP_10_33V] = "10.33V",
    [NBM7100_VCAP_10_69V] = "10.69V",
    [NBM7100_VCAP_11_07V] = "11.07V",
};
static nbm7100_driver_t nbm7100 = {0};
static nbm7100_mode_e g_nbm7100_mode = NBM7100_MODE_MAX;
static nbm7100_rdy_t g_nbm7100_status = RDY_SUCCESS;
static nbm7100_rdy_t g_nbm7100_rdy = RDY_NOT_READY;
#if NBM7100_TYPE == NBM7110A
static nbm7100_i2c_t nbm7100_i2c = 
{
    .Id = NBM7100_I2C,
    .scl = NBM7100_SCL_PIN,
    .sda = NBM7100_SDA_PIN,
    .rdy = 
    {
        .pin = NBM7100_RDY_PIN,
    },
    .start = 
    {
        .pin = NBM7100_START_PIN,
        .enable = GPIO_PIN_SET,
    },
};
#elif NBM7100_TYPE == NBM7110B
static nbm7100_spi_t nbm7100_spi = 
{
    .hspi = &hspi1,
    .nss = 
    {
        .pin = BOARD_NBM7100_NSS_PIN,
        .enable = GPIO_PIN_RESET,
    },
    .rdy = 
    {
        .pin = BOARD_NBM7100_RDY_PIN,
    },
};
#endif /* (NBM7100_TYPE == NBM7110A) */
static nbm7100_cfg_t nbm7100_cfg = 
{
    .type = NBM7100_TYPE,
    .adr_en = false,
    .control = 
    {
        .mode = NBM7100_MODE_CONTINUOUS,
        .optimizer = 
        {
            .profile = NBM7100_PROFILE_FIXED_MODE,
            .fixed_charge = NBM7100_END_CHARGE_5_2V,
            .margin_voltage = NBM7100_MARGIN_3_41V,
        },
        .output = 
        {
            .output_voltage = NBM7100_OUTPUT_3_0V,
            .VDH_cfg        = VDH_CFG_VDH_VBT,
        },
        .charge = 
        {
            .max_storage    = NBM7100_MAX_STORAGE_9_9V,
            .charge_current = NBM7100_CHARGE_CURRENT_8MA,
        },
        .battery = 
        {
            .input_threshold = NBM7100_MIN_INPUT_2_4V,
            .early_warning_enable = false,
            .warning_voltage = NBM7100_WARNING_VOLTAGE_2_4V,
        },
    },
};
/* Private function prototypes -----------------------------------------------*/
static nbm7100_status_e nbm7100_init(void);
static nbm7100_status_e nbm7100_deinit(void);
static nbm7100_status_e nbm7100_read(uint8_t addr, uint8_t *buf, uint16_t len);
static nbm7100_status_e nbm7100_write(uint8_t addr, uint8_t *buf, uint16_t len);
static nbm7100_status_e nbm7100_delay_ms(uint32_t ms);
static nbm7100_ops_t nbm7100_ops = {
    .init           = nbm7100_init,
    .deinit         = nbm7100_deinit,
    .read           = nbm7100_read,
    .write          = nbm7100_write,
    .delay_ms       = nbm7100_delay_ms,
    .lock           = NULL,
    .unlock         = NULL,
};
/* Private user code ---------------------------------------------------------*/
#ifdef NBM7100_RDY_PIN
/**
 * @brief  RDY引脚中断回调函数
 * @note   仅上升沿触发
 * @param  context: 用户数据
 */
static void nbm7100_rdy_event(void* context)
{
    UNUSED(context);
    printf_debug("[nbm7100]RDY:%lu\r\n", GpioRead(&nbm7100_i2c.rdy.obj));
}
#endif // NBM7100_RDY_PIN
/**
 * @brief  RDY引脚初始化
 * @note   None
 * @param  mode: true: 初始化 false: 释放
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_rdy_init(bool mode)
{
#ifdef NBM7100_RDY_PIN
#if (NBM7100_TYPE == NBM7110A)
    if(mode == true) {
        GpioInit(&nbm7100_i2c.rdy.obj, nbm7100_i2c.rdy.pin, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
        GpioSetInterrupt(&nbm7100_i2c.rdy.obj, IRQ_RISING_FALLING_EDGE, IRQ_VERY_HIGH_PRIORITY, nbm7100_rdy_event);
    } else {
        GpioInit(&nbm7100_i2c.rdy.obj, nbm7100_i2c.rdy.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    }
#elif (NBM7100_TYPE == NBM7110B)
    if(mode == true) {
        GpioInit(&nbm7100_spi.rdy.obj, nbm7100_spi.rdy.pin, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    } else {
        GpioInit(&nbm7100_spi.rdy.obj, nbm7100_spi.rdy.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    }
#endif /* (NBM7100_TYPE == NBM7110A) */
#endif // NBM7100_RDY_PIN
    return NBM7100_OK;
}
/**
 * @brief  START引脚初始化
 * @note   None
 * @param  mode: true: 初始化 false: 释放
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_start_init(bool mode)
{
#ifdef NBM7100_START_PIN
    if(mode == true) {
        GpioInit(&nbm7100_i2c.start.obj, nbm7100_i2c.start.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, !nbm7100_i2c.start.enable);
    } else {
        GpioInit(&nbm7100_i2c.start.obj, nbm7100_i2c.start.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    }
#endif // NBM7100_START_PIN
    return NBM7100_OK;
}
/**
 * @brief  初始化引脚与驱动资源
 * @note   None
 */
static nbm7100_status_e nbm7100_init(void)
{
    nbm7100_rdy_init(true);
#if (NBM7100_TYPE == NBM7110A)
    I2cInit(&nbm7100_i2c.obj, nbm7100_i2c.Id, nbm7100_i2c.sda, nbm7100_i2c.scl);
    // nbm7100_start_init(true);
#elif (NBM7100_TYPE == NBM7110B)
    MX_SPI1_Init();
    GpioInit(&nbm7100_spi.nss.obj, nbm7100_spi.nss.pin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, !nbm7100_spi.nss.enable);
#endif /* (NBM7100_TYPE == NBM7110A) */
    return NBM7100_OK;
}
/**
 * @brief  释放引脚与驱动资源
 * @note   None
 */
static nbm7100_status_e nbm7100_deinit(void)
{
#if (NBM7100_TYPE == NBM7110A)
    I2cDeInit(&nbm7100_i2c.obj);
    nbm7100_start_init(false);
#elif (NBM7100_TYPE == NBM7110B)
    HAL_SPI_DeInit(nbm7100_spi.hspi);
    GpioInit(&nbm7100_spi.nss.obj, nbm7100_spi.nss.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0);
#endif /* (NBM7100_TYPE == NBM7110A) */
    return NBM7100_OK;
}
/**
 * @brief  读取数据
 * @note   None
 * @param  *buf: 数据缓存
 * @param  len: 数据长度
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_read(uint8_t addr, uint8_t *buf, uint16_t len)
{
    if(buf == NULL || len == 0) {
        return NBM7100_INVALID;
    }
#if (NBM7100_TYPE == NBM7110A)
    uint8_t ret = I2cReceive(&nbm7100_i2c.obj, addr, buf, len);
    if(ret != HAL_OK) {
        printf_debug("[nbm7100][read]error:0X%02X\r\n", ret);
        return NBM7100_ERROR;
    } else {
        return NBM7100_OK;
    }
#elif (NBM7100_TYPE == NBM7110B)
    if(HAL_SPI_Receive(nbm7100_spi.hspi, buf, len, HAL_MAX_DELAY) != HAL_OK) {
        return NBM7100_ERROR;
    } else {
        return NBM7100_OK;
    }
#endif /* (NBM7100_TYPE == NBM7110A) */
}
/**
 * @brief  写入数据
 * @note   None
 * @param  *buf: 数据缓存
 * @param  len: 数据长度
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_write(uint8_t addr, uint8_t *buf, uint16_t len)
{
    if(buf == NULL || len == 0) {
        return NBM7100_INVALID;
    }
#if (NBM7100_TYPE == NBM7110A)
    uint8_t ret =  I2cTransmit(&nbm7100_i2c.obj, addr, buf, len);
    if(ret != HAL_OK) {
        printf_debug("[nbm7100][write]error:0X%02X\r\n", ret);
        return NBM7100_ERROR;
    } else {
        return NBM7100_OK;
    }
#elif (NBM7100_TYPE == NBM7110B)
    if(HAL_SPI_Transmit(nbm7100_spi.hspi, buf, len, HAL_MAX_DELAY) != HAL_OK) {
        return NBM7100_ERROR;
    } else {
        return NBM7100_OK;
    }
#endif /* (NBM7100_TYPE == NBM7110A) */
}
/**
 * @brief  延时函数
 * @note   None
 * @param  ms: 延时时间
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
    return NBM7100_OK;
}
/**
 * @brief  未接电池,打印将会是默认值
 * @note   None
 */
void nbm7100_printf(void)
{
    nbm7100_info_t info = {0};
    nbm7100_read_info(&nbm7100, &info);

    printf_debug("ready:%u,alarm_low_voltage:%u,alarm_early_warning:%u,alarm_vdh_output:%u\r\n", 
        info.status.data.ready, info.status.data.alarm_low_voltage, info.status.data.alarm_early_warning, info.status.data.alarm_vdh_output);
    printf_debug("electric_quantity:%luuAh,actual_Vcap:%s,target_voltage:%s\r\n", info.electric_quantity, nbm7100_vcap_string[info.actual_Vcap], nbm7100_vcap_string[info.target_voltage]);
    //! 没有接入电池上电,配置将会被重置为默认值,重启防止电容充电过高
    if(info.target_voltage != NBM7100_VCAP_5_21V) {
        printf_debug("[nbm7100] read error\r\n");
        device_restart();
    }
}
/**
 * @brief  nbm7100初始化
 * @note   None
 */
void board_nbm7100_init(void)
{
    nbm7100_status_e status = NBM7100_OK;

    status = nbm7100_driver_init(&nbm7100, &nbm7100_ops);
    if(status != NBM7100_OK) {
        printf_debug("[nbm7100][init]error:0X%02X\r\n", status);
        return;
    }

    status = nbm7100_driver_cfg(&nbm7100, &nbm7100_cfg);
    if(status != NBM7100_OK) {
        printf_debug("[nbm7100][cfg]error:0X%02X\r\n", status);
        device_restart();
        return;
    }

    status = nbm7100_driver_verify(&nbm7100);
    if(status != NBM7100_OK) {
        printf_debug("[nbm7100][verify]error:0X%02X\r\n", status);
        device_restart();
        return;
    }
    printf_debug("[nbm7100][init]ok\r\n");
    //初始化充电电流8MA,后续再次充电变为4MA充电
    g_nbm7100_mode = NBM7100_MODE_CONTINUOUS;
    if(nbm7100_send_mode(&nbm7100, NBM7100_MODE_CONTINUOUS) != NBM7100_OK) {
        printf_debug("[nbm7100][mode]error:0X%02X\r\n", status);
        device_restart();
    } else {
        printf_debug("[nbm7100][mode]0X%02X\r\n", NBM7100_MODE_CONTINUOUS);
    }
    nbm7100_printf();
    // for (uint8_t i = 0; i < sizeof(nbm7100.reg); i++) {
    //     printf_debug("0x%02X ", nbm7100.reg[i]);
    // }
    // printf_debug("\r\n");
}
/**
 * @brief  设置NBM7100模式
 * @note   mode必需在配置完成之后发送,mode发送后立刻进入change状态,此时配置还未生效
 * @param  mode: 模式
 * @retval None
 */
nbm7100_status_e board_nbm7100_set_mode(nbm7100_mode_e mode)
{
    nbm7100_status_e ret = NBM7100_OK;
    if (mode == NBM7100_MODE_FORCE_ACTIVE) {
        if(g_nbm7100_mode == NBM7100_MODE_NONE) {
            ret = nbm7100_send_mode(&nbm7100, NBM7100_MODE_CONTINUOUS);
            if(ret != NBM7100_OK) {
                goto exit;
            }
            HAL_Delay(100);
        }
    }

    ret = nbm7100_send_mode(&nbm7100, mode);
    if(ret != NBM7100_OK) {
        goto exit;
    }

exit:
    printf_debug("[nbm7100][mode]0X%02x\r\n", mode);
    g_nbm7100_mode = mode;

    if(ret != NBM7100_OK) {
        printf_debug("[nbm7100][mode]error:0X%02x\r\n", ret);
        g_nbm7100_status = RDY_FAILED;
    } else {
        g_nbm7100_status = RDY_SUCCESS;
    }
    nbm7100_printf();
    return ret;
}
/**
 * @brief  获取 nbm7100 是否准备
 * @note   仅获取当前状态
 * @retval true: 准备 false: 未准备
 */
bool board_nbm7100_get_rdy(void)
{
    return g_nbm7100_rdy;
}
/**
 * @brief  获取 nbm7100 是否准备
 * @note   是否可以进行大电流操作
 * @retval true: 准备 false: 未准备
 */
bool board_nbm7100_ready(void)
{
    //! 第一次充满电后,需要重新激活一次充电状态再次充满后,才算是准备好
    static bool first = true;
    bool rdy = false;

    if(g_nbm7100_status == RDY_FAILED) {
        printf_debug("[nbm7100]send failed retry\r\n");
        board_nbm7100_set_mode(g_nbm7100_mode);
    } else if(g_nbm7100_status == RDY_SUCCESS) {
        bool flag = false;
        if(GpioRead(&nbm7100_i2c.rdy.obj) == RDY_READY) {
            flag = true;
        }
        if(g_nbm7100_mode == NBM7100_MODE_NONE){
            flag = true;
            printf_debug("[nbm7100]mode into\r\n");
        }
        nbm7100_info_t info = {0};
        nbm7100_read_info(&nbm7100, &info);
        if(info.actual_Vcap >= NBM7100_VCAP_4_80V) {
            flag = true;
            printf_debug("[nbm7100]Vcap into\r\n");
        }
        if(flag == true) {
            if(first == true) {
                first = false;

                nbm7100_status_e ret = NBM7100_OK;
                ret = board_nbm7100_set_mode(NBM7100_MODE_FORCE_ACTIVE);
                if(ret != NBM7100_OK) {
                    goto exit;
                }

                HAL_Delay(100);
                ret = nbm7100_send_charge_current(&nbm7100, NBM7100_CHARGE_CURRENT_4MA);
                if(ret != NBM7100_OK) {
                    goto exit;
                }
                ret = board_nbm7100_set_mode(NBM7100_MODE_CONTINUOUS);
                if(ret != NBM7100_OK) {
                    goto exit;
                }
            }
            rdy = true;
        }
    }
exit:
    g_nbm7100_rdy = rdy;
    return rdy;
}
/**
 * @brief  退出低功耗处理
 * @note   判断充电是否完成,完成则进入待机模式
 * @retval None
 */
void board_nbm7100_lpm_exit(void)
{
    if(g_nbm7100_mode == NBM7100_MODE_CONTINUOUS && GpioRead(&nbm7100_i2c.rdy.obj) == RDY_READY) {
        //充电完成,进入待机模式
        board_nbm7100_set_mode(NBM7100_MODE_NONE);
    }
}
/**
 * @brief  nbm7100单元测试
 * @note   None
 * @param  *buffer: 
 * @param  size: 
 * @retval None
 */
uint8_t board_nbm7100_unit_debug(uint8_t *buffer, uint16_t size)
{
    uint8_t ret = 1;
    if(!strncmp((char *)buffer, "nbm7100_read", strlen("nbm7100_read"))) {
        nbm7100_printf();
    } if(!strncmp((char *)buffer, "nbm7100_set", strlen("nbm7100_set"))) {
        uint32_t mode;
        int ret = sscanf((char *)buffer, "nbm7100_set %lu", &mode);
        if(ret == 1 && mode < NBM7100_MODE_MAX) {
            board_nbm7100_set_mode(mode);
            printf_debug("[debug]nbm7100_set:%lu\r\n", mode);
        } else {
            printf_debug("[error] nbm7100_set error\r\n");
        }
    } else {
        ret = 0;
    }

    return ret;
}