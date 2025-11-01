/**
 * @File Name: nbm7100_driver.c
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
/* Includes ------------------------------------------------------------------*/
#include "nbm7100_driver.h"
/* Private includes ----------------------------------------------------------*/
#include "string.h"
#include "stdio.h"
/* Private macro -------------------------------------------------------------*/
#define BIT(n)                     (1 << (n))                   //返回二进制数值中第 n 位所对应的十六进制表示值
#define NBM7100_CLEAR_BIT(x, bit)  (x  &= ~(1 << bit))	        /* 清零第bit位 */
#define NBM7100_GET_BIT(x, bit)    ((x &   (1 << bit)) >> bit)	/* 获取第bit位 */
#define NBM7100_SET_BIT(x, bit)    (x  |=  (1 << bit))	        /* 置位第bit位 */
/* Private define ------------------------------------------------------------*/
//I2C从机地址 ADR pin0:0x2E, ADR pin1:0x2F
#define I2C_ADDR_ADR0       0x2E
#define I2C_ADDR_ADR1       0x2F
/* Read only-------------------*/
#define REGISTER_STATUS     0x00
#define REGISTER_CHENGY     0X01
#define REGISTER_VCAP       0X05
#define REGISTER_VCHEND     0X06
/* Read Write------------------*/
#define REGISTER_PROFILE    0X07
#define REGISTER_COMMAND    0X08
#define REGISTER_SET1       0X09
#define REGISTER_SET2       0X0A
#define REGISTER_SET3       0X0B
#define REGISTER_SET4       0X0C
#define REGISTER_SET5       0X0D

#define REGISTER_READ_LEN   (REGISTER_SET5 + 1)
#define REGISTER_WRITE_LEN  (REGISTER_SET5 - REGISTER_PROFILE + 1)
/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
static nbm7100_status_e nbm7100_read_reg(nbm7100_driver_t *dev, uint8_t *reg, uint8_t len)
{
    if(reg == NULL || len != REGISTER_READ_LEN) {
        return NBM7100_INVALID;
    }

    nbm7100_status_e ret = NBM7100_OK;

    uint8_t slave_addr = 0;
    if(dev->cfg->type == NBM7110A) {
        slave_addr = I2C_ADDR_ADR0 | dev->cfg->adr_en;
    } else if (dev->cfg->type == NBM7110B) {
        slave_addr = 0;
    } else {
        return NBM7100_INVALID;
    }

    if(dev->ops->lock != NULL) {
        if(dev->ops->lock() != NBM7100_OK) {
            return NBM7100_LOCK;
        }
    }

    ret = dev->ops->read(slave_addr, reg, REGISTER_READ_LEN);
    if (ret != NBM7100_OK) {
        goto exit;
    }

exit:
    if(dev->ops->unlock != NULL) {
        if(dev->ops->unlock() != NBM7100_OK) {
            return NBM7100_LOCK;
        }
    }
    return ret;
}
/**
 * @brief  nbm7100写入寄存器
 * @note   根据芯片类型选择不同的写入方式
 * @param  *dev: 驱动句柄
 * @param  reg:  寄存器地址
 * @param  value: 寄存器值
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_write_register(nbm7100_driver_t *dev, uint8_t reg, uint8_t value)
{
    if(reg > REGISTER_SET5 || reg < REGISTER_PROFILE) {
        return NBM7100_INVALID;
    }

    uint8_t buf[2] = {0};
    uint8_t slave_addr = 0;

    if(dev->cfg->type == NBM7110A) {
        slave_addr = I2C_ADDR_ADR0 | dev->cfg->adr_en;
    } else if (dev->cfg->type == NBM7110B) {
        slave_addr = 0;
    } else {
        return NBM7100_INVALID;
    }

    buf[0] = reg;
    buf[1] = value;

    if(dev->ops->lock != NULL) {
        if(dev->ops->lock() != NBM7100_OK) {
            return NBM7100_LOCK;
        }
    }

    nbm7100_status_e ret = dev->ops->write(slave_addr, buf, sizeof(buf));

    if(dev->ops->unlock != NULL) {
        if(dev->ops->unlock() != NBM7100_OK) {
            return NBM7100_LOCK;
        }
    }

    return ret;
}
/* 写入 ----------------------------------------------------------------------*/
/**
 * @brief  nbm7100模式控制
 * @note   需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  mode: 模式
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_mode_control(nbm7100_driver_t *dev, nbm7100_mode_e mode)
{
    if(mode >= NBM7100_MODE_MAX || (dev->cfg->type == NBM7110B && mode == NBM7100_MODE_AUTO)) {
        return NBM7100_INVALID;
    }

    nbm7100_status_e ret = NBM7100_OK;
    switch (mode)
    {
        case NBM7100_MODE_ON_DEMAND:
            NBM7100_SET_BIT(dev->reg[REGISTER_COMMAND], 0);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 1);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 2);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_SET3], 7);
            break;
        case NBM7100_MODE_CONTINUOUS:
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 0);
            NBM7100_SET_BIT(dev->reg[REGISTER_COMMAND], 1);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 2);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_SET3], 7);
            break;
        case NBM7100_MODE_FORCE_ACTIVE:
            //设置激活状态,必需先设置为连续模式或者按需模式
            if(NBM7100_GET_BIT(dev->reg[REGISTER_COMMAND], 0) == 0 
            && NBM7100_GET_BIT(dev->reg[REGISTER_COMMAND], 1) == 0) {
                ret = NBM7100_INVALID;
                break;
            } else {
                NBM7100_SET_BIT(dev->reg[REGISTER_COMMAND], 2);
                NBM7100_CLEAR_BIT(dev->reg[REGISTER_SET3], 7);
            }
            break;
        case NBM7100_MODE_AUTO:
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 0);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 1);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 2);
            NBM7100_SET_BIT(dev->reg[REGISTER_SET3], 7);
            break;
        case NBM7100_MODE_NONE:
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 0);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 1);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 2);
            NBM7100_CLEAR_BIT(dev->reg[REGISTER_SET3], 7);
            break;
        default:
            ret = NBM7100_INVALID;
            break;
    }

    return ret;
}
/**
 * @brief  nbm7100输出电压控制
 * @note   需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  code: 输出电压
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_output_control(nbm7100_driver_t *dev, nbm7100_output_voltage_e code)
{
    if(code >= NBM7100_OUTPUT_MAX) {
        return NBM7100_INVALID;
    }

    #define OUTPUT_CONTROL_MASK (BIT(3) | BIT(2) | BIT(1) | BIT(0))
    dev->reg[REGISTER_SET1] = (dev->reg[REGISTER_SET1] & ~OUTPUT_CONTROL_MASK) | ((code << 0) & OUTPUT_CONTROL_MASK);

    return NBM7100_OK;
}
/**
 * @brief  nbm7100电容结束充电电压控制
 * @note   需调用 nbm7100_driver_flush() 写入生效
 *         如果 prof = 0h，则应将该寄存器设置为定义的值
 * @param  *dev: 驱动句柄
 * @param  code: 电容结束充电电压
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_end_charge_voltage(nbm7100_driver_t *dev, nbm7100_end_charge_e code)
{
    if(code >= NBM7100_END_CHARGE_MAX || code == NBM7100_END_CHARGE_NONE) {
        return NBM7100_INVALID;
    }

    #define END_CHARGE_MASK (BIT(7) | BIT(6) | BIT(5) | BIT(4))
    dev->reg[REGISTER_SET1] = (dev->reg[REGISTER_SET1] & ~END_CHARGE_MASK) | ((code << 4) & END_CHARGE_MASK);

    return NBM7100_OK;
}
/**
 * @brief  nbm7100电容最大存储电压控制
 * @note   需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  code: 电容最大存储电压
 * @retval 
 */
static nbm7100_status_e nbm7100_max_storage_voltage(nbm7100_driver_t *dev, nbm7100_max_storage_e code)
{
    if(code >= NBM7100_MAX_STORAGE_MAX) {
        return NBM7100_INVALID;
    }

    if (code == NBM7100_MAX_STORAGE_9_9V) {
        NBM7100_CLEAR_BIT(dev->reg[REGISTER_SET4], 3);
    }
    else {
        NBM7100_SET_BIT(dev->reg[REGISTER_SET4], 3);
    }

    return NBM7100_OK;
}
/** 
 * @brief  nbm7100 VBT引脚最小输入电压比较器阈值
 * @note   需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  code: VBT引脚最小输入电压比较器阈值
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_min_input_voltage(nbm7100_driver_t *dev, nbm7100_min_input_e code)
{
    if(code >= NBM7100_MIN_INPUT_MAX) {
        return NBM7100_INVALID;
    }

    #define MIN_INPUT_MASK (BIT(2) | BIT(1) | BIT(0))
    dev->reg[REGISTER_SET2] = (dev->reg[REGISTER_SET2] & ~MIN_INPUT_MASK) | ((code << 0) & MIN_INPUT_MASK);

    return NBM7100_OK;
}
/** 
 * @brief  nbm7100 电容器充电电流
 * @note   需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  code: 电容器充电电流
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_capacitor_charge_current(nbm7100_driver_t *dev, nbm7100_charge_current_e code)
{
    if(code >= NBM7100_CHARGE_CURRENT_MAX) {
        return NBM7100_INVALID;
    }

    #define CHARGE_CURRENT_MASK (BIT(7) | BIT(6) | BIT(5))
    dev->reg[REGISTER_SET2] = (dev->reg[REGISTER_SET2] & ~CHARGE_CURRENT_MASK) | ((code << 5) & CHARGE_CURRENT_MASK);

    return NBM7100_OK;
}
/** 
 * @brief  nbm7100 CAP 引脚输入电压比较器预警电压
 * @note   需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  code: CAP 引脚输入电压比较器预警电压
 * @param  enable: 使能 true:使能 false:禁用
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_early_warning_voltage(nbm7100_driver_t *dev, nbm7100_warning_voltage_e code, bool enable)
{
    if(code >= NBM7100_WARNING_VOLTAGE_MAX) {
        return NBM7100_INVALID;
    }

    #define WARNING_VOLTAGE_MASK (BIT(3) | BIT(2) | BIT(1) | BIT(0))
    dev->reg[REGISTER_SET3] = (dev->reg[REGISTER_SET3] & ~WARNING_VOLTAGE_MASK) | ((code << 0) & WARNING_VOLTAGE_MASK);

    #define WARNING_VOLTAGE_ENABLE_MASK BIT(4)
    dev->reg[REGISTER_SET3] = (dev->reg[REGISTER_SET3] & ~WARNING_VOLTAGE_ENABLE_MASK) | ((enable << 4) & WARNING_VOLTAGE_ENABLE_MASK);

    return NBM7100_OK;
}
/** 
 * @brief  nbm7100 VDH switch 配置
 * @note   需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  code: VDH switch 配置
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_VDH_switch_cfg(nbm7100_driver_t *dev, nbm7100_VDH_cfg_e code)
{
    if(code >= VDH_CFG_MAX) {
        return NBM7100_INVALID;
    }

    #define VDH_CFG_MASK BIT(4)
    dev->reg[REGISTER_SET2] = (dev->reg[REGISTER_SET2] & ~VDH_CFG_MASK) | ((code << 4) & VDH_CFG_MASK);

    return NBM7100_OK;
}
/**
 * @brief  nbm7100 设置自适应功率优化负载配置
 * @note   如果 prof[5:0] = 0h，则优化器被禁用，并且存储电容器根据 vfix[3:0] 寄存器充电
 *         需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  index: 0-63 default:1;0 fixed mode
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_set_profile(nbm7100_driver_t *dev, nbm7100_set_profile_e index)
{
    if(index >= NBM7100_PROFILE_MAX) {
        return NBM7100_INVALID;
    }
    /*
    *   0x07 -          -       -       -       -   -   prof[5] prof[4] profile_msb R/W
    *   0x08 prof[3] prof[2] prof[1] prof[0] rstpf act  ecm     eod     command     R/W
    */
    #define PROFILE_MSB_MASK (BIT(1) | BIT(0))
    #define PROFILE_LSB_MASK (BIT(7) | BIT(6) | BIT(5) | BIT(4))
    index = index & 0x3F;
    dev->reg[REGISTER_PROFILE] = (dev->reg[REGISTER_PROFILE] & ~PROFILE_MSB_MASK) | (((index >> 4)   << 0)   & PROFILE_MSB_MASK);
    dev->reg[REGISTER_COMMAND] = (dev->reg[REGISTER_COMMAND] & ~PROFILE_LSB_MASK) | (((index & 0X0F) << 4) & PROFILE_LSB_MASK);

    return NBM7100_OK;
}
/**
 * @brief  nbm7100 设置优化裕度电压
 * @note   需调用 nbm7100_driver_flush() 写入生效
 * @param  *dev: 驱动句柄
 * @param  code: 优化裕度电压
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_margin_voltage(nbm7100_driver_t *dev, nbm7100_margin_voltage_e code)
{
    if(code >= NBM7100_MARGIN_MAX) {
        return NBM7100_INVALID;
    }

    #define MARGIN_VOLTAGE_MASK (BIT(1) | BIT(0))
    dev->reg[REGISTER_SET5] = (dev->reg[REGISTER_SET5] & ~MARGIN_VOLTAGE_MASK) | ((code << 0) & MARGIN_VOLTAGE_MASK);

    return NBM7100_OK;
}
/**
 * @brief  nbm7100 刷新缓冲,写入寄存器
 * @note   设置寄存器后需调用此函数写入生效
 * @param  *dev: 驱动句柄
 * @retval 驱动状态
 */
static nbm7100_status_e nbm7100_driver_flush(nbm7100_driver_t *dev)
{
    uint8_t buf[1 + REGISTER_WRITE_LEN] = {0};
    uint8_t slave_addr = 0;

    if(dev->cfg->type == NBM7110A) {
        slave_addr = I2C_ADDR_ADR0 | dev->cfg->adr_en;
    } else if (dev->cfg->type == NBM7110B) {
        slave_addr = 0;
    } else {
        return NBM7100_INVALID;
    }

    buf[0] = REGISTER_PROFILE;

    if(dev->ops->lock != NULL) {
        if(dev->ops->lock() != NBM7100_OK) {
            return NBM7100_LOCK;
        }
    }

    memcpy(&buf[1], &dev->reg[REGISTER_PROFILE], REGISTER_WRITE_LEN);
    nbm7100_status_e ret = dev->ops->write(slave_addr, buf, sizeof(buf));

    if(dev->ops->unlock != NULL) {
        if(dev->ops->unlock() != NBM7100_OK) {
            return NBM7100_LOCK;
        }
    }
    return ret;
}
/* 外部调用 ---------------------------------------------------------------------*/
/**
 * @brief  nbm7100 发送配置电容器充电电流
 * @note   None
 * @param  *dev: 驱动句柄
 * @param  code: 电容器充电电流
 * @retval 驱动状态
 */
nbm7100_status_e nbm7100_send_charge_current(nbm7100_driver_t *dev, nbm7100_charge_current_e code)
{
    nbm7100_status_e ret = NBM7100_OK;
    ret = nbm7100_capacitor_charge_current(dev, code);
    if(ret != NBM7100_OK) {
        return ret;
    }
    ret = nbm7100_write_register(dev, REGISTER_SET2, dev->reg[REGISTER_SET2]);
    if(ret != NBM7100_OK) {
        return ret;
    }

    return ret;
}
/**
 * @brief  nbm7100 发送配置模式
 * @note   mode必需在配置完成之后发送,mode发送后立刻进入change状态,此时配置还未生效
 * @param  *dev: 驱动句柄
 * @param  mode: 模式
 * @retval 驱动状态
 */
nbm7100_status_e nbm7100_send_mode(nbm7100_driver_t *dev, nbm7100_mode_e mode)
{
    nbm7100_status_e ret = NBM7100_OK;
    ret = nbm7100_mode_control(dev, mode);
    if(ret != NBM7100_OK) {
        return ret;
    }
    ret = nbm7100_write_register(dev, REGISTER_COMMAND, dev->reg[REGISTER_COMMAND]);
    if(ret != NBM7100_OK) {
        return ret;
    }
    // ret = nbm7100_write_register(dev, REGISTER_SET3, dev->reg[REGISTER_SET3]);
    if(ret != NBM7100_OK) {
        return ret;
    }
    return ret;
}
/**
 * @brief  nbm7100 重置活动配置文件 prof[5:0] 的优化器结果
 * @note   调用后立刻生效
 * @param  *dev: 驱动句柄
 * @retval 驱动状态
 */
nbm7100_status_e nbm7100_reset_profile(nbm7100_driver_t *dev)
{
    NBM7100_SET_BIT(dev->reg[REGISTER_COMMAND], 3);

    nbm7100_status_e ret = nbm7100_write_register(dev, REGISTER_COMMAND, dev->reg[REGISTER_COMMAND]);

    NBM7100_CLEAR_BIT(dev->reg[REGISTER_COMMAND], 3);

    return ret;
}
/**
 * @brief  nbm7100 驱动初始化
 * @note   None
 * @param  *dev: 驱动句柄
 * @param  *ops: 驱动操作函数
 * @retval 驱动状态
 */
nbm7100_status_e nbm7100_driver_init(nbm7100_driver_t *dev, nbm7100_ops_t *ops)
{
    if(dev == NULL || ops == NULL) {
        return NBM7100_INVALID;
    } else {
        dev->ops = ops;
    }
    dev->ops->delay_ms(50);
    if(dev->ops->init() != NBM7100_OK) {
        return NBM7100_ERROR;
    } else {
        dev->state = NBM7100_STATE_STARTUP;
    }
    //在启动状态期间读取或写入串行总线可能会导致非易失性存储器损坏。
    dev->ops->delay_ms(20);
    dev->state = NBM7100_STATE_STANDBY;
    //写入寄存器初始化值
    dev->reg[REGISTER_COMMAND]  = 0B00010000;//Profile_index = 1
    dev->reg[REGISTER_SET1]     = 0B00001001;//Vset = 3.0 V
    dev->reg[REGISTER_SET2]     = 0B01000000;//Ich = 8 mA
    return NBM7100_OK;
}
/**
 * @brief  nbm7100 驱动反初始化
 * @note   None
 * @param  *dev: 驱动句柄
 * @retval 驱动状态
 */
nbm7100_status_e nbm7100_driver_deinit(nbm7100_driver_t *dev)
{
    if(dev == NULL) {
        return NBM7100_INVALID;
    }

    if(dev->ops->deinit() != NBM7100_OK) {
        return NBM7100_ERROR;
    }
    return NBM7100_OK;
}
/**
 * @brief  nbm7100 读取信息
 * @note   None
 * @param  *dev: 驱动句柄
 * @param  *info: 信息
 * @retval 驱动状态
 */
nbm7100_status_e nbm7100_read_info(nbm7100_driver_t *dev, nbm7100_info_t *info)
{
    if(info == NULL || dev == NULL || dev->cfg == NULL || dev->ops == NULL) {
        return NBM7100_INVALID;
    }

    nbm7100_status_e status = NBM7100_OK;
    uint8_t read_buf[REGISTER_SET5 + 1] = {0};

    status = nbm7100_read_reg(dev, read_buf, sizeof(read_buf));
    if(status != NBM7100_OK) {
        return status;
    }
    
    // for(uint8_t i = 0; i < REGISTER_READ_LEN; i++) {
    //     printf("0x%02X ", read_buf[i]);
    // }
    // printf("\r\n");

    memcpy(&info->status.reg, &read_buf[REGISTER_STATUS], sizeof(info->status.reg));

    uint32_t fuel_gauge = 0;
    fuel_gauge = read_buf[REGISTER_CHENGY] << 24 | read_buf[REGISTER_CHENGY + 1] << 16 | read_buf[REGISTER_CHENGY + 2] << 8 | read_buf[REGISTER_CHENGY + 3];

    double electrical_charge = 0;//单位uAh
    if(dev->cfg->control.charge.charge_current < NBM7100_CHARGE_CURRENT_50MA) {
        //正常充电
        electrical_charge = 0.0000044;
    } else if(dev->cfg->control.charge.charge_current == NBM7100_CHARGE_CURRENT_50MA) {
        //紧急充电
        electrical_charge = 0.0000550;
    }
    info->electric_quantity = (uint32_t)(fuel_gauge * electrical_charge);

    info->actual_Vcap = read_buf[REGISTER_VCAP];

    info->target_voltage = read_buf[REGISTER_VCHEND];

    return status;
}
/**
 * @brief  nbm7100 驱动配置
 * @note   调用后立刻生效
 * @param  *dev: 驱动句柄
 * @param  *cfg: 驱动配置
 * @retval 驱动状态
 */
nbm7100_status_e nbm7100_driver_cfg(nbm7100_driver_t *dev, nbm7100_cfg_t *cfg)
{
    if(dev == NULL || cfg == NULL) {
        return NBM7100_INVALID;
    } else {
        dev->cfg = cfg;
    }
    //!mode必需在配置完成之后发送,mode发送后立刻进入change状态,此时配置还未生效
    // nbm7100_mode_control(dev, cfg->control.mode);
    //optimiezer
    nbm7100_set_profile(dev, cfg->control.optimizer.profile);
    if(cfg->control.optimizer.profile == NBM7100_PROFILE_FIXED_MODE) {
        nbm7100_end_charge_voltage(dev, cfg->control.optimizer.fixed_charge);
    }
    nbm7100_margin_voltage(dev, cfg->control.optimizer.margin_voltage);
    //output
    nbm7100_output_control(dev, cfg->control.output.output_voltage);
    nbm7100_VDH_switch_cfg(dev, cfg->control.output.VDH_cfg);
    //charge
    nbm7100_max_storage_voltage(dev, cfg->control.charge.max_storage);
    nbm7100_capacitor_charge_current(dev, cfg->control.charge.charge_current);
    //battery
    nbm7100_min_input_voltage(dev, cfg->control.battery.input_threshold);
    nbm7100_early_warning_voltage(dev, cfg->control.battery.warning_voltage, cfg->control.battery.early_warning_enable);

    return nbm7100_driver_flush(dev);
}
/**
 * @brief  nbm7100 验证写入寄存器是否成功
 * @note   None
 * @param  *dev: 驱动句柄
 * @retval 验证失败返回 NBM7100_ERROR
 */
nbm7100_status_e nbm7100_driver_verify(nbm7100_driver_t *dev)
{
    nbm7100_status_e ret = NBM7100_OK;

    uint8_t read_buf[REGISTER_SET5 + 1] = {0};

    ret = nbm7100_read_reg(dev, read_buf, sizeof(read_buf));
    if(ret == NBM7100_OK) {
        if(memcmp(&dev->reg[REGISTER_PROFILE], &read_buf[REGISTER_PROFILE], REGISTER_WRITE_LEN) != 0) {
            ret = NBM7100_VERIFY_ERROR;
        }
    }
    // for(uint8_t i = 0; i < REGISTER_READ_LEN; i++) {
    //     printf("0x%02X ", read_buf[i]);
    // }
    // printf("\r\n");

    return ret;
}
