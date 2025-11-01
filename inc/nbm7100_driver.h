/**
 * @File Name: nbm7100_driver.h
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
#ifndef __NBM7100_DRIVER_H__
#define __NBM7100_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"
/* Exported constants --------------------------------------------------------*/
#define NBM7100_REGISTER_MAX    (0x0D + 1)  //寄存器最大值
#define NBM7110A 0  //I2C (1 MHz). 
#define NBM7110B 1  //SPI (10 MHz). 4-wire SPI mode
/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  nbm7100驱动状态
 * @note   
 */
typedef enum
{
    NBM7100_OK = 0,     //成功
    NBM7100_ERROR,      //错误
    NBM7100_READ_ERROR, //读取错误
    NBM7100_WRITE_ERROR,//写入错误
    NBM7100_VERIFY_ERROR,//校验错误
    NBM7100_LOCK,       //锁定
    NBM7100_UNLOCK,     //解锁
    NBM7100_INVALID,    //参数无效
}nbm7100_status_e;
/**
 * @brief  nbm7100模式状态
 * @note   
 */
typedef enum
{
    NBM7100_STATE_STARTUP = 0,  //当 VBT 引脚检测到电池或其他电压源并且检测到的电压高于 VPOR 电平时
    NBM7100_STATE_STANDBY,      //待机状态是一种低功耗工作状态，静态电流消耗几乎可以忽略不计；串行端口处于活动状态并准备好接收命令；
    NBM7100_STATE_CHARGE,       //DC-DC 升压转换器以 VBT 作为输入、CAP 作为输出来启用。外部存储电容器使用恒定电流 ICH 从 VBT 充电
    NBM7100_STATE_ACTIVE,       //在活动状态下，VDP 和 VDH 输出均由集成开关稳压器提供，并使用存储电容器作为其能源。在此状态下，从电池消耗的电流非常低。
    NBM7100_STATE_EMERGENCY,    //在紧急充电模式下，电池电流增加至 50 mA（典型值）
}nbm7100_state_e;
/**
 * @brief  nbm7100模式
 * @note   初始化 -> 待机 -> 充电 -> 激活 -> 待机
 *         ! 不可用待机转入激活;不可用充电转入待机
 */
typedef enum
{
    NBM7100_MODE_NONE = 0,      //待机模式
    NBM7100_MODE_ON_DEMAND,     //此模式旨在最大限度地延长系统大部分时间处于睡眠模式的低占空比应用中的电池寿命
    NBM7100_MODE_CONTINUOUS,    //连续模式适用于需要即时脉冲负载能力的应用
    NBM7100_MODE_FORCE_ACTIVE,  //强制激活
    NBM7100_MODE_AUTO,          //自动模式利用 START 引脚来设置和重置 eod 位，无需串行总线
    NBM7100_MODE_MAX,
}nbm7100_mode_e;
/**
 * @brief  nbm7100输出电压
 * @note   
 */
typedef enum
{
    NBM7100_OUTPUT_1_8V = 0,
    NBM7100_OUTPUT_2_0V,
    NBM7100_OUTPUT_2_2V,
    NBM7100_OUTPUT_2_4V,
    NBM7100_OUTPUT_2_5V,
    NBM7100_OUTPUT_2_6V,
    NBM7100_OUTPUT_2_7V,
    NBM7100_OUTPUT_2_8V,
    NBM7100_OUTPUT_2_9V,
    NBM7100_OUTPUT_3_0V,            //default
    NBM7100_OUTPUT_3_1V,
    NBM7100_OUTPUT_3_2V,
    NBM7100_OUTPUT_3_3V,
    NBM7100_OUTPUT_3_4V,
    NBM7100_OUTPUT_3_5V,
    NBM7100_OUTPUT_3_6V,
    NBM7100_OUTPUT_MAX,
}nbm7100_output_voltage_e;
/**
 * @brief  nbm7100电容结束充电电压
 * @note   这将选择在没有选择配置文件(即配置文件设置为0)时电容器充电的电压。
 *         Vfix
 */
typedef enum
{
    NBM7100_END_CHARGE_NONE = 0,    //default
    NBM7100_END_CHARGE_3_4V,
    NBM7100_END_CHARGE_4_4V,
    NBM7100_END_CHARGE_5_2V,
    NBM7100_END_CHARGE_5_9V,
    NBM7100_END_CHARGE_6_5V,
    NBM7100_END_CHARGE_7_1V,
    NBM7100_END_CHARGE_7_7V,
    NBM7100_END_CHARGE_8_2V,
    NBM7100_END_CHARGE_8_7V,
    NBM7100_END_CHARGE_9_1V,
    NBM7100_END_CHARGE_9_5V,
    NBM7100_END_CHARGE_9_9V,
    NBM7100_END_CHARGE_10_3V,
    NBM7100_END_CHARGE_10_7V,
    NBM7100_END_CHARGE_11_0V,
    NBM7100_END_CHARGE_MAX,
}nbm7100_end_charge_e;
/**
 * @brief  nbm7100电容最大存储电压
 * @note   
 */
typedef enum
{
    NBM7100_MAX_STORAGE_9_9V = 0,   //default
    NBM7100_MAX_STORAGE_11_0V,
    NBM7100_MAX_STORAGE_MAX,
}nbm7100_max_storage_e;
/**
 * @brief  nbm7100 VBT引脚最小输入电压比较器阈值
 * @note   当电池电压低于该阈值时，充电电流ICH将自动减小。
 *         input threshold
 */
typedef enum
{
    NBM7100_MIN_INPUT_2_4V = 0,     //default
    NBM7100_MIN_INPUT_2_6V,
    NBM7100_MIN_INPUT_2_8V,
    NBM7100_MIN_INPUT_3_0V,
    NBM7100_MIN_INPUT_3_2V,
    NBM7100_MIN_INPUT_MAX,
}nbm7100_min_input_e;
/**
 * @brief  nbm7100 电容器充电电流
 * @note   0~3:NORMAL, 4:EMERGENCY
 */
typedef enum
{
    NBM7100_CHARGE_CURRENT_2MA = 0,
    NBM7100_CHARGE_CURRENT_4MA,
    NBM7100_CHARGE_CURRENT_8MA,     //default
    NBM7100_CHARGE_CURRENT_16MA,
    NBM7100_CHARGE_CURRENT_50MA,    //emergency charge, ICH = max
    NBM7100_CHARGE_CURRENT_MAX,
}nbm7100_charge_current_e;
/**
 * @brief  nbm7100 CAP 引脚输入电压比较器预警电压
 * @note   
 */
typedef enum
{
    NBM7100_WARNING_VOLTAGE_2_4V = 0,   //default
    NBM7100_WARNING_VOLTAGE_2_6V,
    NBM7100_WARNING_VOLTAGE_2_8V,
    NBM7100_WARNING_VOLTAGE_3_0V,
    NBM7100_WARNING_VOLTAGE_3_2V,
    NBM7100_WARNING_VOLTAGE_3_4V,
    NBM7100_WARNING_VOLTAGE_3_6V,
    NBM7100_WARNING_VOLTAGE_4_0V,
    NBM7100_WARNING_VOLTAGE_4_4V,
    NBM7100_WARNING_VOLTAGE_4_8V,
    NBM7100_WARNING_VOLTAGE_MAX,
}nbm7100_warning_voltage_e;
/**
 * @brief  nbm7100 VDH switch configuration 
 * @note   待机和活动状态下的 VDH 高阻抗模式
 */
typedef enum
{
    VDH_CFG_VDH_VBT = 0,    //VDH voltage = VBT voltage
    VDH_CFG_HIGH_IMPEDANCE, //高阻抗
    VDH_CFG_MAX,
}nbm7100_VDH_cfg_e;
/**
 * @brief  nbm7100 设置自适应功率优化负载配置
 * @note   如果选择 profile = 0，则不进行优化，应用 VFIX
 */
typedef enum
{
    NBM7100_PROFILE_FIXED_MODE = 0,
    NBM7100_PROFILE_1,              //default
    NBM7100_PROFILE_63 = 63,
    NBM7100_PROFILE_MAX,
}nbm7100_set_profile_e;
/**
 * @brief  nbm7100 优化裕度电压
 * @note   
 */
typedef enum
{
    NBM7100_MARGIN_3_41V = 0,   //minimum
    NBM7100_MARGIN_4_39V,       //nominal
    NBM7100_MARGIN_5_21V,       //safe
    NBM7100_MARGIN_5_91V,       //extra safe    [default]
    NBM7100_MARGIN_MAX,
}nbm7100_margin_voltage_e;
/**
 * @brief  nbm7100 优化裕度电压
 * @note   
 */
typedef enum
{
    NBM7100_VCAP_1_40V = 0,
    NBM7100_VCAP_1_51V,
    NBM7100_VCAP_1_80V,
    NBM7100_VCAP_2_19V,
    NBM7100_VCAP_2_40V,
    NBM7100_VCAP_2_60V,
    NBM7100_VCAP_2_79V,
    NBM7100_VCAP_3_01V,
    NBM7100_VCAP_3_20V,
    NBM7100_VCAP_3_41V,
    NBM7100_VCAP_3_61V,
    NBM7100_VCAP_3_99V,
    NBM7100_VCAP_4_39V,
    NBM7100_VCAP_4_80V,
    NBM7100_VCAP_5_21V,
    NBM7100_VCAP_5_59V,
    NBM7100_VCAP_5_91V,
    NBM7100_VCAP_6_02V,
    NBM7100_VCAP_6_40V,
    NBM7100_VCAP_6_55V,
    NBM7100_VCAP_6_80V,
    NBM7100_VCAP_7_14V,
    NBM7100_VCAP_7_23V,
    NBM7100_VCAP_7_68V,
    NBM7100_VCAP_8_19V,
    NBM7100_VCAP_8_65V,
    NBM7100_VCAP_9_10V,
    NBM7100_VCAP_9_53V,
    NBM7100_VCAP_9_91V,
    NBM7100_VCAP_10_33V,
    NBM7100_VCAP_10_69V,
    NBM7100_VCAP_11_07V,
    NBM7100_VCAP_MAX,
}nbm7100_Vcap_e;
/**
 * @brief  nbm7100 状态信息
 * @note   None
 */
typedef struct
{
    union 
    {
        uint8_t reg;
        struct 
        {
            //RDY 处于On-demand 模式,当达到电容器充电水平时，RDY 信号将变高
            //RDY 处于Continuous模式,当 CAP 引脚上的电压达到 VFIX 时，RDY 信号将变高
            uint8_t ready : 1;                 //rdy=1：输出状态指示。参见第 8.4.1.2 节
            //bit[1:4] 保留
            uint8_t none : 4;                  //none
            //在下一个充电周期开始时清零
            uint8_t alarm_vdh_output : 1;       //发生在活动状态。alarm = 1：表示在活动状态期间发生 VDH 警报条件（负载电流过高或 VCAP 电压过低）
            //在下一个充电周期开始时清零
            uint8_t alarm_early_warning : 1;    //发生在活动状态 ew = 1：表示存储电容电压在活动状态期间已降至 VEW 阈值以下
            //设置后，如果不重新启动设备，该警报就不会被清除
            uint8_t alarm_low_voltage : 1;      //发生在充电状态 vmin = 1：表示充电状态期间发生电池电量低的情况
        }data;
    };
}nbm7100_status_t;

typedef struct 
{
    nbm7100_status_t    status;             //状态信息
    uint32_t            electric_quantity;  //电量
    nbm7100_Vcap_e      actual_Vcap;        //实际电容电压
    nbm7100_Vcap_e      target_voltage;     //目标电压
}nbm7100_info_t;

/**
 * @brief  nbm710 0驱动操作函数
 * @note   None
 */
typedef struct 
{
    nbm7100_status_e (*init)(void);
    nbm7100_status_e (*deinit)(void);
    nbm7100_status_e (*read)(uint8_t addr, uint8_t *buf, uint16_t len);
    nbm7100_status_e (*write)(uint8_t addr, uint8_t *buf, uint16_t len);
    nbm7100_status_e (*delay_ms)(uint32_t ms);
    nbm7100_status_e (*lock)(void);
    nbm7100_status_e (*unlock)(void);
    nbm7100_status_e (*section_begin)(void);
    nbm7100_status_e (*section_end)(void);
}nbm7100_ops_t;
/**
 * @brief  nbm7100 驱动配置
 * @note   
 */
typedef struct 
{
    uint8_t type;    //芯片类型
    bool    adr_en;  //地址使能

    struct
    {
        nbm7100_mode_e  mode;   //模式
        struct 
        {
            nbm7100_set_profile_e       profile;                //设置自适应功率优化负载配置
            nbm7100_end_charge_e        fixed_charge;           //电容结束充电电压 [profile = 0时有效]
            nbm7100_margin_voltage_e    margin_voltage;         //优化裕度电压
        }optimizer;
        struct 
        {
            nbm7100_output_voltage_e    output_voltage;         //输出电压
            nbm7100_VDH_cfg_e           VDH_cfg;                //VDH switch configuration
        }output;
        struct
        {
            nbm7100_max_storage_e       max_storage;            //电容最大存储电压
            nbm7100_charge_current_e    charge_current;         //电容器充电电流
        }charge;
        struct 
        {
            nbm7100_min_input_e         input_threshold;        //VBT引脚最小输入电压比较器阈值
            bool                        early_warning_enable;   //CAP 引脚输入电压比较器预警使能
            nbm7100_warning_voltage_e   warning_voltage;        //CAP 引脚输入电压比较器预警电压
        }battery;
    }control;
}nbm7100_cfg_t;
/**
 * @brief  nbm7100 driver API definition.
 * @note   None
 */
typedef struct 
{
    nbm7100_cfg_t       *cfg;   //驱动配置
    nbm7100_ops_t       *ops;   //驱动操作函数
    nbm7100_state_e     state;  //模式状态

    uint8_t             reg[NBM7100_REGISTER_MAX];  //寄存器缓存
}nbm7100_driver_t;
/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
nbm7100_status_e nbm7100_driver_init(nbm7100_driver_t *dev, nbm7100_ops_t *ops);
nbm7100_status_e nbm7100_driver_deinit(nbm7100_driver_t *dev);
nbm7100_status_e nbm7100_driver_cfg(nbm7100_driver_t *dev, nbm7100_cfg_t *cfg);
nbm7100_status_e nbm7100_driver_verify(nbm7100_driver_t *dev);

nbm7100_status_e nbm7100_read_info(nbm7100_driver_t *dev, nbm7100_info_t *info);
nbm7100_status_e nbm7100_send_charge_current(nbm7100_driver_t *dev, nbm7100_charge_current_e code);
nbm7100_status_e nbm7100_send_mode(nbm7100_driver_t *dev, nbm7100_mode_e mode);
nbm7100_status_e nbm7100_reset_profile(nbm7100_driver_t *dev);


#ifdef __cplusplus
}
#endif

#endif /* __NBM7100_DRIVER_H__ */