/*
 * smart_shelf.h - 智慧花架主控模块
 *
 * 平台: STM32F407VGT6, 沿用 bin.ioc 已配置的外设, 仅新增 GPIO 输出与 ADC1 (寄存器方式)
 *
 * 功能:
 *   感知  土壤湿度 x3、水箱水位 (ADC1); 温湿度 AHT20、光照 BH1750 (I2C2)
 *   视觉  K230 (USART1) + 二自由度云台逐盆巡检, 识别植株状态与绿叶覆盖率
 *   控制  单泵 + 电磁阀分盆脉冲浇水、补光灯闭环调光、风扇、遮阳帘舵机
 *   保护  缺水禁泵、单次/每日浇水限时、传感器离线检测、独立看门狗
 *   上报  USART3 每 2s 输出一行状态 (可接 ESP8266 / 蓝牙 / 串口助手)
 *
 * 调用链 (SMART_SHELF_ENABLE=1):
 *   main()  → System_Init() [init.c] → CubeMX 外设 → Motor/Servo/Emm_V5 → SmartShelf_Init()
 *           → while(1) SmartShelf_Loop()
 *   USART1_IRQHandler [stm32f4xx_it.c] → SmartShelf_Vision_ParseByte()
 *   SMART_SHELF_ENABLE=0 时以上三处恢复为 ServoTracker + 步进电机测试程序
 *
 * ---------------------------------------------------------------------------
 * 硬件连接
 *   PC0 / PC1 / PC2  ADC1_IN10~12   土壤湿度 1~3 (电容式, 3.3V 供电; 建议各加 100k 下拉, 断线可识别)
 *   PC3              ADC1_IN13      水箱水位 (模拟量水位传感器)
 *   PB10 / PB11      I2C2           AHT20 (0x38) + BH1750 (0x23, ADDR 接地), 与 MPU6050 共用总线
 *   PA9  / PA10      USART1         K230 视觉模块 115200 8N1
 *   PD8  / PD9       USART3         状态上报 115200 8N1
 *   PA2 + PA6/PA7    TB6612 A 路    水泵 (PWMA / AIN1 / AIN2, 10kHz PWM)
 *   PA3 + PA4/PA5    TB6612 B 路    补光灯 (PWMB / BIN1 / BIN2, 灯带正极接 BO1)
 *   PE2 / PE3 / PE4  GPIO           电磁阀 1~3 (继电器或 MOS 模块)
 *   PE5              GPIO           风扇
 *   PE6              GPIO           有源蜂鸣器
 *   PA8  (Servo 2)   TIM1_CH1       云台水平 Pan
 *   PC9  (Servo 1)   TIM3_CH4       云台俯仰 Tilt
 *   PA11 (Servo 3)   TIM1_CH4       遮阳帘
 *   每盆的 ADC 通道 / 电磁阀 / 云台预置位集中在 smart_shelf.c 顶部 pot_cfg 表
 *
 * ---------------------------------------------------------------------------
 * K230 通信协议 (沿用原 7 字节帧格式: 0x55 0xAA func d0 d1 d2 0xFA)
 *   STM32 → K230  识别请求  func=0x20  d0=花盆编号(0~N-1)  d1=d2=0
 *   K230 → STM32  识别结果  func=0x10  d0=PlantStatus  d1=置信度(0~100)  d2=绿叶覆盖率(0~100)
 *   云台转到某盆预置位、等待稳定后发送请求; K230 建议对多帧投票后回复一帧结果。
 *   K230 也可以持续发送结果帧, STM32 只采纳请求之后收到的第一帧。
 *
 * ---------------------------------------------------------------------------
 * 标定: 调试器查看变量 shelf.pot[i].soil_raw,
 *       传感器置于空气中的读数 → SOIL_ADC_DRY, 插入清水中的读数 → SOIL_ADC_WET;
 *       水箱空 / 满时的 shelf.tank_raw → TANK_ADC_EMPTY / TANK_ADC_FULL
 */

#ifndef __SMART_SHELF_H
#define __SMART_SHELF_H

#include "main.h"
#include <stdint.h>

/* ===================== 功能开关 ===================== */

#define SMART_SHELF_ENABLE      1       /* 1=运行智慧花架; 0=恢复原有步进电机测试程序 */
#define SHELF_USE_IWDG          1       /* 独立看门狗 (约 2s), 防止死机时水泵常开 */

#define SHELF_POT_NUM           3       /* 花盆数量, 须与 smart_shelf.c 中 pot_cfg 行数一致 */

/* ===================== 引脚与通道 ===================== */

#define SHELF_ADC_GPIO_Port     GPIOC
#define SHELF_ADC_Pins          (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define SHELF_TANK_ADC_CH       13      /* PC3 */

#define SHELF_VALVE1_GPIO_Port  GPIOE
#define SHELF_VALVE1_Pin        GPIO_PIN_2
#define SHELF_VALVE2_GPIO_Port  GPIOE
#define SHELF_VALVE2_Pin        GPIO_PIN_3
#define SHELF_VALVE3_GPIO_Port  GPIOE
#define SHELF_VALVE3_Pin        GPIO_PIN_4
#define SHELF_FAN_GPIO_Port     GPIOE
#define SHELF_FAN_Pin           GPIO_PIN_5
#define SHELF_BUZZER_GPIO_Port  GPIOE
#define SHELF_BUZZER_Pin        GPIO_PIN_6

#define SHELF_RELAY_ON_LEVEL    GPIO_PIN_SET    /* 阀/风扇模块有效电平, 低电平触发的继电器改为 GPIO_PIN_RESET */
#define SHELF_BUZZER_ON_LEVEL   GPIO_PIN_SET

#define SHELF_MOTOR_PUMP        0       /* motor.c 通道: TB6612 A 路 */
#define SHELF_MOTOR_LIGHT       1       /* motor.c 通道: TB6612 B 路 */
#define SHELF_SERVO_PAN         2       /* servo.c 编号: PA8  */
#define SHELF_SERVO_TILT        1       /* servo.c 编号: PC9  */
#define SHELF_SERVO_SHADE       3       /* servo.c 编号: PA11 */

/* ===================== 传感器标定 ===================== */

#define SOIL_ADC_DRY            3000    /* 空气中读数 (电容式: 越湿电压越低) */
#define SOIL_ADC_WET            1300    /* 清水中读数 */
#define SOIL_ADC_MIN_VALID      100     /* 超出 [MIN, MAX] 视为断线/短路, 该盆停止自动浇水 */
#define SOIL_ADC_MAX_VALID      4000
#define TANK_ADC_EMPTY          200
#define TANK_ADC_FULL           2500

/* ===================== 浇水 ===================== */

#define SOIL_DRY_PCT            35      /* 低于此值开始浇水 */
#define SOIL_TARGET_PCT         60      /* 达到此值结束本轮浇水 */
#define SOIL_WET_PCT            80      /* 高于此值仍萎蔫 → 积水报警 */
#define WATER_PULSE_MS          4000    /* 单次脉冲浇水时长 */
#define WATER_SOAK_MS           60000   /* 两次脉冲之间等待水分渗透 */
#define WATER_DAILY_MAX_MS      60000   /* 每盆每日累计浇水上限 */
#define PUMP_DUTY               80      /* 水泵 PWM 占空比 % */
#define VALVE_LEAD_MS           200     /* 先开阀再启泵 */
#define VALVE_LAG_MS            300     /* 停泵后延时关阀 */
#define TANK_LOW_PCT            10      /* 水位低于此值禁止浇水并报警 */

/* ===================== 补光 / 通风 / 遮阳 ===================== */

#define CLOCK_BOOT_HOUR         8       /* 无 RTC: 上电视为当天几点, 可用 SmartShelf_SetTime() 校时 */
#define LIGHT_ON_HOUR           6       /* 补光时段 [ON, OFF), 其余时间关灯保证暗期 */
#define LIGHT_OFF_HOUR          20
#define LUX_TARGET              3000    /* 目标照度 (lx) */
#define LUX_DEADBAND            300
#define LIGHT_STEP              5       /* 每秒调光步进 % */
#define LIGHT_FALLBACK_PWM      60      /* 光照传感器离线时的固定亮度 % */

#define FAN_ON_TEMP             30.0f   /* 高于此温度或湿度开风扇 */
#define FAN_ON_HUMI             85.0f
#define TEMP_ALARM_HIGH         38.0f
#define TEMP_ALARM_LOW          5.0f

#define SHADE_CLOSE_LUX         30000   /* 强光或高温时合上遮阳帘 */
#define SHADE_CLOSE_TEMP        35.0f
#define SHADE_OPEN_LUX          8000    /* 遮阳后读数会下降, 打开阈值需按实际透光率调整 */
#define SHADE_HOLD_MS           (10UL * 60 * 1000)
#define SHADE_OPEN_ANGLE        45.0f
#define SHADE_CLOSED_ANGLE      180.0f

/* ===================== 视觉巡检 ===================== */

#define PATROL_PERIOD_MS        (5UL * 60 * 1000)   /* 巡检周期 */
#define PATROL_FIRST_DELAY_MS   5000                /* 上电后首次巡检延时 */
#define CAM_SETTLE_MS           800                 /* 云台到位后等待稳定 */
#define VISION_REPLY_TIMEOUT_MS 3000
#define VISION_CONF_MIN         60                  /* 置信度低于此值的结果不参与决策 */
#define VISION_VALID_MS         (2 * PATROL_PERIOD_MS)
#define CAM_HOME_PAN            135.0f
#define CAM_HOME_TILT           135.0f

#define VISION_FUNC_RESULT      0x10
#define VISION_FUNC_REQUEST     0x20

#define REPORT_PERIOD_MS        2000

/* ===================== 数据类型 ===================== */

typedef enum {
    PLANT_NONE = 0,         /* 未检出植株 */
    PLANT_HEALTHY,          /* 健康 */
    PLANT_WILT,             /* 缺水萎蔫 */
    PLANT_YELLOW,           /* 黄叶 */
    PLANT_SPOT,             /* 病斑 */
    PLANT_PEST,             /* 虫害 */
    PLANT_UNKNOWN = 0xFF    /* 尚未识别 / 本轮无结果 */
} PlantStatus;

/* 报警位 (SmartShelf_State.alarm) */
#define ALARM_TANK_LOW          (1u << 0)   /* 水箱缺水, 已禁止浇水 */
#define ALARM_AHT_OFFLINE       (1u << 1)
#define ALARM_BH_OFFLINE        (1u << 2)
#define ALARM_VISION_OFFLINE    (1u << 3)   /* 上一轮巡检未收到任何结果 */
#define ALARM_SOIL_FAULT        (1u << 4)
#define ALARM_OVERWATER         (1u << 5)   /* 土壤很湿但植株萎蔫: 疑似积水烂根 */
#define ALARM_DISEASE           (1u << 6)
#define ALARM_PEST              (1u << 7)
#define ALARM_TEMP              (1u << 8)
#define ALARM_WATER_LIMIT       (1u << 9)   /* 已达每日浇水上限仍缺水: 检查漏水/传感器 */

/* 需要蜂鸣器提示的报警 (离线类只上报不鸣叫) */
#define ALARM_BEEP_MASK         (ALARM_TANK_LOW | ALARM_OVERWATER | ALARM_DISEASE | \
                                 ALARM_PEST | ALARM_TEMP | ALARM_WATER_LIMIT)

typedef struct {
    uint16_t    soil_raw;           /* 土壤湿度 ADC (滤波后, 标定用) */
    uint8_t     soil_pct;           /* 土壤湿度 0~100% */
    uint8_t     soil_fault;         /* 1=传感器异常 */
    uint8_t     thirsty;            /* 1=处于补水过程中 (滞回状态) */
    PlantStatus plant;              /* 最近一次视觉识别结果 */
    uint8_t     plant_conf;         /* 置信度 0~100 */
    uint8_t     leaf_cover;         /* 绿叶覆盖率 0~100% */
    uint32_t    vision_tick;        /* 识别时间 */
    uint32_t    last_water_tick;    /* 上次脉冲结束时间 */
    uint32_t    water_ms_today;     /* 今日累计浇水时长 */
} PotState;

typedef struct {
    /* 环境 */
    float    temp_c;
    float    humi_pct;
    float    lux;
    uint16_t tank_raw;
    uint8_t  tank_pct;
    uint8_t  aht_online;            /* 1=有有效数据 */
    uint8_t  bh_online;
    uint8_t  vision_online;
    /* 花盆 */
    PotState pot[SHELF_POT_NUM];
    /* 执行器 */
    uint8_t  pump_on;
    int8_t   watering_pot;          /* 正在浇水的花盆, -1=无 */
    uint8_t  light_pwm;             /* 补光灯亮度 0~100% */
    uint8_t  fan_on;
    uint8_t  shade_closed;
    /* 其他 */
    uint32_t alarm;                 /* ALARM_xxx 位掩码 */
    uint32_t clock_sec;             /* 软件时钟: 当天第几秒 */
} SmartShelf_State;

/* ===================== 接口函数 ===================== */

void SmartShelf_Init(void);
void SmartShelf_Loop(void);                         /* 主循环中反复调用, 非阻塞 */
void SmartShelf_Vision_ParseByte(uint8_t byte);     /* USART1 接收中断中调用 */
void SmartShelf_SetTime(uint8_t hour, uint8_t minute);
const SmartShelf_State *SmartShelf_GetState(void);

#endif /* __SMART_SHELF_H */
