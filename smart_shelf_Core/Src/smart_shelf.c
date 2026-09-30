/*
 * smart_shelf.c - 智慧花架主控模块
 *
 * SmartShelf_Loop() 为非阻塞协作式调度, 各任务按自身周期运行:
 *   Sensor_Task   ADC 500ms / AHT20 2s / BH1750 1s, I2C 传感器离线后每 10s 重试
 *   Vision_Task   云台逐盆巡检: 转到预置位 → 稳定 → 请求识别 → 等待结果 → 下一盆
 *   Control_Task  1s: 软件时钟、浇水决策、补光、风扇、遮阳、报警
 *   Water_Task    浇水时序: 开阀 → 启泵 → 停泵 → 关阀, 每次循环检查以保证按时停泵
 *   Buzzer_Task   有需提示的报警时每 3s 鸣叫 200ms
 *   Report_Task   2s: USART3 输出一行状态
 *
 * HAL 阻塞调用均带短超时, 单次循环最长几十 ms, 远小于看门狗 2s。
 */

#include "smart_shelf.h"
#include "motor.h"
#include "servo.h"
#include "i2c.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* ===================== 每盆配置 ===================== */

typedef struct {
    uint8_t       adc_ch;       /* 土壤湿度 ADC1 通道: PC0~PC3 = IN10~IN13 */
    GPIO_TypeDef *valve_port;   /* 电磁阀 */
    uint16_t      valve_pin;
    float         cam_pan;      /* 云台对准该盆的角度, 按实际安装调整 */
    float         cam_tilt;
} PotConfig;

static const PotConfig pot_cfg[] = {
    {10, SHELF_VALVE1_GPIO_Port, SHELF_VALVE1_Pin,  95.0f, 120.0f},
    {11, SHELF_VALVE2_GPIO_Port, SHELF_VALVE2_Pin, 135.0f, 120.0f},
    {12, SHELF_VALVE3_GPIO_Port, SHELF_VALVE3_Pin, 175.0f, 120.0f},
};

/* pot_cfg 行数与 SHELF_POT_NUM 不一致时编译报错 */
typedef char pot_cfg_size_check[(sizeof(pot_cfg) / sizeof(pot_cfg[0]) == SHELF_POT_NUM) ? 1 : -1];

/* ===================== 内部参数 ===================== */

#define ADC_PERIOD_MS           500
#define ADC_AVG_N               8
#define AHT_PERIOD_MS           2000
#define AHT_MEASURE_MS          80
#define BH_PERIOD_MS            1000
#define CONTROL_PERIOD_MS       1000
#define I2C_TIMEOUT_MS          5
#define I2C_FAIL_LIMIT          3
#define I2C_RETRY_MS            10000

#define AHT20_ADDR              (0x38 << 1)
#define BH1750_ADDR             (0x23 << 1)

/* ===================== 运行状态 ===================== */

static SmartShelf_State shelf;
static uint8_t tank_low = 0;                /* 带滞回的缺水标志 */

typedef struct {
    uint8_t  present;                       /* 1=设备在线 (初始化成功) */
    uint8_t  busy;                          /* AHT20: 已触发测量, 等待读取 */
    uint8_t  fail;
    uint32_t tick;
} I2C_Dev;

static I2C_Dev aht;
static I2C_Dev bh;

typedef enum {
    WATER_IDLE = 0,
    WATER_VALVE_LEAD,
    WATER_PUMPING,
    WATER_VALVE_LAG
} WaterPhase;

static struct {
    WaterPhase phase;
    uint8_t    pot;
    uint8_t    rr_next;                     /* 轮询起点, 避免总是优先第 1 盆 */
    uint32_t   phase_tick;
    uint32_t   pump_tick;
} water;

typedef enum {
    PATROL_IDLE = 0,
    PATROL_MOVE,
    PATROL_WAIT
} PatrolStep;

static struct {
    PatrolStep step;
    uint8_t    pot;
    uint8_t    replies;                     /* 本轮收到的有效结果数 */
    uint16_t   req_seq;                     /* 发送请求时的帧序号快照 */
    uint32_t   tick;
    uint32_t   cycle_tick;
} patrol;

/* 视觉结果邮箱: USART1 中断写入, 主循环关中断读取 */
static volatile struct {
    uint8_t  status;
    uint8_t  conf;
    uint8_t  cover;
    uint16_t seq;
} vision_rx;

static uint32_t adc_tick, control_tick, report_tick, clock_tick, retry_tick, shade_tick;

/* ===================== GPIO 执行器 ===================== */

#define RELAY_OFF_LEVEL  ((SHELF_RELAY_ON_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET)

static void Relay_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, on ? SHELF_RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
}

static void Buzzer_Write(uint8_t on)
{
    GPIO_PinState off_level = (SHELF_BUZZER_ON_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(SHELF_BUZZER_GPIO_Port, SHELF_BUZZER_Pin, on ? SHELF_BUZZER_ON_LEVEL : off_level);
}

/* 先写关断电平再切换为输出, 避免上电瞬间继电器误动作 */
static void Output_Init(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState off_level)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(port, pin, off_level);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static void Shelf_GPIO_Init(void)
{
    /* GPIOA~E 时钟已在 MX_GPIO_Init 中开启 */
    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        Output_Init(pot_cfg[i].valve_port, pot_cfg[i].valve_pin, RELAY_OFF_LEVEL);
    }
    Output_Init(SHELF_FAN_GPIO_Port, SHELF_FAN_Pin, RELAY_OFF_LEVEL);
    Output_Init(SHELF_BUZZER_GPIO_Port, SHELF_BUZZER_Pin,
                (SHELF_BUZZER_ON_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* ===================== ADC1 (寄存器方式, 工程未启用 HAL_ADC) ===================== */

static void Shelf_ADC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = SHELF_ADC_Pins;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SHELF_ADC_GPIO_Port, &GPIO_InitStruct);

    __HAL_RCC_ADC1_CLK_ENABLE();

    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;    /* PCLK2/4 = 21MHz */
    ADC1->CR1 = 0;                                                  /* 12 位, 非扫描 */
    ADC1->CR2 = 0;                                                  /* 单次转换, 右对齐 */
    ADC1->SMPR1 = 0x07FFFFFF;                                       /* IN10~18 采样 480 周期, 适配高阻传感器 */
    ADC1->SQR1 = 0;                                                 /* 规则序列长度 1 */
    ADC1->CR2 |= ADC_CR2_ADON;
    HAL_Delay(1);
}

/* 超时返回 0, 会被判定为传感器异常 */
static uint16_t Shelf_ADC_Read(uint8_t ch)
{
    ADC1->SQR3 = ch;
    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    uint32_t start = HAL_GetTick();
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if (HAL_GetTick() - start > 2) return 0;
    }
    return (uint16_t)ADC1->DR;
}

static uint16_t Shelf_ADC_ReadAvg(uint8_t ch)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_AVG_N; i++) {
        sum += Shelf_ADC_Read(ch);
    }
    return (uint16_t)(sum / ADC_AVG_N);
}

/* 将 raw 按两点标定线性映射到 0~100%, 两端可以是递增或递减 */
static uint8_t Map_Pct(uint16_t raw, uint16_t raw_0pct, uint16_t raw_100pct)
{
    int32_t span = (int32_t)raw_100pct - (int32_t)raw_0pct;
    if (span == 0) return 0;

    int32_t pct = ((int32_t)raw - (int32_t)raw_0pct) * 100 / span;
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

static void ADC_Update(uint8_t first)
{
    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        PotState *p = &shelf.pot[i];
        uint16_t raw = Shelf_ADC_ReadAvg(pot_cfg[i].adc_ch);

        /* 一阶低通: 新值占 1/4 */
        p->soil_raw = first ? raw : (uint16_t)((p->soil_raw * 3u + raw) / 4u);
        p->soil_fault = (p->soil_raw < SOIL_ADC_MIN_VALID || p->soil_raw > SOIL_ADC_MAX_VALID);
        p->soil_pct = Map_Pct(p->soil_raw, SOIL_ADC_DRY, SOIL_ADC_WET);
    }

    uint16_t raw = Shelf_ADC_ReadAvg(SHELF_TANK_ADC_CH);
    shelf.tank_raw = first ? raw : (uint16_t)((shelf.tank_raw * 3u + raw) / 4u);
    shelf.tank_pct = Map_Pct(shelf.tank_raw, TANK_ADC_EMPTY, TANK_ADC_FULL);

    /* 缺水按本次读数立即判定 (滤波值要数秒才降下来, 期间水泵会干转), 滤波值回升 5% 以上才解除 */
    if (Map_Pct(raw, TANK_ADC_EMPTY, TANK_ADC_FULL) < TANK_LOW_PCT) {
        tank_low = 1;
    } else if (shelf.tank_pct >= TANK_LOW_PCT + 5) {
        tank_low = 0;
    }
}

/* ===================== I2C 传感器: AHT20 温湿度 / BH1750 光照 ===================== */

static uint8_t AHT20_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t AHT20_Init(void)
{
    uint8_t status;
    if (HAL_I2C_Master_Receive(&hi2c2, AHT20_ADDR, &status, 1, I2C_TIMEOUT_MS) != HAL_OK) return 0;

    /* bit3=0 表示未校准, 发送初始化命令 */
    if (!(status & 0x08)) {
        uint8_t cmd[3] = {0xBE, 0x08, 0x00};
        if (HAL_I2C_Master_Transmit(&hi2c2, AHT20_ADDR, cmd, 3, I2C_TIMEOUT_MS) != HAL_OK) return 0;
        HAL_Delay(10);
    }
    return 1;
}

/* 连续失败 I2C_FAIL_LIMIT 次判为离线, 交给 I2C_Retry 重连 */
static void I2C_Dev_Fail(I2C_Dev *dev, uint8_t *online)
{
    dev->busy = 0;
    if (++dev->fail >= I2C_FAIL_LIMIT) {
        dev->present = 0;
        *online = 0;
    }
}

static void AHT20_Task(uint32_t now)
{
    if (!aht.present) return;

    if (!aht.busy) {
        if (now - aht.tick < AHT_PERIOD_MS) return;
        uint8_t cmd[3] = {0xAC, 0x33, 0x00};
        aht.tick = now;
        if (HAL_I2C_Master_Transmit(&hi2c2, AHT20_ADDR, cmd, 3, I2C_TIMEOUT_MS) == HAL_OK) {
            aht.busy = 1;
        } else {
            I2C_Dev_Fail(&aht, &shelf.aht_online);
        }
        return;
    }

    if (now - aht.tick < AHT_MEASURE_MS) return;
    aht.busy = 0;

    uint8_t d[7];
    if (HAL_I2C_Master_Receive(&hi2c2, AHT20_ADDR, d, 7, I2C_TIMEOUT_MS) != HAL_OK ||
        (d[0] & 0x80) || AHT20_CRC8(d, 6) != d[6])
    {
        I2C_Dev_Fail(&aht, &shelf.aht_online);
        return;
    }

    uint32_t raw_h = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
    uint32_t raw_t = (((uint32_t)d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];
    shelf.humi_pct = (float)raw_h * 100.0f / 1048576.0f;
    shelf.temp_c   = (float)raw_t * 200.0f / 1048576.0f - 50.0f;
    shelf.aht_online = 1;
    aht.fail = 0;
}

static HAL_StatusTypeDef BH1750_Cmd(uint8_t cmd)
{
    return HAL_I2C_Master_Transmit(&hi2c2, BH1750_ADDR, &cmd, 1, I2C_TIMEOUT_MS);
}

/* 上电 + 连续高分辨率模式 (1lx, 120ms) */
static uint8_t BH1750_Init(void)
{
    return BH1750_Cmd(0x01) == HAL_OK && BH1750_Cmd(0x10) == HAL_OK;
}

static void BH1750_Task(uint32_t now)
{
    if (!bh.present || now - bh.tick < BH_PERIOD_MS) return;
    bh.tick = now;

    uint8_t d[2];
    if (HAL_I2C_Master_Receive(&hi2c2, BH1750_ADDR, d, 2, I2C_TIMEOUT_MS) != HAL_OK) {
        I2C_Dev_Fail(&bh, &shelf.bh_online);
        return;
    }
    shelf.lux = (float)(((uint16_t)d[0] << 8) | d[1]) / 1.2f;
    shelf.bh_online = 1;
    bh.fail = 0;
}

/* 离线设备每 10s 重新初始化; 两个都离线时复位 I2C2, 解除总线卡死 */
static void I2C_Retry(uint32_t now)
{
    if ((aht.present && bh.present) || now - retry_tick < I2C_RETRY_MS) return;
    retry_tick = now;

    if (!aht.present && !bh.present) {
        HAL_I2C_DeInit(&hi2c2);
        MX_I2C2_Init();
    }
    if (!aht.present && AHT20_Init()) {
        aht.present = 1;
        aht.fail = 0;
        aht.tick = now - AHT_PERIOD_MS;
    }
    if (!bh.present && BH1750_Init()) {
        bh.present = 1;
        bh.fail = 0;
        bh.tick = now;
    }
}

static void Sensor_Task(uint32_t now)
{
    if (now - adc_tick >= ADC_PERIOD_MS) {
        adc_tick = now;
        ADC_Update(0);
    }
    AHT20_Task(now);
    BH1750_Task(now);
    I2C_Retry(now);
}

/* ===================== 视觉: K230 通信与云台巡检 ===================== */

/*
 * USART1 中断逐字节调用, 按固定 7 字节帧解析:
 *   0x55 0xAA func d0 d1 d2 0xFA
 * 只接收 func=VISION_FUNC_RESULT 的帧, 其余 (如原坐标帧) 丢弃
 */
void SmartShelf_Vision_ParseByte(uint8_t byte)
{
    static uint8_t step = 0;
    static uint8_t buf[4];

    switch (step) {
    case 0:
        if (byte == 0x55) step = 1;
        break;
    case 1:
        step = (byte == 0xAA) ? 2 : (byte == 0x55 ? 1 : 0);
        break;
    case 6:
        if (byte == 0xFA && buf[0] == VISION_FUNC_RESULT) {
            vision_rx.status = buf[1];
            vision_rx.conf   = buf[2];
            vision_rx.cover  = buf[3];
            vision_rx.seq++;
        }
        step = (byte == 0x55) ? 1 : 0;      /* 帧尾错位时该字节可能是下一帧帧头 */
        break;
    default:                                /* step 2~5: func, d0, d1, d2 */
        buf[step - 2] = byte;
        step++;
        break;
    }
}

static void Vision_Request(uint8_t pot)
{
    uint8_t frame[7] = {0x55, 0xAA, VISION_FUNC_REQUEST, pot, 0x00, 0x00, 0xFA};
    HAL_UART_Transmit(&huart1, frame, sizeof(frame), 5);
}

static void Camera_Point(float pan, float tilt)
{
    Servo_Set(SHELF_SERVO_PAN, pan);
    Servo_Set(SHELF_SERVO_TILT, tilt);
}

/* 该盆最近一次识别结果是否为 status, 且置信度足够、未过期 */
static uint8_t Vision_Is(const PotState *p, PlantStatus status, uint32_t now)
{
    return p->plant == status &&
           p->plant_conf >= VISION_CONF_MIN &&
           now - p->vision_tick < VISION_VALID_MS;
}

static void Vision_Task(uint32_t now)
{
    uint8_t status, conf, cover;
    uint16_t seq;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    status = vision_rx.status;
    conf   = vision_rx.conf;
    cover  = vision_rx.cover;
    seq    = vision_rx.seq;
    __set_PRIMASK(primask);

    switch (patrol.step) {
    case PATROL_IDLE:
        if (now - patrol.cycle_tick < PATROL_PERIOD_MS) break;
        patrol.cycle_tick = now;
        patrol.pot = 0;
        patrol.replies = 0;
        Camera_Point(pot_cfg[0].cam_pan, pot_cfg[0].cam_tilt);
        patrol.tick = now;
        patrol.step = PATROL_MOVE;
        break;

    case PATROL_MOVE:
        if (now - patrol.tick < CAM_SETTLE_MS) break;
        patrol.req_seq = seq;
        Vision_Request(patrol.pot);
        patrol.tick = now;
        patrol.step = PATROL_WAIT;
        break;

    case PATROL_WAIT: {
        PotState *p = &shelf.pot[patrol.pot];

        if (seq != patrol.req_seq) {
            /* 请求之后的第一帧结果归属当前花盆 */
            p->plant = (status <= PLANT_PEST) ? (PlantStatus)status : PLANT_UNKNOWN;
            p->plant_conf = conf;
            p->leaf_cover = cover;
            p->vision_tick = now;
            patrol.replies++;
        } else if (now - patrol.tick < VISION_REPLY_TIMEOUT_MS) {
            break;
        } else {
            p->plant = PLANT_UNKNOWN;       /* 超时: 本轮无结果 */
            p->plant_conf = 0;
        }

        if (++patrol.pot < SHELF_POT_NUM) {
            Camera_Point(pot_cfg[patrol.pot].cam_pan, pot_cfg[patrol.pot].cam_tilt);
            patrol.tick = now;
            patrol.step = PATROL_MOVE;
        } else {
            Camera_Point(CAM_HOME_PAN, CAM_HOME_TILT);
            shelf.vision_online = (patrol.replies > 0);
            patrol.step = PATROL_IDLE;
        }
        break;
    }
    }
}

/* ===================== 浇水 ===================== */

static void Pump_Set(uint8_t on)
{
    shelf.pump_on = on;
    Motor_Set(SHELF_MOTOR_PUMP, on ? PUMP_DUTY : 0);
}

static void Water_Start(uint8_t pot, uint32_t now)
{
    water.pot = pot;
    water.phase = WATER_VALVE_LEAD;
    water.phase_tick = now;
    shelf.watering_pot = (int8_t)pot;
    Relay_Write(pot_cfg[pot].valve_port, pot_cfg[pot].valve_pin, 1);
}

/*
 * 每秒调用: 更新各盆补水状态, 空闲时选一盆开始一次脉冲浇水
 * 触发: 土壤湿度 < SOIL_DRY_PCT, 或视觉判定萎蔫且湿度未达目标
 * 结束: 湿度 ≥ SOIL_TARGET_PCT; 两次脉冲间隔 WATER_SOAK_MS 让水分渗透
 */
static void Water_Decide(uint32_t now)
{
    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        PotState *p = &shelf.pot[i];

        if (p->soil_fault) {
            p->thirsty = 0;                 /* 传感器异常时不自动浇水 */
        } else if (p->soil_pct < SOIL_DRY_PCT ||
                   (Vision_Is(p, PLANT_WILT, now) && p->soil_pct < SOIL_TARGET_PCT)) {
            p->thirsty = 1;
        } else if (p->soil_pct >= SOIL_TARGET_PCT) {
            p->thirsty = 0;
        }
    }

    if (water.phase != WATER_IDLE || tank_low) return;

    for (uint8_t k = 0; k < SHELF_POT_NUM; k++) {
        uint8_t i = (uint8_t)((water.rr_next + k) % SHELF_POT_NUM);
        PotState *p = &shelf.pot[i];

        if (!p->thirsty) continue;
        if (now - p->last_water_tick < WATER_SOAK_MS) continue;
        if (p->water_ms_today >= WATER_DAILY_MAX_MS) continue;

        Water_Start(i, now);
        water.rr_next = (uint8_t)((i + 1) % SHELF_POT_NUM);
        break;
    }
}

/* 每次主循环调用, 保证按时停泵; 缺水时立即中止 */
static void Water_Task(uint32_t now)
{
    PotState *p = &shelf.pot[water.pot];

    switch (water.phase) {
    case WATER_IDLE:
        break;

    case WATER_VALVE_LEAD:
        if (tank_low) {
            water.phase = WATER_VALVE_LAG;
            water.phase_tick = now;
        } else if (now - water.phase_tick >= VALVE_LEAD_MS) {
            Pump_Set(1);
            water.pump_tick = now;
            water.phase = WATER_PUMPING;
        }
        break;

    case WATER_PUMPING:
        if (now - water.pump_tick >= WATER_PULSE_MS || tank_low ||
            p->soil_pct >= SOIL_TARGET_PCT)
        {
            Pump_Set(0);
            p->water_ms_today += now - water.pump_tick;
            p->last_water_tick = now;
            water.phase = WATER_VALVE_LAG;
            water.phase_tick = now;
        }
        break;

    case WATER_VALVE_LAG:
        if (now - water.phase_tick >= VALVE_LAG_MS) {
            Relay_Write(pot_cfg[water.pot].valve_port, pot_cfg[water.pot].valve_pin, 0);
            shelf.watering_pot = -1;
            water.phase = WATER_IDLE;
        }
        break;
    }
}

/* ===================== 补光 / 通风 / 遮阳 ===================== */

/* 补光时段内按实测照度闭环调光, 时段外关灯 */
static void Light_Control(void)
{
    uint8_t hour = (uint8_t)(shelf.clock_sec / 3600);
    int16_t pwm = shelf.light_pwm;

    if (hour < LIGHT_ON_HOUR || hour >= LIGHT_OFF_HOUR) {
        pwm = 0;
    } else if (!shelf.bh_online) {
        pwm = LIGHT_FALLBACK_PWM;
    } else if (shelf.lux < LUX_TARGET - LUX_DEADBAND) {
        pwm += LIGHT_STEP;
    } else if (shelf.lux > LUX_TARGET + LUX_DEADBAND) {
        pwm -= LIGHT_STEP;
    }

    if (pwm < 0)   pwm = 0;
    if (pwm > 100) pwm = 100;
    shelf.light_pwm = (uint8_t)pwm;
    Motor_Set(SHELF_MOTOR_LIGHT, shelf.light_pwm);
}

static void Fan_Control(void)
{
    if (!shelf.aht_online) {
        shelf.fan_on = 0;
    } else if (shelf.temp_c > FAN_ON_TEMP || shelf.humi_pct > FAN_ON_HUMI) {
        shelf.fan_on = 1;
    } else if (shelf.temp_c < FAN_ON_TEMP - 2.0f && shelf.humi_pct < FAN_ON_HUMI - 10.0f) {
        shelf.fan_on = 0;
    }
    Relay_Write(SHELF_FAN_GPIO_Port, SHELF_FAN_Pin, shelf.fan_on);
}

/* 强光或高温时遮阳, 每次动作后至少保持 SHADE_HOLD_MS */
static void Shade_Control(uint32_t now)
{
    if (now - shade_tick < SHADE_HOLD_MS) return;

    uint8_t close = shelf.shade_closed;
    uint8_t hot = shelf.aht_online && shelf.temp_c > SHADE_CLOSE_TEMP;
    uint8_t cool = !shelf.aht_online || shelf.temp_c < SHADE_CLOSE_TEMP - 3.0f;

    if ((shelf.bh_online && shelf.lux > SHADE_CLOSE_LUX) || hot) {
        close = 1;
    } else if (shelf.bh_online && shelf.lux < SHADE_OPEN_LUX && cool) {
        close = 0;
    }

    if (close != shelf.shade_closed) {
        shelf.shade_closed = close;
        Servo_Set(SHELF_SERVO_SHADE, close ? SHADE_CLOSED_ANGLE : SHADE_OPEN_ANGLE);
        shade_tick = now;
    }
}

/* ===================== 报警 / 时钟 / 控制任务 ===================== */

static void Alarm_Update(uint32_t now)
{
    uint32_t a = 0;

    if (tank_low)              a |= ALARM_TANK_LOW;
    if (!shelf.aht_online)     a |= ALARM_AHT_OFFLINE;
    if (!shelf.bh_online)      a |= ALARM_BH_OFFLINE;
    if (!shelf.vision_online)  a |= ALARM_VISION_OFFLINE;
    if (shelf.aht_online && (shelf.temp_c > TEMP_ALARM_HIGH || shelf.temp_c < TEMP_ALARM_LOW)) {
        a |= ALARM_TEMP;
    }

    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        const PotState *p = &shelf.pot[i];

        if (p->soil_fault)                  a |= ALARM_SOIL_FAULT;
        if (Vision_Is(p, PLANT_SPOT, now))  a |= ALARM_DISEASE;
        if (Vision_Is(p, PLANT_PEST, now))  a |= ALARM_PEST;
        if (Vision_Is(p, PLANT_WILT, now) && !p->soil_fault && p->soil_pct > SOIL_WET_PCT) {
            a |= ALARM_OVERWATER;           /* 土壤很湿仍萎蔫: 疑似积水烂根 */
        }
        if (p->thirsty && p->water_ms_today >= WATER_DAILY_MAX_MS) {
            a |= ALARM_WATER_LIMIT;
        }
    }
    shelf.alarm = a;
}

/* 软件时钟: 按秒累加 (不受 HAL_GetTick 49 天回绕影响), 跨零点清零每日浇水量 */
static void Clock_Update(uint32_t now)
{
    while (now - clock_tick >= 1000) {
        clock_tick += 1000;
        if (++shelf.clock_sec >= 86400) {
            shelf.clock_sec = 0;
            for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
                shelf.pot[i].water_ms_today = 0;
            }
        }
    }
}

static void Control_Task(uint32_t now)
{
    Clock_Update(now);
    if (now - control_tick < CONTROL_PERIOD_MS) return;
    control_tick = now;

    Water_Decide(now);
    Light_Control();
    Fan_Control();
    Shade_Control(now);
    Alarm_Update(now);
}

/* 有需提示的报警时每 3s 鸣叫 200ms */
static void Buzzer_Task(uint32_t now)
{
    Buzzer_Write((shelf.alarm & ALARM_BEEP_MASK) && (now % 3000) < 200);
}

/* ===================== 状态上报 (USART3) ===================== */

/*
 * 每行一帧, 整数格式 (newlib-nano 默认不支持 %f):
 * $SHELF,t=08:15,T=26.4,H=55.2,L=3120,W=76,S=42/61/58,P=1/1/2,C=35/40/28,PUMP=0,LED=45,FAN=0,SHD=0,ALM=0x0000
 *   T 温度  H 湿度  L 照度  W 水位%  S 各盆土壤湿度%  P 各盆 PlantStatus (255=未知)  C 绿叶覆盖率%
 *   PUMP 正在浇水的花盆编号+1 (0=未浇水)  LED 补光亮度%  SHD 1=遮阳  ALM 报警位
 */
#define REPORT_APPEND(...)                                                      \
    do {                                                                        \
        if (len < sizeof(line))                                                 \
            len += (size_t)snprintf(line + len, sizeof(line) - len, __VA_ARGS__); \
    } while (0)

static void Report_Task(uint32_t now)
{
    static char line[256];
    size_t len = 0;

    if (now - report_tick < REPORT_PERIOD_MS) return;
    report_tick = now;

    int32_t  t10 = (int32_t)(shelf.temp_c * 10.0f + (shelf.temp_c >= 0 ? 0.5f : -0.5f));
    uint32_t h10 = (uint32_t)(shelf.humi_pct * 10.0f + 0.5f);
    uint32_t t_abs = (uint32_t)(t10 < 0 ? -t10 : t10);

    REPORT_APPEND("$SHELF,t=%02lu:%02lu,T=%s%lu.%lu,H=%lu.%lu,L=%lu,W=%u,S=",
                  (unsigned long)(shelf.clock_sec / 3600), (unsigned long)(shelf.clock_sec / 60 % 60),
                  t10 < 0 ? "-" : "", (unsigned long)(t_abs / 10), (unsigned long)(t_abs % 10),
                  (unsigned long)(h10 / 10), (unsigned long)(h10 % 10),
                  (unsigned long)shelf.lux, shelf.tank_pct);
    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        REPORT_APPEND("%s%u", i ? "/" : "", shelf.pot[i].soil_pct);
    }
    REPORT_APPEND(",P=");
    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        REPORT_APPEND("%s%u", i ? "/" : "", (unsigned)shelf.pot[i].plant);
    }
    REPORT_APPEND(",C=");
    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        REPORT_APPEND("%s%u", i ? "/" : "", shelf.pot[i].leaf_cover);
    }
    REPORT_APPEND(",PUMP=%d,LED=%u,FAN=%u,SHD=%u,ALM=0x%04lX\r\n",
                  shelf.watering_pot + 1, shelf.light_pwm, shelf.fan_on,
                  shelf.shade_closed, (unsigned long)shelf.alarm);

    if (len >= sizeof(line)) len = sizeof(line) - 1;
    HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)len, 50);
}

/* ===================== 独立看门狗 ===================== */

#if SHELF_USE_IWDG
/* LSI 32kHz / 64 = 500Hz, 重装载 1000 → 约 2s; 调试暂停时冻结 */
static void Shelf_IWDG_Init(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    IWDG->KR = 0xCCCC;
    IWDG->KR = 0x5555;
    IWDG->PR = 4;
    IWDG->RLR = 1000;
    while (IWDG->SR) {}
    IWDG->KR = 0xAAAA;
}
#endif

/* ===================== 公开接口 ===================== */

void SmartShelf_Init(void)
{
    memset(&shelf, 0, sizeof(shelf));
    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        shelf.pot[i].plant = PLANT_UNKNOWN;
    }
    shelf.watering_pot = -1;
    shelf.clock_sec = (uint32_t)CLOCK_BOOT_HOUR * 3600;

    Shelf_GPIO_Init();
    Shelf_ADC_Init();

    /* 执行器进入安全状态 */
    Pump_Set(0);
    Motor_Set(SHELF_MOTOR_LIGHT, 0);
    Servo_Set(SHELF_SERVO_SHADE, SHADE_OPEN_ANGLE);
    Camera_Point(CAM_HOME_PAN, CAM_HOME_TILT);

    /* I2C 传感器 (System_Init 末尾已延时 500ms, 满足 AHT20 上电 40ms) */
    aht.present = AHT20_Init();
    bh.present = BH1750_Init();

    ADC_Update(1);

    /* USART1 接收中断: 原工程未开启 RXNE, USART1_IRQHandler 不会被触发 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

    /* 上电提示音 */
    Buzzer_Write(1);
    HAL_Delay(100);
    Buzzer_Write(0);

    uint32_t now = HAL_GetTick();
    adc_tick = control_tick = report_tick = clock_tick = retry_tick = now;
    aht.tick = now - AHT_PERIOD_MS;         /* 立即触发第一次测量 */
    bh.tick = now;
    shade_tick = now - SHADE_HOLD_MS;
    patrol.cycle_tick = now - PATROL_PERIOD_MS + PATROL_FIRST_DELAY_MS;
    for (uint8_t i = 0; i < SHELF_POT_NUM; i++) {
        shelf.pot[i].last_water_tick = now; /* 首次浇水前等待一个渗透周期, 读数先稳定 */
    }

#if SHELF_USE_IWDG
    Shelf_IWDG_Init();
#endif
}

void SmartShelf_Loop(void)
{
    uint32_t now = HAL_GetTick();

    Sensor_Task(now);
    Vision_Task(now);
    Control_Task(now);
    Water_Task(now);
    Buzzer_Task(now);
    Report_Task(now);

#if SHELF_USE_IWDG
    IWDG->KR = 0xAAAA;
#endif
}

void SmartShelf_SetTime(uint8_t hour, uint8_t minute)
{
    if (hour > 23 || minute > 59) return;
    shelf.clock_sec = (uint32_t)hour * 3600 + (uint32_t)minute * 60;
}

const SmartShelf_State *SmartShelf_GetState(void)
{
    return &shelf;
}
