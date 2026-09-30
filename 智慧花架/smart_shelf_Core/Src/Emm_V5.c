#include "Emm_V5.h"
#include <string.h>

/* ========== Multi-channel support ========== */

Emm_V5_Channel_t Emm_Ch1 = { .huart = NULL };  /* UART4 (SM1) */
Emm_V5_Channel_t Emm_Ch2 = { .huart = NULL };  /* UART5 (SM2) */

/* Pointer to the currently active channel for command transmission */
static Emm_V5_Channel_t *emm_active_ch = NULL;

/* MMCL buffer */
__IO uint16_t MMCL_count = 0;
__IO uint16_t MMCL_cmd[MMCL_LEN] = {0};

void Emm_V5_Init(Emm_V5_Channel_t *ch)
{
	if (ch == NULL || ch->huart == NULL) return;

	ch->rxFrameFlag = false;
	ch->rxCount = 0;

	/* Enable UART IDLE interrupt */
	__HAL_UART_ENABLE_IT(ch->huart, UART_IT_IDLE);

	/* Start DMA reception in circular mode */
	HAL_UART_Receive_DMA(ch->huart, (uint8_t *)ch->rxCmd, EMM_V5_CMD_LEN);

	/* Set active channel to first initialized one if none set */
	if (emm_active_ch == NULL) {
		emm_active_ch = ch;
	}
}

void Emm_V5_Set_Channel(Emm_V5_Channel_t *ch)
{
	emm_active_ch = ch;
}

void Emm_V5_IDLE_IRQHandler(UART_HandleTypeDef *huart)
{
	Emm_V5_Channel_t *ch = NULL;

	if (huart->Instance == UART4) ch = &Emm_Ch1;
	else if (huart->Instance == UART5) ch = &Emm_Ch2;
	else return;

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);

		HAL_UART_DMAStop(huart);

		ch->rxCount = EMM_V5_CMD_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		ch->rxFrameFlag = true;

		HAL_UART_Receive_DMA(huart, (uint8_t *)ch->rxCmd, EMM_V5_CMD_LEN);
	}
}

/* Helper: transmit via active channel (blocking, safe for static buffers) */
static inline void Emm_Tx(uint8_t *data, uint16_t len)
{
	if (emm_active_ch == NULL) return;
	HAL_UART_Transmit(emm_active_ch->huart, data, len, 10);
}

/* ========== System / Utility ========== */

void Emm_V5_Trig_Encoder_Cal(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x06;  cmd[2] = 0x45;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

void Emm_V5_Reset_Motor(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x08;  cmd[2] = 0x97;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x0A;  cmd[2] = 0x6D;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x0E;  cmd[2] = 0x52;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

void Emm_V5_Restore_Motor(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x0F;  cmd[2] = 0x5F;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

/* ========== Motion Control ========== */

void Emm_V5_Multi_Motor_Cmd(uint8_t addr)
{
	uint16_t i = 0, j = 0, len = 0;
	static uint8_t cmd[MMCL_LEN] = {0};

	if (MMCL_count > 0)
	{
		len = MMCL_count + 5;
		cmd[0] = addr;
		cmd[1] = 0xAA;
		cmd[2] = (uint8_t)(len >> 8);
		cmd[3] = (uint8_t)(len);
		for (i = 0, j = 4; i < MMCL_count; i++, j++) { cmd[j] = MMCL_cmd[i]; }
		cmd[j] = 0x6B; ++j;
		Emm_Tx((uint8_t *)cmd, j);
		MMCL_count = 0;
	}
	else
	{
		MMCL_count = 0;
	}
}

void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xF3;  cmd[2] = 0xAB;
	cmd[3] = (uint8_t)state;  cmd[4] = snF;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xF6;  cmd[2] = dir;
	cmd[3] = (uint8_t)(vel >> 8);  cmd[4] = (uint8_t)(vel >> 0);
	cmd[5] = acc;  cmd[6] = snF;  cmd[7] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 8);
}

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
	static uint8_t cmd[16] = {0};
	cmd[0]  = addr;  cmd[1] = 0xFD;  cmd[2] = dir;
	cmd[3]  = (uint8_t)(vel >> 8);  cmd[4] = (uint8_t)(vel >> 0);
	cmd[5]  = acc;
	cmd[6]  = (uint8_t)(clk >> 24);  cmd[7] = (uint8_t)(clk >> 16);
	cmd[8]  = (uint8_t)(clk >> 8);   cmd[9] = (uint8_t)(clk >> 0);
	cmd[10] = raF;  cmd[11] = snF;   cmd[12] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 13);
}

void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xFE;  cmd[2] = 0x98;  cmd[3] = snF;  cmd[4] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 5);
}

void Emm_V5_Synchronous_motion(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xFF;  cmd[2] = 0x66;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

/* ========== Homing ========== */

void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x93;  cmd[2] = 0x88;  cmd[3] = svF;  cmd[4] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 5);
}

void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x9A;  cmd[2] = o_mode;  cmd[3] = snF;  cmd[4] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 5);
}

void Emm_V5_Origin_Interrupt(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x9C;  cmd[2] = 0x48;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

void Emm_V5_Origin_Read_Params(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x22;  cmd[2] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 3);
}

void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
	static uint8_t cmd[32] = {0};
	cmd[0] = addr;  cmd[1] = 0x4C;  cmd[2] = 0xAE;  cmd[3] = svF;
	cmd[4] = o_mode;  cmd[5] = o_dir;
	cmd[6]  = (uint8_t)(o_vel >> 8);  cmd[7]  = (uint8_t)(o_vel >> 0);
	cmd[8]  = (uint8_t)(o_tm >> 24);  cmd[9]  = (uint8_t)(o_tm >> 16);
	cmd[10] = (uint8_t)(o_tm >> 8);   cmd[11] = (uint8_t)(o_tm >> 0);
	cmd[12] = (uint8_t)(sl_vel >> 8); cmd[13] = (uint8_t)(sl_vel >> 0);
	cmd[14] = (uint8_t)(sl_ma >> 8);  cmd[15] = (uint8_t)(sl_ma >> 0);
	cmd[16] = (uint8_t)(sl_ms >> 8);  cmd[17] = (uint8_t)(sl_ms >> 0);
	cmd[18] = potF;  cmd[19] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 20);
}

/* ========== Read System Params ========== */

static uint8_t SysParams_Code(SysParams_t s)
{
	switch (s) {
		case S_VBUS:  return 0x24;
		case S_CBUS:  return 0x26;
		case S_CPHA:  return 0x27;
		case S_ENCO:  return 0x29;
		case S_CLKC:  return 0x30;
		case S_ENCL:  return 0x31;
		case S_CLKI:  return 0x32;
		case S_TPOS:  return 0x33;
		case S_SPOS:  return 0x34;
		case S_VEL:   return 0x35;
		case S_CPOS:  return 0x36;
		case S_PERR:  return 0x37;
		case S_VBAT:  return 0x38;
		case S_TEMP:  return 0x39;
		case S_FLAG:  return 0x3A;
		case S_OFLAG: return 0x3B;
		case S_OAF:   return 0x3C;
		case S_PIN:   return 0x3D;
		default:      return 0x00;
	}
}

void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
	uint8_t i = 0;
	static uint8_t cmd[16] = {0};
	cmd[i++] = addr;
	cmd[i++] = 0x11;
	cmd[i++] = 0x18;
	cmd[i++] = SysParams_Code(s);
	cmd[i++] = (uint8_t)(time_ms >> 8);
	cmd[i++] = (uint8_t)(time_ms >> 0);
	cmd[i++] = 0x6B;
	Emm_Tx((uint8_t *)cmd, i);
}

void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
	uint8_t i = 0;
	static uint8_t cmd[16] = {0};
	cmd[i++] = addr;
	cmd[i++] = SysParams_Code(s);
	cmd[i++] = 0x6B;
	Emm_Tx((uint8_t *)cmd, i);
}

/* ========== Configuration ========== */

void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xAE;  cmd[2] = 0x4B;  cmd[3] = svF;  cmd[4] = id;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x84;  cmd[2] = 0x8A;  cmd[3] = svF;  cmd[4] = mstep;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x50;  cmd[2] = pdf;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

void Emm_V5_Read_Opt_Param_Sta(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x1A;  cmd[2] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype)
{
	uint8_t MotType = mottype ? 25 : 50;
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xD7;  cmd[2] = 0x35;  cmd[3] = svF;  cmd[4] = MotType;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xD5;  cmd[2] = 0x69;  cmd[3] = svF;  cmd[4] = fwtype;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x46;  cmd[2] = 0x69;  cmd[3] = svF;  cmd[4] = ctrl_mode;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xD4;  cmd[2] = 0x60;  cmd[3] = svF;  cmd[4] = dir;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lock)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xD0;  cmd[2] = 0xB3;  cmd[3] = svF;  cmd[4] = lock;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x4F;  cmd[2] = 0x71;  cmd[3] = svF;  cmd[4] = s_vel;  cmd[5] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 6);
}

void Emm_V5_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x44;  cmd[2] = 0x33;  cmd[3] = svF;
	cmd[4] = (uint8_t)(om_ma >> 8);  cmd[5] = (uint8_t)(om_ma >> 0);
	cmd[6] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 7);
}

void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x45;  cmd[2] = 0x66;  cmd[3] = svF;
	cmd[4] = (uint8_t)(foc_mA >> 8);  cmd[5] = (uint8_t)(foc_mA >> 0);
	cmd[6] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 7);
}

void Emm_V5_Read_PID_Params(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x21;  cmd[2] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 3);
}

void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd)
{
	static uint8_t cmd[20] = {0};
	cmd[0] = addr;  cmd[1] = 0x4A;  cmd[2] = 0xC3;  cmd[3] = svF;
	cmd[4]  = (uint8_t)(kp >> 24);  cmd[5]  = (uint8_t)(kp >> 16);
	cmd[6]  = (uint8_t)(kp >> 8);   cmd[7]  = (uint8_t)(kp >> 0);
	cmd[8]  = (uint8_t)(ki >> 24);  cmd[9]  = (uint8_t)(ki >> 16);
	cmd[10] = (uint8_t)(ki >> 8);   cmd[11] = (uint8_t)(ki >> 0);
	cmd[12] = (uint8_t)(kd >> 24);  cmd[13] = (uint8_t)(kd >> 16);
	cmd[14] = (uint8_t)(kd >> 8);   cmd[15] = (uint8_t)(kd >> 0);
	cmd[16] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 17);
}

void Emm_V5_Read_DMX512_Params(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x49;  cmd[2] = 0x78;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step)
{
	static uint8_t cmd[32] = {0};
	cmd[0] = addr;  cmd[1] = 0xD9;  cmd[2] = 0x90;  cmd[3] = svF;
	cmd[4]  = (uint8_t)(tch >> 8);  cmd[5]  = (uint8_t)(tch >> 0);
	cmd[6]  = nch;  cmd[7] = mode;
	cmd[8]  = (uint8_t)(vel >> 8);  cmd[9]  = (uint8_t)(vel >> 0);
	cmd[10] = (uint8_t)(acc >> 8);  cmd[11] = (uint8_t)(acc >> 0);
	cmd[12] = (uint8_t)(vel_step >> 8);  cmd[13] = (uint8_t)(vel_step >> 0);
	cmd[14] = (uint8_t)(pos_step >> 24); cmd[15] = (uint8_t)(pos_step >> 16);
	cmd[16] = (uint8_t)(pos_step >> 8);  cmd[17] = (uint8_t)(pos_step >> 0);
	cmd[18] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 19);
}

void Emm_V5_Read_Pos_Window(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x41;  cmd[2] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xD1;  cmd[2] = 0x07;  cmd[3] = svF;
	cmd[4] = (uint8_t)(prw >> 8);  cmd[5] = (uint8_t)(prw >> 0);
	cmd[6] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 7);
}

void Emm_V5_Read_Otocp(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x13;  cmd[2] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xD3;  cmd[2] = 0x56;  cmd[3] = svF;
	cmd[4]  = (uint8_t)(otp >> 8);  cmd[5]  = (uint8_t)(otp >> 0);
	cmd[6]  = (uint8_t)(ocp >> 8);  cmd[7]  = (uint8_t)(ocp >> 0);
	cmd[8]  = (uint8_t)(time_ms >> 8);  cmd[9]  = (uint8_t)(time_ms >> 0);
	cmd[10] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 11);
}

void Emm_V5_Read_Heart_Protect(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x16;  cmd[2] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x68;  cmd[2] = 0x38;  cmd[3] = svF;
	cmd[4] = (uint8_t)(hp >> 24);  cmd[5] = (uint8_t)(hp >> 16);
	cmd[6] = (uint8_t)(hp >> 8);   cmd[7] = (uint8_t)(hp >> 0);
	cmd[8] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 9);
}

void Emm_V5_Read_Integral_Limit(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x23;  cmd[2] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x4B;  cmd[2] = 0x57;  cmd[3] = svF;
	cmd[4] = (uint8_t)(il >> 24);  cmd[5] = (uint8_t)(il >> 16);
	cmd[6] = (uint8_t)(il >> 8);   cmd[7] = (uint8_t)(il >> 0);
	cmd[8] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 9);
}

void Emm_V5_Read_System_State_Params(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x43;  cmd[2] = 0x7A;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

void Emm_V5_Read_Motor_Conf_Params(uint8_t addr)
{
	static uint8_t cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x42;  cmd[2] = 0x6C;  cmd[3] = 0x6B;
	Emm_Tx((uint8_t *)cmd, 4);
}

/* ========== MMCL variants ========== */

void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x06;  cmd[2] = 0x45;  cmd[3] = 0x6B;
	for (j = 0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Reset_Motor(uint8_t addr)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x08;  cmd[2] = 0x97;  cmd[3] = 0x6B;
	for (j = 0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x0A;  cmd[2] = 0x6D;  cmd[3] = 0x6B;
	for (j = 0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x0E;  cmd[2] = 0x52;  cmd[3] = 0x6B;
	for (j = 0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Restore_Motor(uint8_t addr)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x0F;  cmd[2] = 0x5F;  cmd[3] = 0x6B;
	for (j = 0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xF3;  cmd[2] = 0xAB;
	cmd[3] = (uint8_t)state;  cmd[4] = snF;  cmd[5] = 0x6B;
	for (j = 0; j < 6; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xF6;  cmd[2] = dir;
	cmd[3] = (uint8_t)(vel >> 8);  cmd[4] = (uint8_t)(vel >> 0);
	cmd[5] = acc;  cmd[6] = snF;  cmd[7] = 0x6B;
	for (j = 0; j < 8; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0]  = addr;  cmd[1] = 0xFD;  cmd[2] = dir;
	cmd[3]  = (uint8_t)(vel >> 8);  cmd[4] = (uint8_t)(vel >> 0);
	cmd[5]  = acc;
	cmd[6]  = (uint8_t)(clk >> 24);  cmd[7] = (uint8_t)(clk >> 16);
	cmd[8]  = (uint8_t)(clk >> 8);   cmd[9] = (uint8_t)(clk >> 0);
	cmd[10] = raF;  cmd[11] = snF;   cmd[12] = 0x6B;
	for (j = 0; j < 13; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xFE;  cmd[2] = 0x98;  cmd[3] = snF;  cmd[4] = 0x6B;
	for (j = 0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Synchronous_motion(uint8_t addr)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0xFF;  cmd[2] = 0x66;  cmd[3] = 0x6B;
	for (j = 0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x93;  cmd[2] = 0x88;  cmd[3] = svF;  cmd[4] = 0x6B;
	for (j = 0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x9A;  cmd[2] = o_mode;  cmd[3] = snF;  cmd[4] = 0x6B;
	for (j = 0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr)
{
	uint8_t j = 0, cmd[16] = {0};
	cmd[0] = addr;  cmd[1] = 0x9C;  cmd[2] = 0x48;  cmd[3] = 0x6B;
	for (j = 0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
	uint8_t j = 0, cmd[32] = {0};
	cmd[0] = addr;  cmd[1] = 0x4C;  cmd[2] = 0xAE;  cmd[3] = svF;
	cmd[4] = o_mode;  cmd[5] = o_dir;
	cmd[6]  = (uint8_t)(o_vel >> 8);  cmd[7]  = (uint8_t)(o_vel >> 0);
	cmd[8]  = (uint8_t)(o_tm >> 24);  cmd[9]  = (uint8_t)(o_tm >> 16);
	cmd[10] = (uint8_t)(o_tm >> 8);   cmd[11] = (uint8_t)(o_tm >> 0);
	cmd[12] = (uint8_t)(sl_vel >> 8); cmd[13] = (uint8_t)(sl_vel >> 0);
	cmd[14] = (uint8_t)(sl_ma >> 8);  cmd[15] = (uint8_t)(sl_ma >> 0);
	cmd[16] = (uint8_t)(sl_ms >> 8);  cmd[17] = (uint8_t)(sl_ms >> 0);
	cmd[18] = potF;  cmd[19] = 0x6B;
	for (j = 0; j < 20; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
	uint8_t i = 0, j = 0;
	uint8_t cmd[16] = {0};
	cmd[i++] = addr;
	cmd[i++] = 0x11;
	cmd[i++] = 0x18;
	cmd[i++] = SysParams_Code(s);
	cmd[i++] = (uint8_t)(time_ms >> 8);
	cmd[i++] = (uint8_t)(time_ms >> 0);
	cmd[i++] = 0x6B;
	for (j = 0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
	uint8_t i = 0, j = 0;
	uint8_t cmd[16] = {0};
	cmd[i++] = addr;
	cmd[i++] = SysParams_Code(s);
	cmd[i++] = 0x6B;
	for (j = 0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}
