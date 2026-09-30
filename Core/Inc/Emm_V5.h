#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

#define ABS(x) ((x) > 0 ? (x) : -(x))

/* Emm_V5 receive buffer size */
#define EMM_V5_CMD_LEN  255

typedef enum {
	S_VBUS  = 5,
	S_CBUS  = 6,
	S_CPHA  = 7,
	S_ENCO  = 8,
	S_CLKC  = 9,
	S_ENCL  = 10,
	S_CLKI  = 11,
	S_TPOS  = 12,
	S_SPOS  = 13,
	S_VEL   = 14,
	S_CPOS  = 15,
	S_PERR  = 16,
	S_VBAT  = 17,
	S_TEMP  = 18,
	S_FLAG  = 19,
	S_OFLAG = 20,
	S_OAF   = 21,
	S_PIN   = 22,
} SysParams_t;

/* MMCL buffer */
#define MMCL_LEN  512
extern __IO uint16_t MMCL_count;
extern __IO uint16_t MMCL_cmd[MMCL_LEN];

/*
 * Per-channel receive state for Emm_V5 (one per UART used for stepper motors).
 */
typedef struct {
	UART_HandleTypeDef *huart;
	volatile uint8_t   rxCmd[EMM_V5_CMD_LEN];
	volatile uint16_t  rxCount;
	volatile bool      rxFrameFlag;
} Emm_V5_Channel_t;

/* Pre-defined channels (allocated in Emm_V5.c) */
extern Emm_V5_Channel_t Emm_Ch1;  /* UART4 (SM1) */
extern Emm_V5_Channel_t Emm_Ch2;  /* UART5 (SM2) */

/*
 * Call once after UART init to start DMA reception on a channel.
 * Also enable the IDLE interrupt for that UART.
 */
void Emm_V5_Init(Emm_V5_Channel_t *ch);

/*
 * Switch the active channel for subsequent Emm_V5_* calls.
 * Must be set before calling any command function.
 */
void Emm_V5_Set_Channel(Emm_V5_Channel_t *ch);

/*
 * Called from USARTx_IRQHandler when IDLE flag is detected.
 * Pass the UART handle; it finds the matching channel.
 */
void Emm_V5_IDLE_IRQHandler(UART_HandleTypeDef *huart);

/* ========== System / Utility ========== */
void Emm_V5_Trig_Encoder_Cal(uint8_t addr);
void Emm_V5_Reset_Motor(uint8_t addr);
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Restore_Motor(uint8_t addr);

/* ========== Motion Control ========== */
void Emm_V5_Multi_Motor_Cmd(uint8_t addr);
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Synchronous_motion(uint8_t addr);

/* ========== Homing ========== */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void Emm_V5_Origin_Interrupt(uint8_t addr);
void Emm_V5_Origin_Read_Params(uint8_t addr);
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

/* ========== Read System Params ========== */
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);

/* ========== Configuration ========== */
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf);
void Emm_V5_Read_Opt_Param_Sta(uint8_t addr);
void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lockbtn);
void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
void Emm_V5_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma);
void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
void Emm_V5_Read_PID_Params(uint8_t addr);
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd);
void Emm_V5_Read_DMX512_Params(uint8_t addr);
void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);
void Emm_V5_Read_Pos_Window(uint8_t addr);
void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
void Emm_V5_Read_Otocp(uint8_t addr);
void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);
void Emm_V5_Read_Heart_Protect(uint8_t addr);
void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
void Emm_V5_Read_Integral_Limit(uint8_t addr);
void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);
void Emm_V5_Read_System_State_Params(uint8_t addr);
void Emm_V5_Read_Motor_Conf_Params(uint8_t addr);

/* ========== MMCL (batch command) variants ========== */
void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr);
void Emm_V5_MMCL_Reset_Motor(uint8_t addr);
void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_MMCL_Restore_Motor(uint8_t addr);
void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF);
void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_MMCL_Synchronous_motion(uint8_t addr);
void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF);
void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr);
void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s);

#endif
