#ifndef __MOTOR_HALL_H__
#define __MOTOR_HALL_H__

#include <stdint.h>
#ifndef HOMING_STOP_AFTER_INIT
#define HOMING_STOP_AFTER_INIT 1   /* 回零完成后是否清指令并停机 */
#endif

/* —— 停机清 PID —— */
#ifndef PID_CLEAR_ON_STOP_ENABLE
#define PID_CLEAR_ON_STOP_ENABLE   1     /* 正常停机后清本侧速度环；两侧都停后清同步环 */
#endif
#ifndef PID_CLEAR_ON_STALL_ENABLE
#define PID_CLEAR_ON_STALL_ENABLE  1     /* 堵转/到位停机时清两侧速度环 + 同步环 */
#endif
#ifndef PID_CLEAR_AFTER_STOP_MS
#define PID_CLEAR_AFTER_STOP_MS    10    /* 判定“停稳”后再清，避免误触发（ms） */
#endif

/* —— 堵转/到位停机 —— */
#ifndef STALL_DET_ENABLE
#define STALL_DET_ENABLE 1
#endif
#ifndef STALL_NO_TOGGLE_MS         /* 连续无霍尔跳变判定到位/堵转的时间窗口 */
#define STALL_NO_TOGGLE_MS 200     /* 200~300ms 视机构阻尼调整 */
#endif
#ifndef STALL_MIN_DUTY_PERCENT      /* 只有占空≥该值才认为“在尝试运动” */
#define STALL_MIN_DUTY_PERCENT 10
#endif
#ifndef STALL_RELEASE_ON_DIR_CHANGE /* 换向后自动解除堵转锁存 */
#define STALL_RELEASE_ON_DIR_CHANGE 1
#endif
#ifndef STALL_STOP_BOTH_ON_TRIGGER  /* 任一侧触发 → 两侧都停机 */
#define STALL_STOP_BOTH_ON_TRIGGER 1
#endif

/* —— 运行态判定（用于“运行中二次下发指令=停机”） —— */
#ifndef RUN_MOVING_WINDOW_MS        /* 近期有霍尔跳变则认为在运行 */
#define RUN_MOVING_WINDOW_MS 20     /* 20ms 内有跳变即“在运行” */
#endif

/* —— 同步增强参数（已有的保持不变，这里只新增几个“止抖”宏） —— */
#ifndef SYNC_MICRO_ERR_COUNTS
/* 微误差带：小于这个误差时，倾向于不再纠偏（建议≈ 1/32 圈） */
#define SYNC_MICRO_ERR_COUNTS   (HALL_CPR/32)
#endif

#ifndef SYNC_MICRO_DERR_COUNTS
/* 误差变化阈值（counts/ms）：误差变化很小也视作静止 */
#define SYNC_MICRO_DERR_COUNTS  2
#endif

#ifndef SYNC_STILL_HOLD_MS
/* 连续静止判定时间，达到后短时间冻结纠偏 */
#define SYNC_STILL_HOLD_MS      20
#endif

#ifndef DUTY_QUANT_PERCENT
/* 输出占空量化步进（％）：减少 0/±1% 抖动来回切换 */
#define DUTY_QUANT_PERCENT      1
#endif

#ifndef FINAL_DUTY_SLEW_PCT_PER_MS
/* 最终占空的额外斜率限制（％/ms）：避免同步纠偏造成的尖跳 */
#define FINAL_DUTY_SLEW_PCT_PER_MS  2
#endif

/* —— 同步增强参数 —— */
#ifndef SYNC_ERR_CATCH_COUNTS
/* 进入“强制追赶模式”的误差阈值：默认 0.5 圈 */
#define SYNC_ERR_CATCH_COUNTS   (HALL_CPR/2)
#endif

#ifndef SYNC_CATCH_HOLD_DUTY
/* 追赶时对领先侧占空的“限流”目标（％） */
#define SYNC_CATCH_HOLD_DUTY    10
#endif

#ifndef SYNC_CATCH_BOOST_DUTY
/* 追赶时对落后侧的“加油”幅度（％） */
#define SYNC_CATCH_BOOST_DUTY   10
#endif

#ifndef SYNC_CORR_MIN_LIMIT_PERCENT
/* 纠偏动态限幅的绝对下限（％），避免低占空时纠偏太小 */
#define SYNC_CORR_MIN_LIMIT_PERCENT  5
#endif

#ifndef SPEED_SYNC_GATE_QPPS
/* 若门控太严导致经常跳过同步，可按需要调大 */
#define SPEED_SYNC_GATE_QPPS  50
#endif

/* ================= 集中可调参数 ================= */
/* 正常控制下的底位判定，默认沿用回零时的无跳变阈值 */
#ifndef RUN_BOTTOM_INIT_ENABLE
#define RUN_BOTTOM_INIT_ENABLE 1
#endif

#ifndef RUN_BOTTOM_NO_TOGGLE_MS
#define RUN_BOTTOM_NO_TOGGLE_MS HOMING_NO_TOGGLE_MS
#endif


/* —— 硬件/基本参数 —— */
#define TIM1_CLOCK_HZ               32000000UL   /* TIM1 时钟 */
#define PWM_FREQ_HZ                 20000        /* PWM 频率 */
#define PWM_MAX_DUTY                1000         /* 期望ARR（在频率满足下尽量接近） */
#define HALL_CPR                    4            /* 霍尔正交 4cpr */

/* —— 运动学/保护 —— */
#define POS_LIMIT_TURNS             2            /* 极端兜底：>2圈偏差时抑制领先侧（保留） */

/* —— 霍尔方向适配 —— */
#define HALL_SIGN_M0                (+1)
#define HALL_SIGN_M1                (+1)

/* —— 霍尔数字滤波/去抖 —— */
#define HALL_FILTER_ENABLE          1           /* 1=启用积分+迟滞滤波 */
#define HALL_INT_MAX                4           /* 积分器上限 */
#define HALL_INT_TH_HI              3           /* >=TH_HI 判 1 */
#define HALL_INT_TH_LO              1           /* <=TH_LO 判 0 */
#define HALL_BLANK_ON_START_MS      2           /* 刚启动时抑制窗口 */
#define HALL_BLANK_ON_REV_MS        3           /* 反转切换时抑制窗口 */

/* —— 速度滤波与死区 —— */
#define SPEED_IIR_ALPHA_NUM         1
#define SPEED_IIR_ALPHA_DEN         8
#define SPEED_QPPS_DEADBAND         3

/* —— 反转保护/同步冻结 —— */
#define REVERSE_HOLD_MS             40
#define SYNC_FREEZE_MS              100

/* —— 上电回零（命令触发） —— */
#define HOMING_INIT_ENABLE          1
#define HOMING_TRIGGER_ON_CMD       1           /* 必须先收到伸/收命令才启动回零 */
#define HOMING_PWM_PERCENT          50
#define HOMING_NO_TOGGLE_MS         500
#define HOMING_INIT_POS             3000
#define HOMING_RETRACT_DIR_M0       2           /* 1=正, 2=反（按你接线） */
#define HOMING_RETRACT_DIR_M1       2
/* —— 回零“武装延时”：未见到边沿前，至少驱动这么久后才允许用“无翻转=到底” —— */
#define HOMING_ARM_DELAY_MS         200         /* 建议 150~300ms */

/* —— 同步PID（两电机） —— */
#define SYNC_PID_K_SHIFT            10
#define SYNC_PID_KP_DEFAULT         60
#define SYNC_PID_KI_DEFAULT         2
#define SYNC_PID_KD_DEFAULT         0
#define SYNC_PID_I_ACC_LIMIT        6000
#define SYNC_PID_CORR_LIMIT_PERCENT 20
#define SYNC_MIN_DUTY_PERCENT       8
#define SYNC_CORR_FRAC_NUM          1           /* 动态限幅：<= min(base0,base1) * 1/4 */
#define SYNC_CORR_FRAC_DEN          4
#define SYNC_ERR_DEADBAND_COUNTS    1           /* |err|<1计数不纠偏 */
#define SYNC_CORR_SLEW_PCT_PER_MS   2           /* 纠偏每ms最大变化(百分点) */

/* —— 速度PID（每电机） —— */
#define SPEED_PID_K_SHIFT           10
#define SPEED_PID_KP_DEFAULT        24
#define SPEED_PID_KI_DEFAULT        2
#define SPEED_PID_KD_DEFAULT        0
#define SPEED_PID_I_ACC_LIMIT       12000
#define SPEED_PID_OUT_LIMIT_PERCENT 100

/* —— 速度前馈（按目标qpps给基线占空） —— */
#define SPEED_FF_ENABLE             1           /* u_out = PI + u_ff */
#define SPEED_FF_K_NUM              1           /* u_ff(%) = |tgt_qpps| * 1/10 */
#define SPEED_FF_K_DEN              10          /* 3000rpm→200qpps→u_ff≈20% */
#define SPEED_FF_MIN_PERCENT        0

/* —— 同步门限：速度未到位先不做同步 —— */
#define SYNC_GATE_ENABLE            1

/* —— 软启动/斜坡与最小导通 —— */
#define SOFT_START_ENABLE           1
#define START_KICK_DUTY_PERCENT     12
#define START_KICK_MS               60
#define START_SPEED_OK_QPPS         3
#define START_SPEED_OK_MS           40
#define SYNC_ENABLE_DELAY_MS        80
#define DUTY_SLEW_UP_PCT_PER_MS     2
#define DUTY_SLEW_DOWN_PCT_PER_MS   3
#define DUTY_MIN_ON_PERCENT         6
#define DUTY_MIN_ON_TIME_MS         30

/* —— 单位换算 —— */
#define RPM_TO_QPPS(rpm)   ((int32_t)((int64_t)(rpm) * (HALL_CPR) / 60))
#define QPPS_TO_RPM(qpps)  ((int32_t)((int64_t)(qpps) * 60 / (HALL_CPR)))

/* ================= 公开 API ================= */
void Motor_InitPWMAndHall(void);
void Motor_Deinit(void);
void MotorRTOS_Init(void);

int32_t Motor_GetTurns(uint8_t id);
int32_t Motor_GetPosRaw(uint8_t id);
int16_t Motor_GetSpeed_qpps(uint8_t id);

void Motor_CommandSpeedRPM (uint8_t id, int32_t target_rpm,  int8_t dir);
void Motor_CommandSpeedQPPS(uint8_t id, int32_t target_qpps, int8_t dir);
void Motor_CommandDuty(uint8_t id, int16_t duty_percent, int8_t dir);

void MotorSyncPID_Enable(uint8_t enable);
void MotorSyncPID_SetGains(int32_t kp_q10, int32_t ki_q10, int32_t kd_q10);
void MotorSyncPID_SetCorrLimitPercent(int16_t limit_percent);
void MotorSyncPID_Reset(void);

void MotorSpeedPID_SetGains(uint8_t id, int32_t kp_q10, int32_t ki_q10, int32_t kd_q10);
void MotorSpeedPID_Reset(uint8_t id);

/* 允许再次回零（命令触发） */
void Motor_HomingArm(void);

#endif /* __MOTOR_HALL_H__ */
