/**
 * motor_hall.c - N32G430 + FreeRTOS
 * 集成：PWM + 霍尔正交 + 霍尔滤波(积分/迟滞+抑制窗) + 速度PID(含前馈) + 同步PID(门限/死区/斜率)
 * 回零：上电后需先收到伸/收命令才触发；固定50%收回；100ms无跳变判底；两侧到底→pos=3000→退出
 * 引脚：
 *  M0: 正极 PA11→TIM1_CH4 ，负极 PA10→TIM1_CH3，HallA PA6，HallB PA5
 *  M1: 正极 PA9 →TIM1_CH2 ，负极 PA8 →TIM1_CH1，HallA PA4，HallB PA3
 *  复用：GPIO_AF3_TIM1
 */
#include "motor_hall.h"
#include "n32g430.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* ==== 同步环抑制阈值（位置差过大暂停，仅恒速；回到较小阈值恢复） ==== */
#ifndef SYNC_POS_DELTA_DISABLE_COUNTS
#define SYNC_POS_DELTA_DISABLE_COUNTS   (200)
#endif
#ifndef SYNC_POS_DELTA_ENABLE_COUNTS
#define SYNC_POS_DELTA_ENABLE_COUNTS    (100)
#endif
static uint8_t g_sync_suspended = 0; /* 1=同步环被抑制，仅恒速 */


#define CLAMP(x, lo, hi)   ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
#define ABS(x)             ((x)>=0?(x):-(x))

typedef enum { CTRL_MODE_DUTY=0, CTRL_MODE_SPEED } ctrl_mode_t;
typedef struct { int16_t duty_percent; int8_t dir; } duty_cmd_t;
typedef struct { int32_t target_qpps; int8_t dir; } speed_cmd_t;
typedef struct { ctrl_mode_t mode; duty_cmd_t duty; speed_cmd_t spd; } motor_cmd_t;

static SemaphoreHandle_t s_cmdMutex;
static volatile motor_cmd_t g_cmd[2];

typedef struct {
    uint8_t a, b, prev;
    int32_t pos;
    int16_t speed_qpps;  /* IIR */
    int8_t  inc_last;    /* -1/0/+1 */
} qei_t;
qei_t qei[2];

typedef struct { int32_t kp,ki,kd; int32_t iacc,last_err; int16_t out_limit; } spid_t;
static spid_t spid[2] = {
    { SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT, 0, 0, SPEED_PID_OUT_LIMIT_PERCENT },
    { SPEED_PID_KP_DEFAULT, SPEED_PID_KI_DEFAULT, SPEED_PID_KD_DEFAULT, 0, 0, SPEED_PID_OUT_LIMIT_PERCENT }
};

typedef struct { int32_t kp,ki,kd; int32_t iacc,last_err; int16_t corr_limit; uint8_t enable; } syncpid_t;
static syncpid_t syncpid = { SYNC_PID_KP_DEFAULT, SYNC_PID_KI_DEFAULT, SYNC_PID_KD_DEFAULT, 0, 0, SYNC_PID_CORR_LIMIT_PERCENT, 1 };

static uint16_t s_pwm_arr = 1000;

static int8_t  s_last_dir[2]     = {0,0};
static uint16_t s_rev_hold_ms[2] = {0,0};
static uint16_t s_sync_freeze_ms = 0;

/* 霍尔滤波/抑制状态 */
static uint16_t s_hall_blank_ms[2] = {0,0}; /* 启动/反转抑制窗口 */
static uint8_t  s_hall_int_a[2] = {0,0}, s_hall_int_b[2] = {0,0};
static uint8_t  s_hall_st_a[2]  = {0,0}, s_hall_st_b[2]  = {0,0};

typedef enum { ST_IDLE=0, ST_KICK, ST_RAMP, ST_CLOSED } start_stage_t;
typedef struct {
    start_stage_t stage;
    uint16_t kick_ms_left, speed_ok_ms, min_on_ms_left;
    int16_t  ramp_duty, last_out_duty;
} start_t;
static start_t sstart[2] = {0};

/* 正常控制时的底位检测与“一次性初始化”门闩 */
static uint16_t run_no_toggle_ms[2] = {0,0};
static uint8_t  run_bottom[2]       = {0,0};
static uint8_t  run_bottom_can_init = 1;   /* 允许本次事件初始化一次 */

/* 上电回零（命令触发） */
static uint8_t  homing_active = 0;
static uint8_t  homing_armed  = HOMING_INIT_ENABLE;
static uint8_t  homing_bottom[2] = {0,0};
static uint16_t homing_no_toggle_ms[2] = {0,0};

/* ★新增：回零驱动时长与是否见到过霍尔边沿 */
static uint16_t homing_run_ms[2]   = {0,0};
static uint8_t  homing_seen_edge[2]= {0,0};

/* 堵转/到位停机状态 */
static uint16_t stall_no_toggle_ms[2] = {0,0};
static uint8_t  stall_latched[2]      = {0,0};
static int8_t   stall_last_dir[2]     = {0,0};

/* 停机判定计时：达到 PID_CLEAR_AFTER_STOP_MS 认为停稳，可清 PID */
static uint16_t pid_stop_ms[2] = {0,0};

/* 运行态判定：最近无跳变计时 + 是否在运行标志 */
static volatile uint16_t qei_no_toggle_ms[2] = {0,0};
static volatile uint8_t  qei_moving[2]       = {0,0};   /* 1=在运行, 0=停止 */


/* 前置声明 */
static void Motor_Set_Low(uint8_t id, int16_t duty_percent, int8_t dir);

/* ================ 硬件初始化 ================ */
static void GPIO_TIM1_Init(void)
{
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_TIM1);

    GPIO_InitType gi;
    GPIO_Structure_Initialize(&gi);
    gi.Pin            = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gi.GPIO_Mode      = GPIO_MODE_AF_PP;
    gi.GPIO_Pull      = GPIO_PULL_UP;
    gi.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    gi.GPIO_Current   = GPIO_DRIVER_8MA;
    gi.GPIO_Alternate = 0;
    GPIO_Peripheral_Initialize(GPIOA, &gi);

    GPIO_Alternate_Set(GPIOA, GPIO_AF3_TIM1, 8);
    GPIO_Alternate_Set(GPIOA, GPIO_AF3_TIM1, 9);
    GPIO_Alternate_Set(GPIOA, GPIO_AF3_TIM1, 10);
    GPIO_Alternate_Set(GPIOA, GPIO_AF3_TIM1, 11);

    GPIO_Structure_Initialize(&gi);
    gi.Pin            = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    gi.GPIO_Mode      = GPIO_MODE_INPUT;
    gi.GPIO_Pull      = GPIO_PULL_UP;
    gi.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    gi.GPIO_Current   = 0;
    gi.GPIO_Alternate = 0;
    GPIO_Peripheral_Initialize(GPIOA, &gi);
}

/* 找到合适的 PSC/ARR 以命中目标频率并尽量保留分辨率 */
static void pick_pwm_params(uint32_t tim_clk, uint32_t target_hz,
                            uint16_t desired_arr, uint16_t* p_psc, uint16_t* p_arr)
{
    uint32_t best_score = 0xFFFFFFFFu;
    uint16_t best_psc=0, best_arr=desired_arr;
    for (uint32_t p=0; p<=255; ++p) {
        uint32_t a1 = tim_clk / (target_hz * (p+1));
        if (a1 == 0) break;
        uint32_t a = a1 - 1;
        if (a < 200 || a > 65535) continue;
        uint32_t real = tim_clk / ((p+1)*(a+1));
        uint32_t err  = (real>target_hz)? (real-target_hz):(target_hz-real);
        uint32_t score = err*2000u + (a>desired_arr? (a-desired_arr):(desired_arr-a));
        if (score < best_score) { best_score=score; best_psc=(uint16_t)p; best_arr=(uint16_t)a; }
    }
    *p_psc = best_psc; *p_arr = best_arr;
    s_pwm_arr = best_arr;
}

static void TIM1_PWM_Init(void)
{
    uint16_t arr=PWM_MAX_DUTY, psc=0;
    pick_pwm_params(TIM1_CLOCK_HZ, PWM_FREQ_HZ, PWM_MAX_DUTY, &psc, &arr);

    TIM_TimeBaseInitType tb;
    TIM_Base_Struct_Initialize(&tb);
    tb.Prescaler = psc;
    tb.CntMode   = TIM_CNT_MODE_UP;
    tb.Period    = arr;
    tb.ClkDiv    = TIM_CLK_DIV1;
    tb.RepetCnt  = 0;
    TIM_Base_Initialize(TIM1, &tb);

    OCInitType oc;
    oc.OcMode      = TIM_OCMODE_PWM1;
    oc.OutputState = TIM_OUTPUT_STATE_ENABLE;
    oc.Pulse       = 0;
    oc.OcPolarity  = TIM_OC_POLARITY_LOW;

    TIM_Output_Channel1_Initialize(TIM1, &oc);
    TIM_Output_Channel1_Preload_Set(TIM1, TIM_OC_PRELOAD_ENABLE);
    TIM_Output_Channel2_Initialize(TIM1, &oc);
    TIM_Output_Channel2_Preload_Set(TIM1, TIM_OC_PRELOAD_ENABLE);
    TIM_Output_Channel3_Initialize(TIM1, &oc);
    TIM_Output_Channel3_Preload_Set(TIM1, TIM_OC_PRELOAD_ENABLE);
    TIM_Output_Channel4_Initialize(TIM1, &oc);
    TIM_Output_Channel4_Preload_Set(TIM1, TIM_OC_PRELOAD_ENABLE);

    TIM_Auto_Reload_Preload_Enable(TIM1);
    TIM_PWM_Output_Enable(TIM1);
    TIM_On(TIM1);
}

void Motor_InitPWMAndHall(void)
{
    GPIO_TIM1_Init();
    TIM1_PWM_Init();
    s_cmdMutex = xSemaphoreCreateMutex();

    for (int i=0;i<2;++i){
        g_cmd[i].mode=CTRL_MODE_DUTY;
        g_cmd[i].duty.duty_percent=0; g_cmd[i].duty.dir=0;
        g_cmd[i].spd.target_qpps=0;   g_cmd[i].spd.dir=0;

        qei[i].a=qei[i].b=qei[i].prev=0;
        qei[i].pos=0; qei[i].speed_qpps=0; qei[i].inc_last=0;

        spid[i].iacc=0; spid[i].last_err=0;
    }
    syncpid.iacc=0; syncpid.last_err=0;

    for (int k=0;k<2;++k){
        sstart[k].stage=ST_IDLE; sstart[k].kick_ms_left=0;
        sstart[k].speed_ok_ms=0; sstart[k].min_on_ms_left=0;
        sstart[k].ramp_duty=0;   sstart[k].last_out_duty=0;
    }

    homing_active=0; homing_armed=HOMING_INIT_ENABLE;
    homing_bottom[0]=homing_bottom[1]=0;
    homing_no_toggle_ms[0]=homing_no_toggle_ms[1]=0;
	run_no_toggle_ms[0]=run_no_toggle_ms[1]=0;
	run_bottom[0]=run_bottom[1]=0;
	run_bottom_can_init = 1;

	stall_no_toggle_ms[0]=stall_no_toggle_ms[1]=0;
	stall_latched[0]=stall_latched[1]=0;
	stall_last_dir[0]=stall_last_dir[1]=0;

	qei_no_toggle_ms[0]=qei_no_toggle_ms[1]=0;
	qei_moving[0]=qei_moving[1]=0;
	pid_stop_ms[0]=pid_stop_ms[1]=0;

}

void Motor_Deinit(void)
{
    TIM_Off(TIM1);
    if (s_cmdMutex) { vSemaphoreDelete(s_cmdMutex); s_cmdMutex=NULL; }
}

/* ================ PWM 输出 ================ */
static void pwm_apply(uint8_t ch, uint16_t cmp)
{
    switch (ch) {
    case 1: TIM_Compare2_Set(TIM1, cmp); break;
    case 2: TIM_Compare1_Set(TIM1, cmp); break;
    case 3: TIM_Compare4_Set(TIM1, cmp); break;
    case 4: TIM_Compare3_Set(TIM1, cmp); break;
    default: break;
    }
}

static void Motor_Set_Low(uint8_t id, int16_t duty_percent, int8_t dir)
{
    duty_percent = CLAMP(duty_percent, 0, 100);
    uint16_t cmp = (uint16_t)((duty_percent * (s_pwm_arr+1)) / 100);

    if (id == 0) {
        if (dir == 2) { pwm_apply(4, cmp); pwm_apply(3, 0); }
        else if (dir == 1) { pwm_apply(3, cmp); pwm_apply(4, 0); }
        else { pwm_apply(3, 0); pwm_apply(4, 0); }
    } else {
        if (dir == 2) { pwm_apply(2, cmp); pwm_apply(1, 0); }
        else if (dir == 1) { pwm_apply(1, cmp); pwm_apply(2, 0); }
        else { pwm_apply(1, 0); pwm_apply(2, 0); }
    }
}

/* ================ 霍尔读取/滤波/计数 ================ */
static inline uint8_t read_hall_ab_raw(uint8_t id)
{
    if (id == 0) {
        uint8_t a = GPIO_Input_Pin_Data_Get(GPIOA, GPIO_PIN_5) ? 1 : 0;
        uint8_t b = GPIO_Input_Pin_Data_Get(GPIOA, GPIO_PIN_6) ? 1 : 0;
        qei[0].a=a; qei[0].b=b; return (uint8_t)((a<<1)|b);
    } else {
        uint8_t a = GPIO_Input_Pin_Data_Get(GPIOA, GPIO_PIN_3) ? 1 : 0;
        uint8_t b = GPIO_Input_Pin_Data_Get(GPIOA, GPIO_PIN_4) ? 1 : 0;
        qei[1].a=a; qei[1].b=b; return (uint8_t)((a<<1)|b);
    }
}

static const int8_t qei_table[16] = {
  0, +1, -1,  0,
 -1,  0,  0, +1,
 +1,  0,  0, -1,
  0, -1, +1,  0
};

static void qei_step_1ms(void)
{
    for (uint8_t i=0;i<2;++i) {
        /* 原始采样 */
        uint8_t curr_raw = read_hall_ab_raw(i);
        uint8_t ra = (curr_raw >> 1) & 1;
        uint8_t rb = curr_raw & 1;

#if HALL_FILTER_ENABLE
        /* 积分 + 迟滞 */
        if (ra) { if (s_hall_int_a[i] < HALL_INT_MAX) ++s_hall_int_a[i]; }
        else    { if (s_hall_int_a[i] > 0)           --s_hall_int_a[i]; }
        if (rb) { if (s_hall_int_b[i] < HALL_INT_MAX) ++s_hall_int_b[i]; }
        else    { if (s_hall_int_b[i] > 0)           --s_hall_int_b[i]; }

        if (s_hall_int_a[i] >= HALL_INT_TH_HI) s_hall_st_a[i] = 1;
        else if (s_hall_int_a[i] <= HALL_INT_TH_LO) s_hall_st_a[i] = 0;
        if (s_hall_int_b[i] >= HALL_INT_TH_HI) s_hall_st_b[i] = 1;
        else if (s_hall_int_b[i] <= HALL_INT_TH_LO) s_hall_st_b[i] = 0;

        uint8_t curr = (uint8_t)((s_hall_st_a[i]<<1) | s_hall_st_b[i]);
#else
        uint8_t curr = curr_raw;
#endif

        /* 启动/反转抑制窗口：固定为上一个稳定状态 */
        if (s_hall_blank_ms[i]) {
            --s_hall_blank_ms[i];
            curr = qei[i].prev;
        }

        uint8_t key  = ((qei[i].prev & 0x3) << 2) | (curr & 0x3);
        int8_t inc   = qei_table[key];

        if (i==0 && HALL_SIGN_M0<0) inc = -inc;
        if (i==1 && HALL_SIGN_M1<0) inc = -inc;

        qei[i].prev  = curr;
        qei[i].pos  += inc;
        qei[i].inc_last = inc;

		/* 更新“是否在运行”判定：有跳变→清零；无跳变→累加 */
		if (inc != 0) qei_no_toggle_ms[i] = 0;
		else if (qei_no_toggle_ms[i] < 0xFFFF) ++qei_no_toggle_ms[i];
		qei_moving[i] = (qei_no_toggle_ms[i] < RUN_MOVING_WINDOW_MS) ? 1 : 0;

        /* IIR 速度估计（qpps） */
        qei[i].speed_qpps += (int16_t)(((int32_t)(inc*1000) - qei[i].speed_qpps) * SPEED_IIR_ALPHA_NUM / SPEED_IIR_ALPHA_DEN);
        if (qei[i].speed_qpps < SPEED_QPPS_DEADBAND && qei[i].speed_qpps > -SPEED_QPPS_DEADBAND) qei[i].speed_qpps = 0;
    }
}

int32_t Motor_GetPosRaw(uint8_t id){ return qei[id].pos; }
int32_t Motor_GetTurns(uint8_t id){ return qei[id].pos / HALL_CPR; }
int16_t Motor_GetSpeed_qpps(uint8_t id){ return qei[id].speed_qpps; }

/* ================ 速度 PID（含前馈） ================ */
static int16_t speed_pid_step(uint8_t id, int32_t target_qpps_signed, int32_t meas_qpps_signed)
{
    int32_t err = target_qpps_signed - meas_qpps_signed;

    int32_t p = spid[id].kp * err;
    int32_t d = spid[id].kd * (err - spid[id].last_err);
    int32_t iacc_next = spid[id].iacc + spid[id].ki * err;

    int32_t lim = (int32_t)SPEED_PID_I_ACC_LIMIT << SPEED_PID_K_SHIFT;
    if (iacc_next >  lim) iacc_next =  lim;
    if (iacc_next < -lim) iacc_next = -lim;

    int32_t u_q10 = p + iacc_next + d;
    int32_t u = (u_q10 >> SPEED_PID_K_SHIFT); /* % */

#if SPEED_FF_ENABLE
    {
        int32_t u_ff = ( (int32_t)ABS(target_qpps_signed) * SPEED_FF_K_NUM ) / SPEED_FF_K_DEN;
        if (u_ff < SPEED_FF_MIN_PERCENT) u_ff = SPEED_FF_MIN_PERCENT;
        u += u_ff;
    }
#endif

    int sat = 0;
    if (u > spid[id].out_limit) { u = spid[id].out_limit; sat = 1; }
    if (u < -spid[id].out_limit){ u = -spid[id].out_limit; sat = 1; }
    if (!sat) spid[id].iacc = iacc_next;

    spid[id].last_err = err;

    return (int16_t)u;
}

/* ================ duty 斜坡限制 ================ */
static inline int16_t duty_slew_limit(int16_t prev, int16_t target)
{
    int16_t up = DUTY_SLEW_UP_PCT_PER_MS;
    int16_t dn = DUTY_SLEW_DOWN_PCT_PER_MS;
    if (target > prev + up)   return prev + up;
    if (target < prev - dn)   return prev - dn;
    return target;
}
static inline void SpeedPID_Clear(uint8_t id)
{
    if (id < 2) { spid[id].iacc = 0; spid[id].last_err = 0; }
}

static inline void SyncPID_Clear(void)
{
    syncpid.iacc = 0; syncpid.last_err = 0;
}

/* 占空量化：按整数百分比步进，抑制 ±1% 抖动 */
static inline int16_t duty_quantize(int16_t pct)
{
    if (pct <= 0) return 0;
    int16_t step = (DUTY_QUANT_PERCENT <= 0) ? 1 : DUTY_QUANT_PERCENT;
    return (pct / step) * step;
}

/* 最终占空的额外斜率限制（与启动阶段的斜率分离，专门给同步纠偏用） */
static inline int16_t duty_final_slew(int16_t prev, int16_t target)
{
    int16_t s = FINAL_DUTY_SLEW_PCT_PER_MS;
    if (s <= 0) return target;
    if (target > prev + s) return prev + s;
    if (target < prev - s) return prev - s;
    return target;
}


/* ================ 同步 PID ================ */
/* ================ 同步 PID（增强版：强制追赶 + 纠偏下限） ================ */
/* ================ 同步 PID（增强+止抖：强制追赶 + 纠偏下限 + 静止冻结 + 输出去抖） ================ */
static void sync_pid_apply_and_drive(int16_t base0, int8_t dir0, int16_t base1, int8_t dir1)
{
/* 速度未贴近目标 → 先跳过同步，避免与速度环打架（保留你现有门控） */
#if SYNC_GATE_ENABLE
    {
        int32_t tgt0 = 0, tgt1 = 0;
        if (g_cmd[0].mode == CTRL_MODE_SPEED)
            tgt0 = (g_cmd[0].spd.dir==2) ? -(int32_t)g_cmd[0].spd.target_qpps : (int32_t)g_cmd[0].spd.target_qpps;
        if (g_cmd[1].mode == CTRL_MODE_SPEED)
            tgt1 = (g_cmd[1].spd.dir==2) ? -(int32_t)g_cmd[1].spd.target_qpps : (int32_t)g_cmd[1].spd.target_qpps;

        int32_t e0 = qei[0].speed_qpps - tgt0;
        int32_t e1 = qei[1].speed_qpps - tgt1;
/* —— 单电机优先通道（早于速度门限早退）——
   一侧有效（dir!=0 且 未锁存 或 正在反向离开锁存方向），另一侧无效，则直接驱动有效侧并返回。 */
{
    uint8_t active0 = (dir0 != 0) && (!stall_latched[0] || (dir0 != stall_last_dir[0]));
    uint8_t active1 = (dir1 != 0) && (!stall_latched[1] || (dir1 != stall_last_dir[1]));
    if (active0 && !active1) {
        Motor_Set_Low(0, ABS(base0), dir0);
        Motor_Set_Low(1, 0, 0);
        syncpid.iacc = 0; syncpid.last_err = 0;
        return;
    }
    if (!active0 && active1) {
        Motor_Set_Low(0, 0, 0);
        Motor_Set_Low(1, ABS(base1), dir1);
        syncpid.iacc = 0; syncpid.last_err = 0;
        return;
    }
}
        if (ABS(e0) > SPEED_SYNC_GATE_QPPS || ABS(e1) > SPEED_SYNC_GATE_QPPS) {
            Motor_Set_Low(0, ABS(base0), dir0);
            Motor_Set_Low(1, ABS(base1), dir1);
            syncpid.iacc = 0; syncpid.last_err = 0;
            return;
        }
    }
#endif

    /* —— 同步环抑制门（必须在早退判断之前运行） —— */
{
    int32_t _pdiff = qei[0].pos - qei[1].pos;
    uint32_t _adiff = (_pdiff >= 0) ? (uint32_t)_pdiff : (uint32_t)(-_pdiff);
    uint8_t _prev = g_sync_suspended;
    if (!g_sync_suspended) {
        if (_adiff >= (uint32_t)SYNC_POS_DELTA_DISABLE_COUNTS) g_sync_suspended = 1;
    } else {
        if (_adiff <= (uint32_t)SYNC_POS_DELTA_ENABLE_COUNTS)  g_sync_suspended = 0;
    }
    if (g_sync_suspended != _prev) { SyncPID_Clear(); }
}
/* 同步关闭、任一侧停转、或方向不一致：直接按基准驱动 */
    if (!syncpid.enable || g_sync_suspended || dir0==0 || dir1==0 || dir0!=dir1) {
        Motor_Set_Low(0, ABS(base0), dir0);
        Motor_Set_Low(1, ABS(base1), dir1);
        syncpid.iacc=0; syncpid.last_err=0;
        return;
    }

    /* 领先方向：以速度和为主，近零时退回命令方向 */
    int sum_v = qei[0].speed_qpps + qei[1].speed_qpps;
    int sign_dir = ( (sum_v > SPEED_QPPS_DEADBAND) ? +1 :
                     (sum_v < -SPEED_QPPS_DEADBAND) ? -1 :
                     ((dir0==1)? +1 : -1) );

    int32_t err = sign_dir * (qei[0].pos - qei[1].pos); /* >0 => M0 领先 */
    /* 小误差死区（冻结I） */
    if (err > -SYNC_ERR_DEADBAND_COUNTS && err < SYNC_ERR_DEADBAND_COUNTS) {
        err = 0;
    }

    /* —— 强制追赶模式：误差过大时，直接限流领先侧、加油落后侧 —— */
    {
        int32_t abs_err = (err >= 0) ? err : -err;
        if (abs_err >= SYNC_ERR_CATCH_COUNTS) {
            int lead = (err > 0) ? 0 : 1;        /* err>0: M0 领先；否则 M1 领先 */
            int32_t d0 = base0, d1 = base1;

            if (lead == 0) { d0 = (d0 > SYNC_CATCH_HOLD_DUTY) ? SYNC_CATCH_HOLD_DUTY : d0; d1 = d1 + SYNC_CATCH_BOOST_DUTY; }
            else           { d1 = (d1 > SYNC_CATCH_HOLD_DUTY) ? SYNC_CATCH_HOLD_DUTY : d1; d0 = d0 + SYNC_CATCH_BOOST_DUTY; }

            /* 饱和 + 最小占空保障 */
            if (d0 < 0) d0 = 0; if (d0 > 100) d0 = 100;
            if (d1 < 0) d1 = 0; if (d1 > 100) d1 = 100;

            /* 输出去抖：量化 + 末端斜率限制 */
            static int16_t d0_prev=0, d1_prev=0;
            d0 = duty_final_slew(d0_prev, (int16_t)d0);
            d1 = duty_final_slew(d1_prev, (int16_t)d1);
            d0 = duty_quantize((int16_t)d0);
            d1 = duty_quantize((int16_t)d1);
            d0_prev = (int16_t)d0; d1_prev = (int16_t)d1;

            if (dir0 && d0>0 && d0<SYNC_MIN_DUTY_PERCENT) d0 = SYNC_MIN_DUTY_PERCENT;
            if (dir1 && d1>0 && d1<SYNC_MIN_DUTY_PERCENT) d1 = SYNC_MIN_DUTY_PERCENT;

            Motor_Set_Low(0, (int16_t)d0, dir0);
            Motor_Set_Low(1, (int16_t)d1, dir1);
            /* 强制追赶：不同步积分，避免残留偏置 */
            return;
        }
    }

    /* —— 正常同步 PID —— */
    int32_t derr = err - syncpid.last_err;       /* 用于“静止判定” */
    int32_t p = syncpid.kp * err;
    int32_t d = syncpid.kd * derr;
    int32_t iacc_next = syncpid.iacc + syncpid.ki * err;
    int32_t i_lim = (int32_t)SYNC_PID_I_ACC_LIMIT << SYNC_PID_K_SHIFT;
    if (iacc_next >  i_lim) iacc_next =  i_lim;
    if (iacc_next < -i_lim) iacc_next = -i_lim;

    int32_t corr = (p + iacc_next + d) >> SYNC_PID_K_SHIFT;

    /* 动态限幅（增强版）：再加绝对下限，避免低占空时纠偏太小 */
    int32_t dyn = ((int32_t)((base0<base1)?base0:base1) * SYNC_CORR_FRAC_NUM) / SYNC_CORR_FRAC_DEN;
    if (dyn < SYNC_CORR_MIN_LIMIT_PERCENT) dyn = SYNC_CORR_MIN_LIMIT_PERCENT;
    int32_t limit = syncpid.corr_limit;
    if (dyn < limit) limit = dyn;
    if (corr >  limit) corr =  limit;
    if (corr < -limit) corr = -limit;

    /* 纠偏斜率限制（你已有，保留） */
    static int16_t corr_prev = 0;
    int16_t step = SYNC_CORR_SLEW_PCT_PER_MS;
    if (corr > corr_prev + step)       corr = corr_prev + step;
    else if (corr < corr_prev - step)  corr = corr_prev - step;
    corr_prev = (int16_t)corr;

    /* —— 微扰“静止”判定：小误差 + 小变化 持续一段时间 → 冻结纠偏，避免抖动 —— */
    {
        static uint16_t still_ms = 0;
        int32_t aerr  = (err  >= 0) ? err  : -err;
        int32_t aderr = (derr >= 0) ? derr : -derr;
        if (aerr <= SYNC_MICRO_ERR_COUNTS && aderr <= SYNC_MICRO_DERR_COUNTS) {
            if (still_ms < 0xFFFF) ++still_ms;
        } else {
            still_ms = 0;
        }
        if (still_ms >= SYNC_STILL_HOLD_MS) {
            corr = 0;            /* 暂时不纠偏 */
            iacc_next = 0;       /* 冻结 I，避免反复累积 */
        }
    }

    int32_t duty0 = base0 - corr;
    int32_t duty1 = base1 + corr;

    /* 饱和与 I 抗饱和 */
    int sat = 0;
    if (duty0<0){duty0=0;sat=1;} if (duty0>100){duty0=100;sat=1;}
    if (duty1<0){duty1=0;sat=1;} if (duty1>100){duty1=100;sat=1;}
    if (!sat) syncpid.iacc = iacc_next;
    syncpid.last_err = err;

    /* 输出去抖：量化 + 末端斜率限制 */
    static int16_t duty0_prev=0, duty1_prev=0;
    duty0 = duty_final_slew(duty0_prev, (int16_t)duty0);
    duty1 = duty_final_slew(duty1_prev, (int16_t)duty1);
    duty0 = duty_quantize((int16_t)duty0);
    duty1 = duty_quantize((int16_t)duty1);
    duty0_prev = (int16_t)duty0;
    duty1_prev = (int16_t)duty1;

    if (dir0 && duty0>0 && duty0<SYNC_MIN_DUTY_PERCENT) duty0=SYNC_MIN_DUTY_PERCENT;
    if (dir1 && duty1>0 && duty1<SYNC_MIN_DUTY_PERCENT) duty1=SYNC_MIN_DUTY_PERCENT;

    Motor_Set_Low(0, (int16_t)duty0, dir0);
    Motor_Set_Low(1, (int16_t)duty1, dir1);
}



/* ================ 线程安全接口 ================ */
void Motor_CommandSpeedRPM (uint8_t id, int32_t target_rpm,  int8_t dir)
{
    if (!s_cmdMutex || id > 1) return;

    /* 若该电机当前在运行：此次调用视为“停机指令” */
    if (qei_moving[id]) {
        xSemaphoreTake(s_cmdMutex, portMAX_DELAY);
        g_cmd[id].mode = CTRL_MODE_SPEED;
        g_cmd[id].spd.target_qpps = 0;
        g_cmd[id].spd.dir = 0;
        g_cmd[id].duty.duty_percent = 0;
        g_cmd[id].duty.dir = 0;
        xSemaphoreGive(s_cmdMutex);
		SpeedPID_Clear(id);      /* ★可选：接口层立即清本侧速度环 */
        return;
    }

    /* 只有电机处于停止状态时，才按正常逻辑接收并运行 */
    if (target_rpm == 0 || dir == 0) {
        xSemaphoreTake(s_cmdMutex, portMAX_DELAY);
        g_cmd[id].mode=CTRL_MODE_SPEED; g_cmd[id].spd.target_qpps=0; g_cmd[id].spd.dir=0;
        xSemaphoreGive(s_cmdMutex); return;
    }
    int32_t qpps = RPM_TO_QPPS( ABS(target_rpm) );
    xSemaphoreTake(s_cmdMutex, portMAX_DELAY);
    g_cmd[id].mode=CTRL_MODE_SPEED;
    g_cmd[id].spd.target_qpps=qpps;
    g_cmd[id].spd.dir=(dir==2)?2:1;
    xSemaphoreGive(s_cmdMutex);
}


void Motor_CommandSpeedQPPS(uint8_t id, int32_t target_qpps, int8_t dir)
{
    if (!s_cmdMutex || id > 1) return;

    if (qei_moving[id]) {
        xSemaphoreTake(s_cmdMutex, portMAX_DELAY);
        g_cmd[id].mode = CTRL_MODE_SPEED;
        g_cmd[id].spd.target_qpps = 0;
        g_cmd[id].spd.dir = 0;
        g_cmd[id].duty.duty_percent = 0;
        g_cmd[id].duty.dir = 0;
        xSemaphoreGive(s_cmdMutex);
		SpeedPID_Clear(id);      /* ★可选：接口层立即清本侧速度环 */
        return;
    }

    if (target_qpps==0 || dir==0) {
        xSemaphoreTake(s_cmdMutex, portMAX_DELAY);
        g_cmd[id].mode=CTRL_MODE_SPEED; g_cmd[id].spd.target_qpps=0; g_cmd[id].spd.dir=0;
        xSemaphoreGive(s_cmdMutex); return;
    }
    xSemaphoreTake(s_cmdMutex, portMAX_DELAY);
    g_cmd[id].mode=CTRL_MODE_SPEED;
    g_cmd[id].spd.target_qpps=ABS(target_qpps);
    g_cmd[id].spd.dir=(dir==2)?2:1;
    xSemaphoreGive(s_cmdMutex);
}


/* 兼容：开环占空 */
void Motor_CommandDuty(uint8_t id, int16_t duty_percent, int8_t dir)
{
    if (!s_cmdMutex || id > 1) return;

    if (qei_moving[id]) {
        xSemaphoreTake(s_cmdMutex, portMAX_DELAY);
        g_cmd[id].mode = CTRL_MODE_SPEED;           /* 统一用 SPEED 模式的“停机表达” */
        g_cmd[id].spd.target_qpps = 0;
        g_cmd[id].spd.dir = 0;
        g_cmd[id].duty.duty_percent = 0;
        g_cmd[id].duty.dir = 0;
        xSemaphoreGive(s_cmdMutex);
		SpeedPID_Clear(id);      /* ★可选：接口层立即清本侧速度环 */
        return;
    }

    xSemaphoreTake(s_cmdMutex, portMAX_DELAY);
    g_cmd[id].mode=CTRL_MODE_DUTY;
    g_cmd[id].duty.duty_percent = CLAMP((int16_t)ABS(duty_percent),0,100);
    g_cmd[id].duty.dir          = (dir==2)?2:(dir==1?1:0);
    xSemaphoreGive(s_cmdMutex);
}


/* 允许再次回零（命令触发） */
void Motor_HomingArm(void)
{
#if HOMING_INIT_ENABLE
    taskENTER_CRITICAL();
    homing_armed=1; homing_active=0;
    homing_bottom[0]=homing_bottom[1]=0;
    homing_no_toggle_ms[0]=homing_no_toggle_ms[1]=0;
    spid[0].iacc=spid[1].iacc=0; spid[0].last_err=spid[1].last_err=0;
    syncpid.iacc=0; syncpid.last_err=0;
    taskEXIT_CRITICAL();
#endif
}

/* ================ 1ms 任务 ================ */
static void MotorTask(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);

    for(;;){
        qei_step_1ms();

        if (s_rev_hold_ms[0]) --s_rev_hold_ms[0];
        if (s_rev_hold_ms[1]) --s_rev_hold_ms[1];
        if (s_sync_freeze_ms) --s_sync_freeze_ms;

        int16_t out0=0,out1=0; int8_t dir0=0,dir1=0;

        if (s_cmdMutex && xSemaphoreTake(s_cmdMutex,0)==pdTRUE){
            /* 方向切换检测：反向→空转&同步冻结&积分清零；并打霍尔抑制窗 */
            for (int i=0;i<2;++i){
                int8_t newdir = (i==0)
                    ? (g_cmd[0].mode==CTRL_MODE_SPEED? g_cmd[0].spd.dir: g_cmd[0].duty.dir)
                    : (g_cmd[1].mode==CTRL_MODE_SPEED? g_cmd[1].spd.dir: g_cmd[1].duty.dir);
                if (newdir!=0 && s_last_dir[i]!=0 && newdir!=s_last_dir[i]){
                    s_rev_hold_ms[i] = REVERSE_HOLD_MS;
                    s_sync_freeze_ms = SYNC_FREEZE_MS;
                    s_hall_blank_ms[i] = HALL_BLANK_ON_REV_MS;   /* 反转抑制窗口 */
                    spid[i].iacc=0; spid[i].last_err=0;
                    syncpid.iacc=0; syncpid.last_err=0;
                }
                if (newdir != s_last_dir[i]) {
                    if (s_last_dir[i]==0 && newdir!=0) s_hall_blank_ms[i] = HALL_BLANK_ON_START_MS; /* 启动抑制窗口 */
                    s_last_dir[i] = newdir;
                }
            }

            /* 计算基准占空（速度环） */
            if (g_cmd[0].mode==CTRL_MODE_SPEED){
                dir0=g_cmd[0].spd.dir;
                int32_t tgt=(dir0==2)?-(int32_t)g_cmd[0].spd.target_qpps:(int32_t)g_cmd[0].spd.target_qpps;
                int16_t u=speed_pid_step(0,tgt,qei[0].speed_qpps);
                out0=ABS(u);
            } else {
                dir0=g_cmd[0].duty.dir; out0=g_cmd[0].duty.duty_percent;
                spid[0].iacc=0; spid[0].last_err=0;
            }

            if (g_cmd[1].mode==CTRL_MODE_SPEED){
                dir1=g_cmd[1].spd.dir;
                int32_t tgt=(dir1==2)?-(int32_t)g_cmd[1].spd.target_qpps:(int32_t)g_cmd[1].spd.target_qpps;
                int16_t u=speed_pid_step(1,tgt,qei[1].speed_qpps);
                out1=ABS(u);
            } else {
                dir1=g_cmd[1].duty.dir; out1=g_cmd[1].duty.duty_percent;
                spid[1].iacc=0; spid[1].last_err=0;
            }

            xSemaphoreGive(s_cmdMutex);
        }

/* —— 回零触发：上电后需先收到伸/收命令 —— */
#if HOMING_INIT_ENABLE && HOMING_TRIGGER_ON_CMD
		if (!homing_active && homing_armed) {
			if (dir0 != 0 || dir1 != 0) {
				homing_active = 1;
				homing_bottom[0] = homing_bottom[1] = 0;
				homing_no_toggle_ms[0] = homing_no_toggle_ms[1] = 0;
			}
		}
#endif


        /* 回零流程 */
/* ========== 回零流程（首个运行统一回零；STOP则停；两侧到底后恢复正常） ========== */
#if HOMING_INIT_ENABLE
		if (homing_active) {
			/* “停止命令则停止运行”：只有当两个电机都下达停止（dir==0）才视为STOP */
			uint8_t stop_req = ((dir0 == 0) && (dir1 == 0)) ? 1 : 0;

			/* 底位判定：基于“连续 HOMING_NO_TOGGLE_MS 无跳变”认为碰到底位开关 */
			for (int mi = 0; mi < 2; ++mi) {
				if (stop_req || homing_bottom[mi]) {
					homing_no_toggle_ms[mi] = 0;
				} else {
					int8_t inc = (mi == 0) ? qei[0].inc_last : qei[1].inc_last;
					if (inc != 0) homing_no_toggle_ms[mi] = 0;
					else if (homing_no_toggle_ms[mi] < 0xFFFF) ++homing_no_toggle_ms[mi];
					if (homing_no_toggle_ms[mi] >= HOMING_NO_TOGGLE_MS) {
						homing_bottom[mi] = 1; /* 认为已到底位（行程开关触发） */
					}
				}
			}

			/* 输出策略：
			   - STOP：两侧都停；
			   - 非STOP：未到底侧用固定占空沿“收回方向”转，已到底侧停。*/
			if (stop_req) {
				Motor_Set_Low(0, 0, 0);
				Motor_Set_Low(1, 0, 0);
			} else {
				int16_t duty = HOMING_PWM_PERCENT;
				if (!homing_bottom[0])
				{
					Motor_Set_Low(0, duty, HOMING_RETRACT_DIR_M0);
				}else 
				{
					Motor_Set_Low(0, 0, 0);
				}
				if (!homing_bottom[1])
				{
					Motor_Set_Low(1, duty, HOMING_RETRACT_DIR_M1);
				}else
				{
					Motor_Set_Low(1, 0, 0);
				}
			}

			/* 两侧都到底：设置基准位置→退出回零→恢复正常运行 */
			if (homing_bottom[0] && homing_bottom[1]) {
				qei[0].pos = HOMING_INIT_POS;
				qei[1].pos = HOMING_INIT_POS;
				qei[0].speed_qpps = 0;
				qei[1].speed_qpps = 0;

				homing_active = 0;   /* 退出回零，恢复正常 */
				homing_armed  = 0;   /* 本次上电回零只触发一次 */

				s_sync_freeze_ms = SYNC_FREEZE_MS; /* 给同步/速度环一点缓冲 */
				/* 清积分，避免切换瞬间的残留偏置 */
				spid[0].iacc = spid[1].iacc = 0;
				spid[0].last_err = spid[1].last_err = 0;
				syncpid.iacc = 0;
				syncpid.last_err = 0;
				#if HOMING_STOP_AFTER_INIT
/* ← 回零完成后立即“停机并清指令”，避免继续执行上一次按键的运行方向 */
				taskENTER_CRITICAL();
				g_cmd[0].mode = CTRL_MODE_SPEED;
				g_cmd[0].spd.target_qpps = 0;
				g_cmd[0].spd.dir = 0;
				g_cmd[0].duty.duty_percent = 0;
				g_cmd[0].duty.dir = 0;

				g_cmd[1].mode = CTRL_MODE_SPEED;
				g_cmd[1].spd.target_qpps = 0;
				g_cmd[1].spd.dir = 0;
				g_cmd[1].duty.duty_percent = 0;
				g_cmd[1].duty.dir = 0;
				taskEXIT_CRITICAL();

				s_last_dir[0] = 0;
				s_last_dir[1] = 0;

				/* 保险起见把两侧直接拉停一次 */
				Motor_Set_Low(0, 0, 0);
				Motor_Set_Low(1, 0, 0);
#endif
			}

			vTaskDelayUntil(&last, period);
			continue; /* 回零期间不走下面的同步PID/速度闭环 */
		}
#endif
/* ★ 堵转/到位停机：有指令且无霍尔跳变达到超时→停机（两侧同时停） */
#if STALL_DET_ENABLE
        if (!homing_active) {
            for (int mi=0; mi<2; ++mi) {
                int8_t  d   = (mi==0) ? dir0 : dir1;          /* 当前命令方向 */
                int16_t cmd = (mi==0) ? out0 : out1;          /* 当前命令占空 */
                int8_t  inc = (mi==0) ? qei[0].inc_last : qei[1].inc_last;

#if STALL_RELEASE_ON_DIR_CHANGE
                if (d != 0 && d != stall_last_dir[mi]) {      /* 换向→解锁&清计时 */
                    stall_latched[mi] = 0;
                    stall_no_toggle_ms[mi] = 0;
                }
#endif
                stall_last_dir[mi] = d;

                if (d == 0 || cmd < STALL_MIN_DUTY_PERCENT) { /* 无指令/占空太小→清计时 */
                    stall_no_toggle_ms[mi] = 0;
                } else {
                    if (inc == 0) {
                        if (stall_no_toggle_ms[mi] < 0xFFFF) ++stall_no_toggle_ms[mi];
                    } else {
                        stall_no_toggle_ms[mi] = 0;
                    }
                    if (stall_no_toggle_ms[mi] >= STALL_NO_TOGGLE_MS) {
                        stall_latched[mi] = 1;
                    }
                }
            }

/* —— 到位/堵转处理（取消联停）：各停各的；若指令反向则释放锁存 —— */
if (stall_latched[0]) {
    if (dir0 != 0 && dir0 != stall_last_dir[0]) {
        stall_latched[0] = 0; stall_no_toggle_ms[0] = 0;
    } else {
        out0 = 0; dir0 = 0;
    }
}
if (stall_latched[1]) {
    if (dir1 != 0 && dir1 != stall_last_dir[1]) {
        stall_latched[1] = 0; stall_no_toggle_ms[1] = 0;
    } else {
        out1 = 0; dir1 = 0;
    }
}
        }
#endif


        /* 软启动/斜坡 */
#if SOFT_START_ENABLE
        for (int mi=0; mi<2; ++mi) {
            int8_t  d   = (mi==0)? dir0 : dir1;
            int16_t cmd = (mi==0)? out0 : out1;
            int16_t v   = (mi==0)? qei[0].speed_qpps : qei[1].speed_qpps;
            start_t* st = &sstart[mi];

            if (d==0) { st->stage=ST_IDLE; st->kick_ms_left=0; st->speed_ok_ms=0; st->min_on_ms_left=0; st->ramp_duty=0; }
            else {
                if (st->stage==ST_IDLE) {
                    st->stage=ST_KICK; st->kick_ms_left=START_KICK_MS;
                    st->ramp_duty=START_KICK_DUTY_PERCENT; st->speed_ok_ms=0;
                    st->min_on_ms_left=DUTY_MIN_ON_TIME_MS;
                }
                if (st->stage==ST_KICK) { if (st->kick_ms_left) --st->kick_ms_left; if (st->kick_ms_left==0) st->stage=ST_RAMP; }
                if (st->stage==ST_RAMP) {
                    int16_t target=(cmd>st->ramp_duty)?cmd:st->ramp_duty;
                    st->ramp_duty = duty_slew_limit(st->ramp_duty, target);
                    int16_t av=(v>=0)?v:-v;
                    if (av>=START_SPEED_OK_QPPS) {
                        if (st->speed_ok_ms<0xFFFF) ++st->speed_ok_ms;
                        if (st->speed_ok_ms>=START_SPEED_OK_MS) st->stage=ST_CLOSED;
                    } else st->speed_ok_ms=0;
                }
                if (st->stage==ST_CLOSED) {
                    st->ramp_duty = duty_slew_limit(st->ramp_duty, cmd);
                }
            }
            if (mi==0) out0=st->ramp_duty; else out1=st->ramp_duty;

            if (d!=0 && st->min_on_ms_left){
                if (mi==0 && out0>0 && out0<DUTY_MIN_ON_PERCENT) out0=DUTY_MIN_ON_PERCENT;
                if (mi==1 && out1>0 && out1<DUTY_MIN_ON_PERCENT) out1=DUTY_MIN_ON_PERCENT;
                --st->min_on_ms_left;
            }
        }
#endif

        /* 反转保持：强制停 */
        if (s_rev_hold_ms[0]){ out0=0; dir0=0; }
        if (s_rev_hold_ms[1]){ out1=0; dir1=0; }

        if (s_sync_freeze_ms) {
            Motor_Set_Low(0,out0,dir0);
            Motor_Set_Low(1,out1,dir1);
        } else {
            sync_pid_apply_and_drive(out0,dir0,out1,dir1);
        }
		#if RUN_BOTTOM_INIT_ENABLE
        if (!homing_active) {
            /* 仅在“按收回方向”运动时，基于霍尔“无跳变”来判底 */
            for (int mi=0; mi<2; ++mi) {
                int8_t d   = (mi==0) ? dir0 : dir1;                /* 本拍命令方向 */
                int8_t inc = (mi==0) ? qei[0].inc_last : qei[1].inc_last;
                uint8_t retract_dir = (mi==0) ? HOMING_RETRACT_DIR_M0 : HOMING_RETRACT_DIR_M1;

                if (d == retract_dir) {
                    if (inc != 0) run_no_toggle_ms[mi] = 0;
                    else if (run_no_toggle_ms[mi] < 0xFFFF) ++run_no_toggle_ms[mi];
                    run_bottom[mi] = (run_no_toggle_ms[mi] >= RUN_BOTTOM_NO_TOGGLE_MS);
                } else {
                    /* 非收回方向不参与底位判定 */
                    run_no_toggle_ms[mi] = 0;
                    run_bottom[mi] = 0;
                }
            }

            /* 两侧同时到底：只初始化一次（离开底位后再“上膛”允许下一次） */
            if (run_bottom_can_init && run_bottom[0] && run_bottom[1]) {
                qei[0].pos = HOMING_INIT_POS;
                qei[1].pos = HOMING_INIT_POS;
                /* 速度清零更稳妥，避免瞬间偏差 */
                qei[0].speed_qpps = 0;
                qei[1].speed_qpps = 0;
                run_bottom_can_init = 0;   /* 本次事件已初始化 */
            }
            if (!(run_bottom[0] && run_bottom[1])) {
                run_bottom_can_init = 1;   /* 任一侧离开底位后，允许下一次初始化 */
            }
        }
#endif

#if PID_CLEAR_ON_STOP_ENABLE
        /* 判定停稳：该侧命令停（dir=0 或 out=0） 且 霍尔无跳变达到阈值 */
        for (int mi=0; mi<2; ++mi) {
            int8_t  d   = (mi==0)? dir0 : dir1;
            int16_t o   = (mi==0)? out0 : out1;
            uint8_t mv  = qei_moving[mi];       /* 你在 qei_step_1ms() 中已维护 */

            if (d == 0 || o == 0) {
                if (!mv) {
                    if (pid_stop_ms[mi] < 0xFFFF) ++pid_stop_ms[mi];
                    if (pid_stop_ms[mi] == PID_CLEAR_AFTER_STOP_MS) {
                        SpeedPID_Clear(mi);     /* 本侧停稳：清本侧速度环 */
                    }
                } else {
                    pid_stop_ms[mi] = 0;        /* 还在动，计时重置 */
                }
            } else {
                pid_stop_ms[mi] = 0;            /* 有运行指令，计时清零 */
            }
        }

        /* 两侧都已停稳：再清一次同步环更稳妥 */
        if (pid_stop_ms[0] >= PID_CLEAR_AFTER_STOP_MS &&
            pid_stop_ms[1] >= PID_CLEAR_AFTER_STOP_MS) {
            SyncPID_Clear();
        }
#endif
        vTaskDelayUntil(&last,period);
    }
}

void MotorRTOS_Init(void)
{
    Motor_InitPWMAndHall();
    xTaskCreate(MotorTask, "motor1ms", 768, NULL, tskIDLE_PRIORITY+2, NULL);
}

/* ================ 配置接口 ================ */
void MotorSyncPID_Enable(uint8_t enable){ syncpid.enable=(enable?1:0); }
void MotorSyncPID_SetGains(int32_t kp_q10, int32_t ki_q10, int32_t kd_q10){ syncpid.kp=kp_q10; syncpid.ki=ki_q10; syncpid.kd=kd_q10; }
void MotorSyncPID_SetCorrLimitPercent(int16_t limit_percent){ if(limit_percent<0)limit_percent=0; if(limit_percent>100)limit_percent=100; syncpid.corr_limit=limit_percent; }
void MotorSyncPID_Reset(void){ syncpid.iacc=0; syncpid.last_err=0; }

void MotorSpeedPID_SetGains(uint8_t id, int32_t kp_q10, int32_t ki_q10, int32_t kd_q10){ if(id<2){ spid[id].kp=kp_q10; spid[id].ki=ki_q10; spid[id].kd=kd_q10; } }
void MotorSpeedPID_Reset(uint8_t id){ if(id<2){ spid[id].iacc=0; spid[id].last_err=0; } }
