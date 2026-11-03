/**
  ******************************************************************************
  * @file    Sample.h
  * @brief   ADC 采样 + 三种功率估计（头文件）
  ******************************************************************************
  * 信号链：
  *   PA1 (0~3.3 V) → ADC1_IN1 12 位 → DMA 乒乓缓冲 → 每点算瞬时功率 x²
  *     → 同时维护三种估计：累积能量 / 滑动窗口平均 / 指数平均
  *
  * 归一化方式（很重要，关系到所有数值的单位）：
  *   x = (码值 - 2048) / 2048        码值 2048 = 1.65 V 当成"零"
  *   所以 x 的范围是 -1.0 ~ +1.0，对应输入 -1.65 V ~ +1.65 V
  *   想换回实际电压：Vrms = rms_win × 1.65
  *
  * 三种估计器的区别：
  *   累积能量   energy  = Σx²              从开机一直累加，单调增长
  *   滑动窗口   p_win   = mean(最近 WIN_M 点 x²)   窗口 21.33 ms @48 kHz，平滑但滞后
  *   指数平均   p_exp   = α·p_exp + (1-α)·x²      τ = 2.08 ms @48 kHz，跟得快但抖
  *
  * 硬件连接（STM32F407G-DISC1）：
  *   PA1 —— 模拟输入（必须围绕 1.65 V 摆动，否则算出来是直流偏移的功率）
  *   PA4 —— DAC1_OUT1，输出正比于 RMS 的直流电压（= 2 × 输入 Vrms）
  *   PA2 —— USART2_TX，接板载 ST-LINK 虚拟串口或外接 CH340，给 PC 发 CSV
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SAMPLE_H
#define __SAMPLE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* ===== 采样参数（改这里就能换采样率 / 换估计器响应速度） =================== */

/* 采样率，单位: Hz。改这一个数 TIM2 的 ARR 就自动跟着算。
   48 kHz 是音频标准，也是原工程的取值。
   常用值：8000 / 16000 / 32000 / 44100 / 48000
   注意 84 MHz 要能被它整除才精确（48000 → 84e6/48000 = 1750，正好整除）。
   [E] 84 MHz = APB1(42 MHz) × 2，来自本工程 PLL: HSE 8 MHz / M8 × N336 / P2 = 168 MHz。 */
#define SAMPLE_FS_HZ        48000U

/* TIM2 自动重装值：采样率 = 84 MHz / (ARR + 1)
   默认 84e6/48000 - 1 = 1749，反汇编里是 MOV r0,#0x6d5 已核对。 */
#define SAMPLE_TIM_ARR      ((84000000U / SAMPLE_FS_HZ) - 1U)

/* DMA 乒乓缓冲总点数，必须能被 2 整除（DMA 写一半时 CPU 处理另一半）。
   512 点 @48 kHz = 10.67 ms 转一圈，半满/全满各 5.33 ms 处理一次。 */
#define SAMPLE_BUF_LEN      512U

/* 滑动窗口长度，单位: 点。时间窗 = SAMPLE_WIN_M / SAMPLE_FS_HZ
   1024 点 @48 kHz = 21.33 ms。
   注意：窗口必须远大于信号周期，否则平均值会跟着信号相位起伏
   （50 Hz 在 21.33 ms 里只有 1.07 个周期，就是这么来的）。

   ★ 改 SAMPLE_FS_HZ 时这个"点数"不变，但窗口的"时长"会跟着变：
     24 kHz → 42.67 ms，96 kHz → 10.67 ms。教材要求测三种采样率时，
     这不是"同一个滤波器"的比较，写报告时必须说明。 */
#define SAMPLE_WIN_M        1024U

/* 窗口点数的倒数。[E] 由 SAMPLE_WIN_M 推导，用于把每点一次的除法
   改成块末一次的乘法（详见 Sample.c 的 Sample_ProcessBlock）。 */
#define SAMPLE_WIN_INV      (1.0 / (double)SAMPLE_WIN_M)

/* 指数平均系数 α。时间常数 τ = -1/(ln α)/采样率
   0.99 @48 kHz = 2.07 ms，比滑动窗口快十倍左右。
   同样注意：改采样率时 α 不变，τ 会跟着变（24 kHz → 4.15 ms，96 kHz → 1.04 ms）。 */
#define SAMPLE_ALPHA        0.99f

/* 1-α。单独定义一次，避免每点在循环里重复算减法。 */
#define SAMPLE_ALPHA_1M     (1.0f - SAMPLE_ALPHA)

/* 每几块 DMA 数据发一行 CSV。
   每块 256 点 = 5.33 ms，取 2 块 → 约 10.7 ms 一行 ≈ 94 行/秒。
   这个数要和波特率匹配：115200 8N1 可用带宽 11520 字节/秒，
   实测行宽 64~72 字节，94 行/秒 × 72 = 6750 B/s，占 59%。
   ★ 96 kHz 采样时块时长减半，必须把这个数提到 4，否则一定丢行。 */
#define SAMPLE_TX_BLOCKS    2U

/* 归一化参数：2048 码值 = 1.65 V = 波形中心
   [E] 12 位 ADC 满量程 4096 码 = VDDA，中点是 2048 码 = VDDA/2 = 1.65 V。 */
#define SAMPLE_ADC_MID      2048.0f
#define SAMPLE_ADC_SCALE    2048.0f

/* 1/2048。乘倒数代替除以 2048，省一个 FPU 除法（14 周期）。 */
#define SAMPLE_ADC_INV_SCALE (1.0f / SAMPLE_ADC_SCALE)

/* ===== 对外结果结构体 ===================================================== */
typedef struct
{
  uint64_t count;      /* 累计采样点数，单位: 点；除以采样率就是开机秒数。
                          用 64 位是为了避免 48 kHz 下 24.85 小时回绕。 */
  double   energy;     /* 累积能量 Σx²，归一化；× 1.65² / 采样率 = V²·s */
  float    p_win;      /* 滑动窗口平均功率，归一化 x_rms²；× 1.65² = V² */
  float    p_exp;      /* 指数平均功率，归一化 */
  float    p_all;      /* 总平均功率 = energy / count，归一化 */
  float    rms_win;    /* 滑动窗口 RMS，归一化 0~1；× 1.65 V = 实际 Vrms */
  float    rms_exp;    /* 指数平均 RMS，归一化。
                          注意：这是 sqrt(mean(x²))，不是"先开根号再平均"。
                          对纯正弦两者相等；波形不纯时前者偏大（Jensen 不等式）。 */
} Sample_ResultTypeDef;

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  初始化采样链路：PA1 模拟输入、DMA2_Stream0、ADC1、TIM2、NVIC
  * @param  None
  * @retval None
  * @note   初始化顺序：时钟 → 状态清零 → GPIO → DMA → ADC → TIM2 → NVIC，
  *         中断最后才使能（代码规范 3.5）。此时还没开始采样。
  */
void Sample_Init(void);

/**
  * @brief  启动采样：开 TIM2 触发 + 开 DMA 搬运
  * @param  None
  * @retval None
  * @note   必须在 Sample_Init() 之后调用。可重复调用，每次都会接着之前的
  *         能量继续累加（不会清零），想从头开始要先调 Sample_Init()。
  */
void Sample_Start(void);

/**
  * @brief  取一次当前的所有结果（快照）
  * @param  result 输出结构体指针，不加 const（会被写）
  * @retval None
  * @note   内部短暂关闭全局中断再读，保证 count / energy / p_win 来自同一时刻。
  *         关闭时间约 1 µs，远小于 5.33 ms 的采样中断周期，不影响采样。
  *         rms_win / rms_exp 对负值（浮点漂移）和 NaN 都做了保护，返回 0。
  */
void Sample_GetResult(Sample_ResultTypeDef *result);

/**
  * @brief  取走"该上报了"的标志（有副作用：取完自动清零）
  * @param  None
  * @retval 1 = 这一轮该发一行 CSV；0 = 还没到
  * @note   故意用 Take 而不是 Get（代码规范 3.4：Get 不许有副作用）。
  *         标志由 DMA 中断按 SAMPLE_TX_BLOCKS 的节奏置位。
  */
uint8_t Sample_TakeReport(void);

/**
  * @brief  DMA 中断处理片段：判断半满/全满 → 处理那半块数据 → 累计块数
  * @param  None
  * @retval None
  * @note   只能由 main.c 里的 DMA2_Stream0_IRQHandler() 调用。
  */
void Sample_IsrHandler(void);

#endif /* __SAMPLE_H */
