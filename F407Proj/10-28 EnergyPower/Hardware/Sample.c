/**
  ******************************************************************************
  * @file    Sample.c
  * @brief   ADC 采样 + 三种功率估计（实现）
  ******************************************************************************
  * 数据流：
  *   TIM2 每 1/48kHz = 20.83 µs 触发一次 ADC1
  *     → DMA2_Stream0 自动把结果搬进 Sample_Buf[]（CPU 不参与）
  *     → 搬到一半（256 点）触发半满中断，搬满（512 点）触发全满中断
  *     → 中断里处理刚搬完的那半块：算 x²、更新三种估计器
  *     → 每 SAMPLE_TX_BLOCKS 块，置一个"该上报了"的标志
  *
  * 性能说明（这一版的最大改动，务必看）：
  *   Cortex-M4F 只有单精度硬件浮点（编译参数 -mfpu=fpv4-sp-d16）。
  *   double 的加减乘除全部是软件模拟（__aeabi_dadd / __aeabi_ddiv），
  *   一次要比 float 慢一到两个数量级。所以：
  *     1) 环形缓冲从 float 改成 double —— 省掉每点两次 float→double 转换；
  *     2) 累积能量在块内用 float 累加，块末一次性加到 double —— 省掉每点
  *        一次转换 + 一次软浮点加法；
  *     3) 滑动窗口的除法移到块末，满窗后改成乘法 —— 原来每点一次
  *        __aeabi_ddiv 是整块中断里最贵的一行。
  *   改完每点只剩 1 次 f2d + 1 次 dsub + 1 次 dadd，软浮点调用从 8 次/点
  *   降到 3 次/点，中断耗时大约减半。
  *
  *   精度不受影响：double 环形缓冲 + double 增量求和，长期漂移在 1e-12 量级；
  *   块内 float 累加 256 点，最坏舍入 3.6e-4（相对 47 是 8e-6），可忽略。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Sample.h"
#include <math.h>       /* sqrtf：把功率开根号变成 RMS */

/* Private variables ---------------------------------------------------------*/

/* DMA 目标缓冲区。4 字节对齐：DMA 半字传输要求至少 2 字节对齐，
   Watch 窗口按 uint16 显示也整齐。故意不加 static，调试器能按名字看到。 */
uint16_t Sample_Buf[SAMPLE_BUF_LEN] __attribute__((aligned(4)));

/* ---- 三种估计器的内部状态（static 跨调用保持，按规范加 _Sta） ---- */

static volatile double   Energy_Sta;      /* 累积能量 Σx²（只在块末更新） */
static volatile uint64_t Count_Sta;       /* 累计点数，64 位避免 24.85 小时回绕 */

/* 滑动窗口环形缓冲。用 double 存每点的 x²：
   这样 PSum 的增量加减不需要每点做 float→double 转换。
   内存代价 1024 × 8 = 8 KB（F407VG 有 128 KB SRAM，够）。 */
static double   PRing_Sta[SAMPLE_WIN_M] __attribute__((aligned(8)));
static uint32_t PIdx_Sta;                 /* 环形缓冲写指针 */
static double   PSum_Sta;                 /* 窗口内所有 x² 的和（增量维护） */
static uint32_t WinFill_Sta;              /* 窗口已填多少点，开机阶段用 */
static volatile float PWin_Sta;           /* 滑动窗口平均功率 */

static volatile float PExp_Sta;           /* 指数平均功率 */

static uint32_t BlockCount_Sta;           /* 已处理了多少块 */
static volatile uint8_t Report_Flag_Sta;  /* 1 = 该发一行 CSV 了 */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  处理一块采样数据，更新三种估计器
  * @param  buf  这块数据的首地址
  * @param  len  这块有多少点
  * @retval None
  * @note   只在 DMA 中断里调用。逐点做的运算要尽量少，
  *         因为每 5.33 ms 就要把 256 个点全部过一遍。
  */
static void Sample_ProcessBlock(const uint16_t *buf, uint16_t len)
{
  uint16_t i;
  float    x;
  float    p;
  double   pd;
  float    energy_inc = 0.0f;   /* 块内单精度累加，块末一次性加到 double */

  for (i = 0U; i < len; i++)
  {
    /* 归一化：2048 码值当零点，±2048 码值对应 ±1.0。
       [E] x = (code - 2048) / 2048，即 V/1.65 - 1 的归一化形式。 */
    x = ((float)buf[i] - SAMPLE_ADC_MID) * SAMPLE_ADC_INV_SCALE;

    /* 瞬时功率（归一化，等于 x 的平方） */
    p = x * x;

    /* ---- 估计器 1：累积能量 ----
       块内只做 float 加法（硬件 FPU，1 周期），块末再转 double。
       256 点累加的最坏舍入误差约 3.6e-4，相对块内总和 47 是 8e-6。 */
    energy_inc += p;

    /* ---- 估计器 2：滑动窗口平均 ----
       环形缓冲里"顶掉最老的、放进最新的"，用增量维护和，
       这样每个点只要两次加减，不用每次把 1024 个点重新加一遍。
       这里只做一次 float→double 转换，扣旧值和加新值共用。 */
    pd = (double)p;
    PSum_Sta -= PRing_Sta[PIdx_Sta];
    PRing_Sta[PIdx_Sta] = pd;
    PSum_Sta += pd;

    PIdx_Sta++;
    if (PIdx_Sta >= SAMPLE_WIN_M)
    {
      PIdx_Sta = 0U;
    }

    /* 开机阶段窗口没填满，除数要用实际点数，否则平均值会被人为压小 */
    if (WinFill_Sta < SAMPLE_WIN_M)
    {
      WinFill_Sta++;
    }

    /* ---- 估计器 3：指数平均（一阶 IIR 低通） ----
       全程单精度，走硬件 FPU，很便宜。 */
    PExp_Sta = SAMPLE_ALPHA * PExp_Sta + SAMPLE_ALPHA_1M * p;
  }

  /* ---- 块末统一提交（每块只做一次，不再每点一次） ---- */
  Energy_Sta += (double)energy_inc;
  Count_Sta  += (uint64_t)len;

  /* 窗口填满后是常数除法，改成乘倒数：省掉一次软件浮点除法。
     只有开机前 4 块（窗口还没填满）才走真正的除法。 */
  if (WinFill_Sta >= SAMPLE_WIN_M)
  {
    PWin_Sta = (float)(PSum_Sta * SAMPLE_WIN_INV);
  }
  else
  {
    PWin_Sta = (float)(PSum_Sta / (double)WinFill_Sta);
  }
}

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  初始化采样链路
  * @param  None
  * @retval None
  */
void Sample_Init(void)
{
  GPIO_InitTypeDef        GPIO_InitStructure      = {0};
  DMA_InitTypeDef         DMA_InitStructure       = {0};
  ADC_CommonInitTypeDef   ADC_CommonInitStructure = {0};
  ADC_InitTypeDef         ADC_InitStructure       = {0};
  TIM_TimeBaseInitTypeDef TIM_InitStructure       = {0};
  NVIC_InitTypeDef        NVIC_InitStructure      = {0};
  uint16_t i;

  /* --- 1. 外设时钟 --- */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);    /* PA1 */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,  ENABLE);    /* ADC1 走 DMA2_Stream0 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE);

  /* --- 2. 状态全部清零（规范 3.5：状态就绪后才开中断） --- */
  Energy_Sta      = 0.0;
  Count_Sta       = 0U;
  PIdx_Sta        = 0U;
  PSum_Sta        = 0.0;
  WinFill_Sta     = 0U;
  PWin_Sta        = 0.0f;
  PExp_Sta        = 0.0f;
  BlockCount_Sta  = 0U;
  Report_Flag_Sta = 0U;

  for (i = 0U; i < SAMPLE_WIN_M; i++)
  {
    PRing_Sta[i] = 0.0;
  }

  /* --- 3. PA1 配成模拟输入 --- */
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* --- 4. DMA2_Stream0 / Channel 0（ADC1 的固定映射） ---
     循环模式：搬完 512 点自动回到开头，永远不停，所以能一直采。 */
  DMA_InitStructure.DMA_Channel            = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)Sample_Buf;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize         = SAMPLE_BUF_LEN;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   /* 外设地址固定 */
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;        /* 内存地址递增 */
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);

  /* 半传输 + 传输完成两个中断都要开，这就是"乒乓"的来源 */
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* --- 5. ADC 公共配置 ---
     预分频 Div4 ⇒ ADC 时钟 = 84 MHz / 4 = 21 MHz（手册上限 36 MHz）。 */
  ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;
  ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* --- 6. ADC1 本体：12 位 / 单通道 / 单次 / T2_TRGO 上升沿触发 --- */
  ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode         = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode   = DISABLE;   /* 采样率由 TIM2 决定 */
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T2_TRGO;
  ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion      = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* 采样时间 84 个 ADC 时钟 = 4 µs（21 MHz 下），占 20.83 µs 周期的 19%。
     取长一点是为了让采样电容在源阻抗偏高时也能充满；
     注意不要短到让"采样 + 转换"（84 + 12 周期 = 4.57 µs）逼近触发周期。 */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);

  /* 转换完成就发一次 DMA 请求；最后一次传输后仍然继续请求，配合 DMA 循环模式 */
  ADC_DMACmd(ADC1, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);

  /* --- 7. TIM2 时基 + TRGO ---
     84 MHz / (1749 + 1) = 48 kHz，ARR 由 Sample.h 的 SAMPLE_FS_HZ 自动算。 */
  TIM_TimeBaseStructInit(&TIM_InitStructure);
  TIM_InitStructure.TIM_Prescaler     = 0U;
  TIM_InitStructure.TIM_Period        = SAMPLE_TIM_ARR;
  TIM_InitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);

  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

  /* --- 8. 最后才开中断 --- */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  启动采样
  * @param  None
  * @retval None
  */
void Sample_Start(void)
{
  /* 开 TIM2：第一个 TRGO 在 20.83 µs 后到来，ADC 开始被触发 */
  TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  取一次当前结果快照
  * @param  result 输出结构体
  * @retval None
  * @note   先关中断再读，保证 count / energy / p_win 来自同一时刻。
  *         之前是分几次裸读，如果 DMA 中断刚好插在两次读之间，
  *         p_all = energy / count 会用"新能量除旧点数"（开机头几十行能偏 25%）。
  *         关中断时间约 1 µs，采样中断周期 5.33 ms，不影响采样。
  */
void Sample_GetResult(Sample_ResultTypeDef *result)
{
  uint32_t primask;
  uint64_t count;
  double   energy;
  float    pwin;
  float    pexp;

  /* 保存并关中断。用 __set_PRIMASK 恢复而不是 __enable_irq()：
     后者是无条件开中断，如果将来有人在已经关中断的上下文里调用这个函数，
     会意外把中断打开。 */
  primask = __get_PRIMASK();
  __disable_irq();
  count  = Count_Sta;
  energy = Energy_Sta;
  pwin   = PWin_Sta;
  pexp   = PExp_Sta;
  __set_PRIMASK(primask);

  result->count   = count;
  result->energy  = energy;
  result->p_win   = pwin;
  result->p_exp   = pexp;
  result->p_all   = (count > 0U) ? (float)(energy / (double)count) : 0.0f;

  /* RMS 就是功率开根号。
     写成 (p > 0) 而不是 sqrtf(fabs(p))：浮点漂移偶尔会让 PSum 微小变负，
     直接开根号会得到 NaN，NaN 会一路污染 DAC 和 CSV，所以这里钳到 0。
     （NaN > 0 为假，因此 NaN 也被这条路挡住。） */
  result->rms_win = (pwin > 0.0f) ? sqrtf(pwin) : 0.0f;
  result->rms_exp = (pexp > 0.0f) ? sqrtf(pexp) : 0.0f;
}

/**
  * @brief  取走上报标志
  * @param  None
  * @retval 1 = 该发一行 CSV；0 = 还没到
  */
uint8_t Sample_TakeReport(void)
{
  if (Report_Flag_Sta != 0U)
  {
    Report_Flag_Sta = 0U;
    return 1U;
  }

  return 0U;
}

/**
  * @brief  DMA 中断处理片段
  * @param  None
  * @retval None
  */
void Sample_IsrHandler(void)
{
  /* 半满中断：DMA 正在写后半段，所以处理前半段 [0 .. 半长-1] */
  if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0) != RESET)
  {
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);

    Sample_ProcessBlock(&Sample_Buf[0], SAMPLE_BUF_LEN / 2U);

    BlockCount_Sta++;
    if ((BlockCount_Sta % SAMPLE_TX_BLOCKS) == 0U)
    {
      Report_Flag_Sta = 1U;
    }
  }

  /* 全满中断：DMA 绕回开头写前半段，所以处理后半段 */
  if (DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) != RESET)
  {
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

    Sample_ProcessBlock(&Sample_Buf[SAMPLE_BUF_LEN / 2U], SAMPLE_BUF_LEN / 2U);

    BlockCount_Sta++;
    if ((BlockCount_Sta % SAMPLE_TX_BLOCKS) == 0U)
    {
      Report_Flag_Sta = 1U;
    }
  }
}
