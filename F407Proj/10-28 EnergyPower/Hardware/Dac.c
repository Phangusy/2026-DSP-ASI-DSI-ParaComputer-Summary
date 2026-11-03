/**
  ******************************************************************************
  * @file    Dac.c
  * @brief   DAC 模拟输出模块实现（PA4，直流电平 ∝ RMS）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Dac.h"

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  初始化 DAC1 通道 1
  * @param  None
  * @retval None
  */
void Dac_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};
  DAC_InitTypeDef  DAC_InitStructure  = {0};

  /* --- 1. 时钟 --- */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   /* PA4 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,   ENABLE);   /* DAC 挂 APB1 */

  /* --- 2. PA4 配成模拟模式（DAC 输出脚必须这样配，否则被 GPIO 抢走） --- */
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* --- 3. DAC1 通道 1 ---
     Trigger = None：不需要定时器触发，软件写一次立刻生效（这里输出的是
     缓慢变化的直流电平，不是波形，所以不需要定时刷新）。
     OutputBuffer = Enable：内部输出缓冲打开，才能直接驱动万用表/示波器。
     注意：带输出缓冲时，输出两端各有约 0.2 V 的不可达区（datasheet 的
     DAC 输出范围一项），所以实际不是精确的 0~3.3 V，两端要留余量。 */
  DAC_InitStructure.DAC_Trigger                      = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration               = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  DAC_InitStructure.DAC_OutputBuffer                 = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  /* 先给 0 V，避免上电瞬间输出不确定 */
  DAC_SetChannel1Data(DAC_Align_12b_R, 0U);
  DAC_Cmd(DAC_Channel_1, ENABLE);
}

/**
  * @brief  按归一化 RMS 设置输出电压
  * @param  rms 归一化 RMS，0~1
  * @retval None
  * @note   对 NaN 做了保护：原来的写法是 `if (u < 0)` + `if (u > 1)`，
  *         而 IEEE-754 下 NaN 与任何数比较都是"无序"，两个条件都不成立，
  *         NaN 会一路传到 VCVT.U32.F32（结果为 0，不崩但静默输出 0 V）。
  *         改成 `if (!(u >= 0))` 之后，负数和 NaN 都走同一条钳位分支。
  */
void Dac_SetRms(float rms)
{
  float    u;
  uint32_t code;

  u = rms * DAC_RMS_INV_FULL_SCALE;

  /* !(u >= 0) 对负数成立，对 NaN 也成立（NaN 比较结果为无序） */
  if (!(u >= 0.0f))
  {
    u = 0.0f;
  }
  if (u > 1.0f)
  {
    u = 1.0f;
  }

  /* +0.5f 做四舍五入（原来是直接截断，最大 1 LSB 的向下偏差）。
     u 已经钳到 [0,1]，所以最大 4095.5，截断后不会溢出 4096。 */
  code = (uint32_t)(u * DAC_MAX_CODE + 0.5f);
  DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)code);
}
