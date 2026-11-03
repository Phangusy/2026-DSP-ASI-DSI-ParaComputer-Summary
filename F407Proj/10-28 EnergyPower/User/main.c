/**
  ******************************************************************************
  * @file    main.c
  * @brief   DSP 功率 / 能量测量 —— 主程序
  ******************************************************************************
  * 这是一台"数字功率计"，一块板子带两个表盘：
  *
  *   模拟表盘：PA4（DAC1_OUT1）输出一个正比于 RMS 的直流电压，
  *             万用表/示波器直接量，读数 ÷ 2 就是输入信号的 Vrms。
  *             ★ 示波器必须切 DC 耦合，否则看不到这个直流电平。
  *   数字表盘：PA2（USART2_TX）每约 10.7 ms 发一行 CSV 给 PC，
  *             7 个数据列，可以实时画曲线。
  *
  * 信号链：
  *   PA1 → ADC1（48 kHz，DMA 乒乓）→ 每点算 x² → 三种估计器
  *        ├→ 滑动窗口平均（21.33 ms）
  *        ├→ 指数平均（τ = 2.07 ms）
  *        └→ 累积能量
  *
  * 分工：
  *   Hardware\Sample.c —— ADC + DMA + 三种估计器（本工程核心）
  *   Hardware\Usart.c  —— USART2 + DMA 发送
  *   Hardware\Dac.c    —— DAC1 / PA4 模拟输出
  *   本文件            —— main() 和所有 xxx_IRQHandler（代码规范 4.1）
  *
  * 接线：
  *   PA1  模拟输入，必须围绕 1.65 V 摆动（发生器要开 1.65 V 直流偏置！）
  *   PA4  DAC 输出（万用表 / 示波器，DC 耦合）
  *   PA2  USART2_TX → 板载 ST-LINK 虚拟串口，或外接 CH340 的 RXD
  *   GND  共地（发生器、板子、示波器三方共地）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Sample.h"
#include "Usart.h"
#include "Dac.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/

/* CSV 表头。长度用 sizeof 算，不再手写 34 这种一改就错的魔数。 */
static const char CSV_HEADER[] = "N,E,Pwin,Pexp,Pall,RMSwin,RMSexp\r\n";

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  主函数
  * @param  None
  * @retval None
  */
int main(void)
{
  Sample_ResultTypeDef result;
  char                 line[USART_LINE_MAX];
  int                  n;

  /* 外设初始化：DAC 先好，采样和串口就绪后再真正开采样 */
  Dac_Init();
  Sample_Init();
  Usart_Init();

  /* 先发一行表头，PC 端看到它就说明串口通了 */
  (void)Usart_SendLine(CSV_HEADER, (uint16_t)(sizeof(CSV_HEADER) - 1U));

  /* 开采样：DMA 开始转，中断开始进 */
  Sample_Start();

  while (1)
  {
    /* DMA 中断按 SAMPLE_TX_BLOCKS 的节奏置这个标志 */
    if (Sample_TakeReport() != 0U)
    {
      /* 取一次快照（内部关中断，保证各字段同一时刻） */
      Sample_GetResult(&result);

      /* 模拟表盘：PA4 输出正比于 RMS 的直流 */
      Dac_SetRms(result.rms_win);

      /* 数字表盘：组一行 CSV 交给 DMA 发出去。
         N 用 %.0f 而不是 %llu —— double 在 2^53 以内能精确表示整数
         （等于 6000 年），而 %f 的浮点格式化已经确认被链接进来了
         （map 里 printfa.o 占 2300 字节），不依赖 MicroLIB 对 ll 的支持。 */
      n = snprintf(line, sizeof(line),
                   "%.0f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                   (double)result.count,
                   result.energy,
                   (double)result.p_win,
                   (double)result.p_exp,
                   (double)result.p_all,
                   (double)result.rms_win,
                   (double)result.rms_exp);

      /* n < 0 是编码错误；n >= sizeof(line) 是被截断。两种都别发。 */
      if ((n > 0) && ((size_t)n < sizeof(line)))
      {
        /* 返回 0 表示上一行还没发完，这一行丢掉就行，不影响采样 */
        (void)Usart_SendLine(line, (uint16_t)n);
      }
    }
  }
}

/* Interrupt handlers --------------------------------------------------------*/

/**
  * @brief  DMA2_Stream0 中断（ADC1 的数据搬运）
  * @param  None
  * @retval None
  * @note   代码规范 4.1：ISR 本体只能定义在含 main() 的文件里，
  *         模块只交出被它调用的片段 Sample_IsrHandler()。
  */
void DMA2_Stream0_IRQHandler(void)
{
  Sample_IsrHandler();
}

/**
  * @brief  DMA1_Stream6 中断（USART2_TX 发完了 / 传输出错）
  * @param  None
  * @retval None
  */
void DMA1_Stream6_IRQHandler(void)
{
  Usart_IsrHandler();
}
