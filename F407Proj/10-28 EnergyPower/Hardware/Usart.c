/**
  ******************************************************************************
  * @file    Usart.c
  * @brief   串口上传模块实现（USART2 + DMA 发送）
  ******************************************************************************
  * 为什么这么绕：一行 CSV 约 64~72 字节，115200 下全阻塞发送要 5~6 ms，
  * 主循环会被拖住一半时间。改成 DMA 之后：
  *   主循环只做 memcpy（几十纳秒）→ 启动 DMA → 立刻返回
  *   硬件搬完这一行 → 进一次中断 → 释放缓冲
  *
  * 关于那两块缓冲（说明更正，原来的注释说的是错的）：
  *   TxBusy_Sta 是在 memcpy **之前**检查的，忙就直接丢这一行。
  *   所以两块的真正作用不是"一边发一边准备下一行"（那需要发送队列），
  *   而是保证正在被 DMA 读的那一块，永远不会被下一次 memcpy 踩掉 ——
  *   因为 TxIdx_Sta 每次发完就翻转，DMA 读的永远是上一块。
  *   丢行由带宽决定（见 Usart.h 的带宽核算），不靠缓冲解决。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Usart.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/

/* 两块发送缓冲交替使用，保证 DMA 正在读的那块不被覆盖。 */
static char              TxBuf_Sta[2][USART_LINE_MAX];
static volatile uint8_t  TxIdx_Sta;      /* 下一次用哪一块：0 或 1 */
static volatile uint8_t  TxBusy_Sta;     /* 1 = 有一包正在发 */

/* 传输错误计数。正常应该一直是 0；如果这个数在涨，说明 DMA/USART 配置
   或硬件有问题（比如外设被别的地方抢了）。调试时在 Watch 窗口看它。 */
static volatile uint32_t TxErr_Sta;

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  初始化 USART2 与它的 DMA 发送通道
  * @param  None
  * @retval None
  */
void Usart_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure  = {0};
  USART_InitTypeDef USART_InitStructure = {0};
  DMA_InitTypeDef   DMA_InitStructure   = {0};
  NVIC_InitTypeDef  NVIC_InitStructure  = {0};

  /* --- 1. 时钟 --- */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);   /* PA2 / PA3 */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,   ENABLE);   /* USART2_TX 走 DMA1 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* --- 2. PA2 = TX，PA3 = RX，复用功能 AF7（USART2） --- */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;    /* 空闲时保持高电平，标准 UART 空闲态 */
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* --- 3. USART2：115200 8N1，无流控，收发都开 --- */
  USART_InitStructure.USART_BaudRate            = USART_BAUD_HZ;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART2, &USART_InitStructure);

  /* --- 4. DMA1_Stream6 / Channel4（USART2_TX 的固定映射） ---
     用普通模式而不是循环：每发一行重新装一次长度，发完就停，
     这样"发完了没有"用一次传输完成中断就能判断。 */
  DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART2->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)TxBuf_Sta[0];
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize         = USART_LINE_MAX;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;   /* 永远写 DR */
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);

  /* ★ 必须同时开传输错误中断（TE）。
     只开 TC 的话，一旦发生一次总线错误，DMA 会被硬件关掉、置 TEIF，
     而 TCIF 永远不来 → TxBusy_Sta 一直是 1 → 之后每一行都被丢弃，
     串口彻底哑掉而且没有任何提示。开了 TE 才能自己恢复。 */
  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC | DMA_IT_TE, ENABLE);

  /* 让 USART2 的发送请求走 DMA */
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
  USART_Cmd(USART2, ENABLE);

  /* --- 5. 中断 --- */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   /* 比采样的 DMA 低一级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* 状态清零 */
  TxIdx_Sta  = 0U;
  TxBusy_Sta = 0U;
  TxErr_Sta  = 0U;
}

/**
  * @brief  发一行字符串（交给 DMA 搬，立即返回）
  * @param  str  内容
  * @param  len  字节数
  * @retval 1 = 已启动发送；0 = 上一包还没发完，本次丢弃
  */
uint8_t Usart_SendLine(const char *str, uint16_t len)
{
  uint32_t guard;

  /* 上一包还在搬就直接放弃这一行：丢一行 CSV 比卡住主循环好得多 */
  if (TxBusy_Sta != 0U)
  {
    return 0U;
  }

  if (len == 0U)
  {
    return 0U;
  }
  if (len > USART_LINE_MAX)
  {
    len = USART_LINE_MAX;
  }

  /* 拷到当前这块缓冲（另一块可能正被 DMA 读） */
  memcpy(TxBuf_Sta[TxIdx_Sta], str, len);

  TxBusy_Sta = 1U;

  /* 重新装填 DMA：
     关流 → 等它真的关掉（EN 位清零有时要好几个周期）→ 换地址和长度 → 开流。
     中间直接写 M0AR 换缓冲，比整个 DMA_Init 再来一遍快得多。

     guard 是防死循环的：如果流的 EN 位因为硬件异常一直不归零，
     原来那个裸 while 会把整个主循环永久卡死在这里，连串口都救不回来。
     超时就放弃这一行，让主循环继续跑。 */
  DMA_Cmd(DMA1_Stream6, DISABLE);
  guard = 0U;
  while ((DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) && (guard < 10000U))
  {
    guard++;
  }
  if (guard >= 10000U)
  {
    TxBusy_Sta = 0U;
    return 0U;
  }

  DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 |
                              DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);

  DMA1_Stream6->M0AR = (uint32_t)TxBuf_Sta[TxIdx_Sta];
  DMA_SetCurrDataCounter(DMA1_Stream6, len);
  DMA_Cmd(DMA1_Stream6, ENABLE);

  /* 下次用另一块 */
  TxIdx_Sta ^= 1U;

  return 1U;
}

/**
  * @brief  DMA 中断处理片段（发送完成 / 传输出错）
  * @param  None
  * @retval None
  */
void Usart_IsrHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
  {
    DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

    /* 这一包发完了，缓冲可以再用 */
    TxBusy_Sta = 0U;
  }

  /* 传输错误：硬件已经把流关掉了。这里只释放忙标志，
     不去重新 ENABLE —— 下一次 Usart_SendLine 会完整重配地址和长度，
     在这里贸然使能会按旧的 NDTR 把缓冲区里的残留字节再发一遍。 */
  if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TEIF6) != RESET)
  {
    DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TEIF6);

    TxErr_Sta++;
    TxBusy_Sta = 0U;
  }
}
