#include "bluetooth.h"
#include <string.h>

static volatile uint8_t rx_buf[BT_BUF_LEN];
static volatile uint8_t w_ptr = 0;
static volatile uint8_t r_ptr = 0;


void BT_Init(uint32_t baud)
{
    GPIO_InitTypeDef gp;
    USART_InitTypeDef us;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    gp.GPIO_Pin   = BT_TX_PIN | BT_RX_PIN;
    gp.GPIO_Mode  = GPIO_Mode_AF;
    gp.GPIO_OType = GPIO_OType_PP;
    gp.GPIO_Speed = GPIO_Speed_100MHz;
    gp.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(BT_PORT, &gp);

    GPIO_PinAFConfig(BT_PORT, GPIO_PinSource9,  BT_AF);
    GPIO_PinAFConfig(BT_PORT, GPIO_PinSource10, BT_AF);

    us.USART_BaudRate            = baud;
    us.USART_WordLength          = USART_WordLength_8b;
    us.USART_StopBits            = USART_StopBits_1;
    us.USART_Parity              = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(BT_USART, &us);

    USART_ITConfig(BT_USART, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(BT_USART, ENABLE);
    BT_ClearBuf();
}

void BT_ClearBuf(void)
{
    w_ptr = r_ptr = 0;
    memset((void*)rx_buf, 0, BT_BUF_LEN);
}

void BT_SendChar(char ch)
{
    while (!USART_GetFlagStatus(BT_USART, USART_FLAG_TXE));
    USART_SendData(BT_USART, ch);
}

void BT_SendStr(const char *str)  // 修复：const char*
{
    while (*str) BT_SendChar(*str++);
}

char BT_Read(void)
{
    char dat;
    if (w_ptr == r_ptr) return 0;
    dat = rx_buf[r_ptr];
    r_ptr = (r_ptr + 1) % BT_BUF_LEN;
    return dat;
}

// 串口接收中断
void USART1_IRQHandler(void)
{
    uint8_t next;
    uint8_t data;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        data = (uint8_t)USART_ReceiveData(USART1);
        next = (w_ptr + 1) % BT_BUF_LEN;

        if (next == r_ptr)
        {
            r_ptr = (r_ptr + 1) % BT_BUF_LEN;
        }

        rx_buf[w_ptr] = data;
        w_ptr = next;
    }
}

