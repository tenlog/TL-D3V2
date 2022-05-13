

#include "../board/startup.h"
#include "usart.h"


stc_usart_uart_init_t stcInitCfg = {		  
UsartIntClkCkNoOutput,		  
UsartClkDiv_4,		  
UsartDataBits8, 	   
UsartDataLsbFirst,		  
UsartOneStopBit,		
UsartParityNone,		
UsartSamleBit8, 	   
UsartStartBitFallEdge,		  
UsartRtsEnable,    
};	   
ring_buffer usart1_rb;
ring_buffer usart1_wb;
ring_buffer usart2_rb;
ring_buffer usart2_wb;
ring_buffer usart3_rb;
ring_buffer usart3_wb;
ring_buffer usart4_rb;
ring_buffer usart4_wb;

usart_dev usart4 = {
    .regs     = M4_USART4,
    .rb       = &usart4_rb,
    .wb       = &usart4_wb,
    .max_baud = 2250000UL,
    .clk_id = PWC_FCG1_PERIPH_USART4,
    .pstcInitCfg = &stcInitCfg,
	.RX_IRQ   = Int020_IRQn,
	.TX_IRQ   = Int086_IRQn,
	.RX_error_IRQ   = Int087_IRQn,
	.TX_complete_IRQ   = Int088_IRQn,
	.IRQ_priority = DDL_IRQ_PRIORITY_03,
};
/***************************************/
/***************************************/
usart_dev usart3 = {
    .regs     = M4_USART3,
    .rb       = &usart3_rb,
    .wb       = &usart3_wb,
    .max_baud = 2250000UL,
    .clk_id = PWC_FCG1_PERIPH_USART3,
    .pstcInitCfg = &stcInitCfg,
	.RX_IRQ   = Int021_IRQn,
	.TX_IRQ   = Int089_IRQn,
	.RX_error_IRQ	= Int090_IRQn,
	.TX_complete_IRQ   = Int091_IRQn,
	.IRQ_priority = DDL_IRQ_PRIORITY_03,
};
/***************************************/
/***************************************/

usart_dev usart2 = {
    .regs     = M4_USART2,
    .rb       = &usart2_rb,
    .wb       = &usart2_wb,
    .max_baud = 2250000UL,
    .clk_id = PWC_FCG1_PERIPH_USART2,
    .pstcInitCfg = &stcInitCfg,
	.RX_IRQ   = Int022_IRQn,
	.TX_IRQ   = Int080_IRQn,
	.RX_error_IRQ	= Int081_IRQn,
	.TX_complete_IRQ   = Int082_IRQn,
	.IRQ_priority = DDL_IRQ_PRIORITY_03,
};
/***************************************/
/***************************************/
usart_dev usart1 = {
    .regs     = M4_USART1,
    .rb       = &usart1_rb,
    .wb       = &usart1_wb,
    .max_baud = 2250000UL,
    .clk_id = PWC_FCG1_PERIPH_USART1,
    .pstcInitCfg = &stcInitCfg,
	.RX_IRQ   = Int023_IRQn,
	.TX_IRQ   = Int083_IRQn,
	.RX_error_IRQ	= Int084_IRQn,
	.TX_complete_IRQ   = Int085_IRQn,
	.IRQ_priority = DDL_IRQ_PRIORITY_03,
};

/** USART4 device */
usart_dev *USART4 = &usart4;
/** USART3 device */
usart_dev *USART3 = &usart3;
/** USART2 device */
usart_dev *USART2 = &usart2;
/** USART1 device */
usart_dev *USART1 = &usart1;
/***************************************/
/***************************************/

void usart_foreach(void (*fn)(usart_dev*)) {
    fn(USART1);
    fn(USART2);
    fn(USART3);
    fn(USART4);
}
