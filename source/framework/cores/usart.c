
#include "../board/startup.h"
#include "usart.h"

struct usart_dev* dev4 = (struct usart_dev*)&usart4;
struct usart_dev* dev3 = (struct usart_dev*)&usart3;
struct usart_dev* dev2 = (struct usart_dev*)&usart2;
struct usart_dev* dev1 = (struct usart_dev*)&usart1;

/******************************************************/
/******************************************************/
void Usart4RxIrqCallback(void)
{  
    usart_rx_irq(dev4->rb, dev4->regs);
}
void Usart4ErrIrqCallback(void)
{
    if (Set == USART_GetStatus(dev4->regs, UsartFrameErr))
    {
    	USART_ClearStatus(dev4->regs, UsartFrameErr);
    }
    if (Set == USART_GetStatus(dev4->regs, UsartParityErr))
    {
    	USART_ClearStatus(dev4->regs, UsartParityErr);
    }
    if (Set == USART_GetStatus(dev4->regs, UsartOverrunErr))
    {
    	USART_ClearStatus(dev4->regs, UsartOverrunErr);
    }
}

void Usart4TxIrqCallback(void)
{    
    usart_tx_irq(dev4->wb, dev4->regs);
}

void Usart4TxCmpltIrqCallback(void)
{    
    USART_FuncCmd(dev4->regs, UsartTxCmpltInt, Disable);    
    USART_FuncCmd(dev4->regs, UsartTx, Disable);
}
/******************************************************/
/******************************************************/
void Usart3RxIrqCallback(void)
{  
    usart_rx_irq(dev3->rb, dev3->regs);
}
void Usart3ErrIrqCallback(void)
{
    if (Set == USART_GetStatus(dev3->regs, UsartFrameErr))
    {
    	USART_ClearStatus(dev3->regs, UsartFrameErr);
    }
    if (Set == USART_GetStatus(dev3->regs, UsartParityErr))
    {
    	USART_ClearStatus(dev3->regs, UsartParityErr);
    }
    if (Set == USART_GetStatus(dev3->regs, UsartOverrunErr))
    {
    	USART_ClearStatus(dev3->regs, UsartOverrunErr);
    }
}

void Usart3TxIrqCallback(void)
{    
    usart_tx_irq(dev3->wb, dev3->regs);
}

void Usart3TxCmpltIrqCallback(void)
{    
    USART_FuncCmd(dev3->regs, UsartTxCmpltInt, Disable);    
    USART_FuncCmd(dev3->regs, UsartTx, Disable);
}

/******************************************************/
/******************************************************/
void Usart2RxIrqCallback(void)
{  
    usart_rx_irq(dev2->rb, dev2->regs);
}
void Usart2ErrIrqCallback(void)
{
    if (Set == USART_GetStatus(dev2->regs, UsartFrameErr))
    {
    	USART_ClearStatus(dev2->regs, UsartFrameErr);
    }
    if (Set == USART_GetStatus(dev2->regs, UsartParityErr))
    {
    	USART_ClearStatus(dev2->regs, UsartParityErr);
    }
    if (Set == USART_GetStatus(dev2->regs, UsartOverrunErr))
    {
    	USART_ClearStatus(dev2->regs, UsartOverrunErr);
    }
}

void Usart2TxIrqCallback(void)
{    
    usart_tx_irq(dev2->wb, dev2->regs);
}

void Usart2TxCmpltIrqCallback(void)
{    
    USART_FuncCmd(dev2->regs, UsartTxCmpltInt, Disable);    
    USART_FuncCmd(dev2->regs, UsartTx, Disable);
}

/******************************************************/
/******************************************************/
void Usart1RxIrqCallback(void)
{  
    usart_rx_irq(dev1->rb, dev1->regs);
}
void Usart1ErrIrqCallback(void)
{
    if (Set == USART_GetStatus(dev1->regs, UsartFrameErr))
    {
    	USART_ClearStatus(dev1->regs, UsartFrameErr);
    }
    if (Set == USART_GetStatus(dev1->regs, UsartParityErr))
    {
    	USART_ClearStatus(dev1->regs, UsartParityErr);
    }
    if (Set == USART_GetStatus(dev1->regs, UsartOverrunErr))
    {
    	USART_ClearStatus(dev1->regs, UsartOverrunErr);
    }
}

void Usart1TxIrqCallback(void)
{    
    usart_tx_irq(dev1->wb, dev1->regs);
}

void Usart1TxCmpltIrqCallback(void)
{    
    USART_FuncCmd(dev1->regs, UsartTxCmpltInt, Disable);    
    USART_FuncCmd(dev1->regs, UsartTx, Disable);
}

/******************************************************/
/******************************************************/

void usart_init(usart_dev *dev) {
	rb_init(dev->rb, USART_RX_BUF_SIZE, dev->rx_buf);
	rb_init(dev->wb, USART_TX_BUF_SIZE, dev->tx_buf);

	/* Enable peripheral clock */
	 PWC_Fcg1PeriphClockCmd(dev->clk_id, Enable);
	/* Initialize UART */
	USART_UART_Init(dev->regs, dev->pstcInitCfg);
}

void usart_set_baud_rate(usart_dev *dev, uint32 baud) {
	/* Set baudrate */
	USART_SetBaudrate(dev->regs, baud);
}

void usart_enable(usart_dev *dev) {
	 stc_irq_regi_conf_t stcIrqRegiCfg;
	
	/* Set USART RX IRQ */
	stcIrqRegiCfg.enIRQn = dev->RX_IRQ;
	if(dev->regs== dev4->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART4_RI;
		stcIrqRegiCfg.pfnCallback = &Usart4RxIrqCallback;
	}
	else if(dev->regs == dev3->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART3_RI;
		stcIrqRegiCfg.pfnCallback = &Usart3RxIrqCallback;
	}
	else if(dev->regs == dev2->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART2_RI;
		stcIrqRegiCfg.pfnCallback = &Usart2RxIrqCallback;
	}
	else if(dev->regs == dev1->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART1_RI;
		stcIrqRegiCfg.pfnCallback = &Usart1RxIrqCallback;
	}
	enIrqRegistration(&stcIrqRegiCfg);
	NVIC_SetPriority(stcIrqRegiCfg.enIRQn, dev->IRQ_priority);
	NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
	NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
	
	/* Set USART RX error IRQ */
	stcIrqRegiCfg.enIRQn = dev->RX_error_IRQ;
	if(dev->regs == dev4->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART4_EI;
		stcIrqRegiCfg.pfnCallback = &Usart4ErrIrqCallback;
	}
	else if(dev->regs == dev3->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART3_EI;
		stcIrqRegiCfg.pfnCallback = &Usart3ErrIrqCallback;
	}
	else if(dev->regs == dev2->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART2_EI;
		stcIrqRegiCfg.pfnCallback = &Usart2ErrIrqCallback;
	}
	else if(dev->regs == dev1->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART1_EI;
		stcIrqRegiCfg.pfnCallback = &Usart1ErrIrqCallback;
	}
	enIrqRegistration(&stcIrqRegiCfg);
	NVIC_SetPriority(stcIrqRegiCfg.enIRQn, dev->IRQ_priority);
	NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
	NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
	
	/* Set USART TX IRQ */
	stcIrqRegiCfg.enIRQn = dev->TX_IRQ;
	if(dev->regs == dev4->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART4_TI;
		stcIrqRegiCfg.pfnCallback = &Usart4TxIrqCallback;
	}
	else if(dev->regs == dev3->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART3_TI;
		stcIrqRegiCfg.pfnCallback = &Usart3TxIrqCallback;
	}
	else if(dev->regs == dev2->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART2_TI;
		stcIrqRegiCfg.pfnCallback = &Usart2TxIrqCallback;
	}
	else if(dev->regs == dev1->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART1_TI;
		stcIrqRegiCfg.pfnCallback = &Usart1TxIrqCallback;
	}
	enIrqRegistration(&stcIrqRegiCfg);
	NVIC_SetPriority(stcIrqRegiCfg.enIRQn, dev->IRQ_priority);
	NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
	NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
	
	/* Set USART TX complete IRQ */
	
	stcIrqRegiCfg.enIRQn = dev->TX_complete_IRQ;
	if(dev->regs == dev4->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART4_TCI;
		stcIrqRegiCfg.pfnCallback = &Usart4TxCmpltIrqCallback;
	}
	else if(dev->regs == dev3->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART3_TCI;
		stcIrqRegiCfg.pfnCallback = &Usart3TxCmpltIrqCallback;
	}
	else if(dev->regs == dev2->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART2_TCI;
		stcIrqRegiCfg.pfnCallback = &Usart2TxCmpltIrqCallback;
	}
	else if(dev->regs == dev1->regs)
	{
		stcIrqRegiCfg.enIntSrc = INT_USART1_TCI;
		stcIrqRegiCfg.pfnCallback = &Usart1TxCmpltIrqCallback;
	}
	enIrqRegistration(&stcIrqRegiCfg);
	NVIC_SetPriority(stcIrqRegiCfg.enIRQn, dev->IRQ_priority);
	NVIC_ClearPendingIRQ(stcIrqRegiCfg.enIRQn);
	NVIC_EnableIRQ(stcIrqRegiCfg.enIRQn);
		
	/*Enable RX && RX interupt function*/
	USART_FuncCmd(dev->regs, UsartRx, Enable);
	USART_FuncCmd(dev->regs, UsartRxInt, Enable);
}

void usart_disable(usart_dev *dev) {
    while(!rb_is_empty(dev->wb));
     USART_DeInit(dev->regs);
    /* Clean up buffer */
    usart_reset_rx(dev);
    usart_reset_tx(dev);
}

uint32 usart_tx(usart_dev *dev, const uint8 *buf, uint32 len) {
	uint32 txed = 0;
	uint32 errcnt=0;
	while (!rb_is_empty(dev->wb))
	{
		if(++errcnt>500000)
		{
			errcnt = 0;
			return txed;
		}
	}
	while (txed < len)
	{
		if(++errcnt>500000)
		{
			errcnt = 0;
			break;
		}
		if (rb_safe_insert(dev->wb, buf[txed]))
		{
			txed++;
		}
		else
			break;
	}
	if (!rb_is_empty(dev->wb)) 
	{
		USART_FuncCmd(dev->regs, UsartTxAndTxEmptyInt, Enable);
	}
	return txed;
}

uint32 usart_rx(usart_dev *dev, uint8 *buf, uint32 len) {
    uint32 rxed = 0;
    uint32 errcnt=0;
    while (usart_data_available(dev) && rxed < len) {
        *buf++ = usart_getc(dev);
        rxed++;
        if(++errcnt>500000)
        {
            	errcnt = 0;
            	break;
        }
    }
    return rxed;
}

void usart_putudec(usart_dev *dev, uint32 val) {
    char digits[12];
    int i = 0;

    do {
        digits[i++] = val % 10 + '0';
        val /= 10;
    } while (val > 0);

    while (--i >= 0) {
        usart_putc(dev, digits[i]);
    }
}

