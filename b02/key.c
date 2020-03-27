#include <rtthread.h>
#include <board.h>
//created by yangwensen@20200326
//************************************************************************************************************
#define HOME_KEY_PIN                    GET_PIN(7, 0)
//************************************************************************************************************
static void home_key_isr(void *args)
{
    rt_kprintf("port7 int\n");
}
//************************************************************************************************************
void home_key_init(void)
{
    rt_pin_mode(HOME_KEY_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(HOME_KEY_PIN, PIN_IRQ_MODE_RISING, home_key_isr, RT_NULL);
    rt_pin_irq_enable(HOME_KEY_PIN, PIN_IRQ_ENABLE);
}
//************************************************************************************************************
