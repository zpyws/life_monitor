#include <rtthread.h>
#include <board.h>
//created by yangwensen@20200326
//************************************************************************************************************
#define LED2_PIN    GET_PIN(8, 2)
#define LED3_PIN    GET_PIN(8, 1)
#define LED4_PIN    GET_PIN(8, 3)
#define LED5_PIN    GET_PIN(8, 0)

const uint8_t TAB_LED_PIN[] = {LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN};
//************************************************************************************************************
#ifndef ARRAY_SIZE
    #define ARRAY_SIZE(ar)     (sizeof(ar)/sizeof(ar[0]))
#endif
//************************************************************************************************************
//by yangwensen@20200611
static void led(uint8_t argc, char **argv)
{
    uint32_t index,status;
    
    if(argc != 3)
    {
        rt_kprintf("[Y]bad parameter! led(index,status)\n");
    }
    
    index = strtol(argv[1], NULL, 0);
    status = strtol(argv[2], NULL, 0);
    
    if(index >= ARRAY_SIZE(TAB_LED_PIN))
    {
        rt_kprintf("led index parameter out of range\n");
    }
    if(status >= 2)
    {
        rt_kprintf("led status parameter out of range\n");
    }
    
    rt_pin_write(TAB_LED_PIN[index], status);
}
MSH_CMD_EXPORT(led, led[off=0.on=1.tog=2]);
//************************************************************************************************************
//by yangwensen@20200611
static int hw_led_init(void)
{
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED2_PIN, PIN_LOW);

    rt_pin_mode(LED3_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED3_PIN, PIN_LOW);

    rt_pin_mode(LED4_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED4_PIN, PIN_LOW);

    rt_pin_mode(LED5_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED5_PIN, PIN_LOW);

    return RT_EOK;
}
INIT_BOARD_EXPORT(hw_led_init);
//************************************************************************************************************

