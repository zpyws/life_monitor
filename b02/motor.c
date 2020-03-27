#include <rtthread.h>
#include <board.h>
//created by yangwensen@20200326
//************************************************************************************************************
#define MOTOR_PIN    GET_PIN(5, 3)

//************************************************************************************************************
static void vibrate(uint8_t argc, char **argv)
{
    uint32_t reps,on,off;
    uint32_t i;
    
    if(argc != 4)
    {
        rt_kprintf("[Y]bad parameter! vibrate(repets,on_time,off_time)\n");
    }
    
    reps = strtol(argv[1], NULL, 0);
    on = strtol(argv[2], NULL, 0);
    off = strtol(argv[2], NULL, 0);
    
    for(i=0; i<reps; i++)
    {
        rt_pin_write(MOTOR_PIN, PIN_HIGH);
        rt_thread_mdelay(on);
        rt_pin_write(MOTOR_PIN, PIN_LOW);
        rt_thread_mdelay(off);
    }
}
MSH_CMD_EXPORT(vibrate, vibrate motor[reps.ton.toff]);
//************************************************************************************************************
static int hw_motor_init(void)
{
    rt_pin_mode(MOTOR_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(MOTOR_PIN, PIN_LOW);
    
    return RT_EOK;
}
INIT_BOARD_EXPORT(hw_motor_init);
//************************************************************************************************************

