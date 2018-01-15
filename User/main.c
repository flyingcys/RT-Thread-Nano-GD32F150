#include "gd32f1x0.h"
#include <rtthread.h>


/**
  *
  *
  *
  */
int main(void)
{
	while(1)
	{
		rt_thread_delay(100);
	}
} 
#if 1
#include "finsh.h"
int hello_world(void)
{
    rt_kprintf("%s\n", "hello world!");

    return 0;
}
MSH_CMD_EXPORT(hello_world, printf hello world);
#endif

