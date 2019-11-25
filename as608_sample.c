/**
 * @author greedyhao (hao_kr@163.com)
 * @version 1.0.0
 * @date 2019-11-24
 *  
 */

#include "as608.h"

#define AS60X_UART_NAME "uart3"

void as60x_sample_entry(void *parm)
{

}

void as60x_sample(int argc, char *argv[])
{
    as60x_set_hand_shake_baud(115200);
    as60x_init(AS60X_UART_NAME);
    rt_kprintf("Now test input fingerprint..\r\n");
    as60x_str_fp_to_flash(0x02);
    rt_kprintf("Now test finding the fingerprint..\r\n");
    as60x_search_fp_in_flash();
}
MSH_CMD_EXPORT(as60x_sample, "as60x sample");
