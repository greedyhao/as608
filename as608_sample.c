/**
 * @author greedyhao (hao_kr@163.com)
 * @version 0.0.1
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
    // rt_err_t ret = RT_EOK;

    as60x_init(AS60X_UART_NAME);

    //return ret;
}
MSH_CMD_EXPORT(as60x_sample, "as60x sample");
