/**
 * @author greedyhao (hao_kr@163.com)
 * @version 1.0.2
 * @date 2020-2-21
 *  
 */

#include "as608.h"

#define AS60X_UART_NAME "uart3"

void as60x_sample(int argc, char *argv[])
{
    rt_uint16_t page_id = 0x02;
    rt_uint16_t mat_score = 0;
    as60x_set_hand_shake_baud(115200);
    as60x_init(AS60X_UART_NAME);
    rt_kprintf("Now test input fingerprint..\r\n");
    as60x_str_fp_to_flash(page_id);
    rt_kprintf("Now test finding the fingerprint..\r\n");
    as60x_search_fp_in_flash(&page_id, &mat_score);
    rt_kprintf("Now test delet the fingerprint..\r\n");
    as60x_delet_fp_n_id(page_id, 1);
    rt_kprintf("retry finding the fingerprint..\r\n");
    as60x_search_fp_in_flash(&page_id, &mat_score);
}
MSH_CMD_EXPORT(as60x_sample, "as60x sample");
