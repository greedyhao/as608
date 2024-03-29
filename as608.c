/**
 * @author greedyhao (hao_kr@163.com)
 * @version 1.0.2
 * @date 2020-2-21
 *  
 */

#include "as608.h"

#define DBG_TAG              "fp.dev"
#define DBG_LVL              DBG_INFO
// #define DBG_LVL              DBG_LOG
#include <rtdbg.h>

#define BUF_SIZE    50U
#define RETRY_CNT   20U
static rt_uint8_t tx_buf[BUF_SIZE] = {AS60X_FP_HEAD_H, AS60X_FP_HEAD_L, AS60X_FP_ADDR_0,
                                AS60X_FP_ADDR_1, AS60X_FP_ADDR_2, AS60X_FP_ADDR_3};
static rt_uint8_t rx_buf[BUF_SIZE];

#define EVENT_AS60X_RX      (1 << 0)
#define EVENT_AS60X_TOUCH   (1 << 1)
static struct rt_event event_fp;
static rt_event_t evt_fp_p = &event_fp;
static rt_uint32_t evt_touch_set = (rt_uint32_t)EVENT_AS60X_TOUCH;
static rt_device_t as60x_dev;
static rt_uint8_t flag_vfy = 0; /* 握手成功标志 */
static rt_uint32_t hs_baud = 0;
static rt_uint8_t flag_init = 0;

/**
 * @brief verify checksum in buf is right
 * 
 * @param buf 
 * @return rt_uint8_t 0 is error, 1 is ok
 */
static rt_uint8_t cnt_checksum(rt_uint8_t *buf)
{
    rt_uint8_t checksum_is_ok = 0;
    rt_uint16_t i = 0;
    rt_uint16_t checksum = 0; /* 超出两字节的进位不理会 */
    rt_uint8_t pkg_len_h = buf[AS60X_FP_LEN_BIT] << 8;
    rt_uint16_t pkg_len = pkg_len_h + buf[AS60X_FP_LEN_BIT+1];

    if (!((rx_buf[AS60X_FP_HEAD_BIT] == AS60X_FP_HEAD_H) && (rx_buf[AS60X_FP_HEAD_BIT+1] == AS60X_FP_HEAD_L)))
        return checksum_is_ok;

    for (i = 0; i < pkg_len; i++)
    {
        checksum += buf[i+AS60X_FP_LEN_BIT];
    }
    checksum += buf[AS60X_FP_TOK_BIT];

    if ((*(buf+AS60X_PREFIX_SIZE+pkg_len-2) == ((checksum&0xff00)>>8))
    && (*(buf+AS60X_PREFIX_SIZE+pkg_len-1) == (checksum&0x00ff)))
        checksum_is_ok = 1;
    else
    {
        LOG_E("checksum_1:%x,checksum_2:%x,sum_1:%x,sum_2:%x", checksum&0xff00,checksum&0x00ff,*(buf+AS60X_PREFIX_SIZE+pkg_len-2),*(buf+AS60X_PREFIX_SIZE+pkg_len-1));
    }

    return checksum_is_ok;
}

/**
 * @brief tx_buf add checksum
 * 
 * @param buf
 * @note 包长度 = 包长度至校验和（指令、参数或数据）的总字节数，包含校验和，但不包含包长度本身的字节数
 *       校验和是从包标识至校验和之间所有字节之和
 */
static void tx_buf_add_checksum(rt_uint8_t *buf)
{
    rt_uint16_t i = 0;
    rt_uint16_t checksum = 0; /* 超出两字节的进位不理会 */
    rt_uint8_t pkg_len_h = buf[AS60X_FP_LEN_BIT] << 8;
    rt_uint16_t pkg_len = pkg_len_h + buf[AS60X_FP_LEN_BIT+1];

    for (i = 0; i < pkg_len; i++)
    {
        checksum += buf[i+AS60X_FP_LEN_BIT];
    }
    checksum += buf[AS60X_FP_TOK_BIT];

    *(buf+AS60X_PREFIX_SIZE+pkg_len-2) = (checksum&0xff00)>>8;
    *(buf+AS60X_PREFIX_SIZE+pkg_len-1) = checksum&0x00ff;
}

static rt_size_t cnt_tx_pkg_size(void)
{
    rt_size_t result = 0;
    rt_enter_critical();
    rt_uint8_t size_tmp = tx_buf[AS60X_FP_LEN_BIT] << 8;
    result = size_tmp + tx_buf[AS60X_FP_LEN_BIT+1] + AS60X_PREFIX_SIZE;
    rt_exit_critical();

    return result;
}

static rt_size_t cnt_rx_pkg_size(void)
{
    rt_size_t result = 0;
    rt_enter_critical();
    rt_uint8_t size_tmp = rx_buf[AS60X_FP_LEN_BIT] << 8;
    result = size_tmp + rx_buf[AS60X_FP_LEN_BIT+1];
    rt_exit_critical();
    if (result > BUF_SIZE)
    {
        result = (rt_size_t)(BUF_SIZE - AS60X_PREFIX_SIZE);
        LOG_W("Next packet is out of buf size!");
    }

    return result;
}

#if DBG_LVL == DBG_LOG
static void print_buf(rt_uint8_t *buf, rt_size_t size)
{
    for (int i = 0; i < size; i++)
    {
        rt_kprintf("%x ", buf[i]);
    }
    rt_kprintf("\r\n");
}
#endif

static rt_err_t as60x_rx_handle(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_event_send(&event_fp, EVENT_AS60X_RX);

    return RT_EOK;
}

static void as60x_wak_handle(void *args)
{
//    rt_event_send(&event_fp, EVENT_AS60X_TOUCH);
    rt_event_send(evt_fp_p, evt_touch_set);
}

static rt_err_t as60x_hand_shake(void)
{
    // rt_uint8_t i;
    rt_err_t ret = RT_EOK;
    struct serial_configure as60x_cfg = RT_SERIAL_CONFIG_DEFAULT;
    // rt_uint32_t baud_table[12] = {BAUD_RATE_57600, BAUD_RATE_115200, BAUD_RATE_38400, BAUD_RATE_19200,
    //                              BAUD_RATE_9600, 105600, 96000, 86400,
    //                              76800, 67200, 48000, 28800};

    rt_device_open(as60x_dev, RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(as60x_dev, as60x_rx_handle);

    // for (i = 0; i < 12; i++) /* 使用 N*9600 的波特率进行握手测试 */
    // {
    //     as60x_cfg.baud_rate = baud_table[i%12];
    //     rt_device_control(as60x_dev, RT_DEVICE_CTRL_CONFIG, &as60x_cfg);
    //     // rt_device_open(as60x_dev, RT_DEVICE_FLAG_INT_RX);
    //     // rt_device_set_rx_indicate(as60x_dev, as60x_rx_handle);
    //     ret = fp_vfy_password();
    //     if (ret == -RT_ETIMEOUT)
    //     {
    //         LOG_I("Hand shake in %d timeout.", as60x_cfg.baud_rate);
    //         rt_device_close(as60x_dev);
    //     }
    //     else if (ret == -RT_EINVAL)
    //     {
    //         LOG_I("Recived PKG no vaild!");
    //     }
    //     else if (ret == -RT_ERROR)
    //     {
    //         LOG_E("Hand shake error!");
    //         rt_device_close(as60x_dev);
    //     }
    //     else if (ret == RT_EOK)
    //     {
    //         LOG_I("Establish connection in %d successfully!", as60x_cfg.baud_rate);
    //         return RT_EOK;
    //     }
    // }

    // as60x_cfg.baud_rate = baud_table[1];
    as60x_cfg.baud_rate = hs_baud;
    rt_device_control(as60x_dev, RT_DEVICE_CTRL_CONFIG, &as60x_cfg);
    ret = fp_vfy_password();
    if (ret == -RT_ETIMEOUT)
    {
        LOG_E("Hand shake in %d timeout.", as60x_cfg.baud_rate);
    }
    else if (ret == -RT_EINVAL)
    {
        LOG_E("Recived PKG no vaild!");
    }
    else if (ret == -RT_ERROR)
    {
        LOG_E("Hand shake error!");
    }
    else if (ret == RT_EOK)
    {
        LOG_I("Establish connection in %d successfully!", as60x_cfg.baud_rate);
        return RT_EOK;
    }
    rt_device_close(as60x_dev);

    ret = -RT_ERROR;
    return ret;    
}

static rt_err_t master_get_rx(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t rec = 0;
    rt_size_t size = 0;
    rt_uint8_t rx_cnt = 0;

    ret = rt_event_recv(&event_fp, EVENT_AS60X_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 1000, &rec);
    if (ret != RT_EOK)
        return ret;

    rt_memset(rx_buf, 0x00, BUF_SIZE);
    LOG_D("after clear rx_buf[0]=%x",rx_buf[0]);

    rt_thread_mdelay(10);
    do
    {
        rt_device_read(as60x_dev, -1, rx_buf, (rt_size_t)1);
        rx_cnt++;
    } while ((rx_buf[0] != 0xEF)&&(rx_cnt < RETRY_CNT)); /* 清除串口缓冲中的脏数据 */

    if (rx_cnt >= RETRY_CNT)
    {
        ret = -RT_ETIMEOUT;
        return ret;
    }

    rt_device_read(as60x_dev, -1, rx_buf+1, (rt_size_t)AS60X_PREFIX_SIZE-1); /* 获取包头 */
    LOG_D("after first read rx_buf[0]=%x",rx_buf[0]);
    LOG_D("after first read rx_buf[AS60X_FP_LEN_BIT+1]=%x",rx_buf[AS60X_FP_LEN_BIT+1]);

    rt_event_recv(&event_fp, EVENT_AS60X_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &rec);
    size = cnt_rx_pkg_size(); /* 计算包长度 */
    rt_device_read(as60x_dev, -1, rx_buf+AS60X_PREFIX_SIZE, size);  /* 获取包剩下数据 */
    rt_event_recv(&event_fp, EVENT_AS60X_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &rec);

#if DBG_LVL == DBG_LOG
    rt_kprintf("rx_size:%d rx: ", size+AS60X_PREFIX_SIZE);
    print_buf(rx_buf, size+AS60X_PREFIX_SIZE);
#endif

    return ret;
}

static rt_err_t make_prefix(const char *func)
{
    rt_err_t ret = -1;
    rt_size_t size = 0;

    tx_buf_add_checksum(tx_buf);
    size = cnt_tx_pkg_size();

#if DBG_LVL == DBG_LOG
    rt_kprintf("func:%s tx_size:%d tx: ", func, size);
    print_buf(tx_buf, size);
#endif

    rt_device_write(as60x_dev, 0, tx_buf, size);
    ret = master_get_rx();

    return ret;
}

static as60x_ack_type_t get_img_gen_ch(rt_uint8_t buff_id)
{
    as60x_ack_type_t code = AS60X_CMD_OK;
    rt_uint8_t retry_times = 0;

    while (retry_times < 5)
    {
        code = fp_get_image();
        if (code != AS60X_CMD_OK)
        {
            LOG_E("fp_get_image error!id=0x%x", (rt_uint16_t)code);
            rt_thread_mdelay(100);
            retry_times++;
        }
        else
            break;
    }

    code = fp_gen_char(buff_id);
    if (code != AS60X_CMD_OK)
    {
        LOG_E("fp_get_image error!id=0x%x", (rt_uint16_t)code);
    }
    return code;
}

void fp_wak_evt_reg(rt_event_t evt, rt_uint32_t set)
{
    if (evt == NULL) return;
    evt_fp_p = evt;
    evt_touch_set = set;
}

rt_err_t fp_vfy_password(void)
{
    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x07;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x13;
    tx_buf[AS60X_FP_INS_PAR_BIT(0)] = 0x00;
    tx_buf[AS60X_FP_INS_PAR_BIT(1)] = 0x00;
    tx_buf[AS60X_FP_INS_PAR_BIT(2)] = 0x00;
    tx_buf[AS60X_FP_INS_PAR_BIT(3)] = 0x00;

    rt_err_t ret = make_prefix(__func__);

    if (ret != RT_EOK)
        return ret;

    if (cnt_checksum(rx_buf) == 1)
    {
        if ((rx_buf[AS60X_FP_REP_ACK_BIT(0)] == 0x00) && (rx_buf[AS60X_FP_REP_ACK_BIT(1)] == 0x00))
        {
            flag_vfy = 1;
            ret = RT_EOK;
        }
    }
    else
        ret = -RT_EINVAL;

    return ret;
}

/**
 * @brief Get the image object
 *        获取传感器图像
 * 
 * @return as60x_ack_type_t 模块确认码
 */
as60x_ack_type_t fp_get_image(void)
{
    as60x_ack_type_t code = AS60X_CMD_OK;

    if (flag_vfy != 1)
    {
        LOG_E("Please verify the password before using the fingerprint module!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x03;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x01;

    rt_err_t ret = make_prefix(__func__);

    if (-RT_ETIMEOUT == ret)
    {
        LOG_E("Function fp_get_image timeout!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    if (cnt_checksum(rx_buf) == 1)
        code = (as60x_ack_type_t)rx_buf[AS60X_FP_REP_ACK_BIT(0)]; /* 校验和正确则返回模块确认码 */
    else
        code = AS60X_DAT_ERR;

    return code;
}

as60x_ack_type_t fp_gen_char(rt_uint8_t buff_id)
{
    if (buff_id < 0x01)
    {
        buff_id = 0x01;
        LOG_W("buff id out of range(0x01-0x02)!");
    }
    if (buff_id > 0x02)
    {
        buff_id = 0x02;
        LOG_W("buff id out of range(0x01-0x02)!");
    }

    as60x_ack_type_t code = AS60X_CMD_OK;

    if (flag_vfy != 1)
    {
        LOG_E("Please verify the password before using the fingerprint module!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x04;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x02;
    tx_buf[AS60X_FP_INS_PAR_BIT(0)] = buff_id;

    rt_err_t ret = make_prefix(__func__);

    if (-RT_ETIMEOUT == ret)
    {
        LOG_E("Function fp_gen_char timeout!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    if (cnt_checksum(rx_buf) == 1)
        code = (as60x_ack_type_t)rx_buf[AS60X_FP_REP_ACK_BIT(0)]; /* 校验和正确则返回模块确认码 */
    else
        code = AS60X_DAT_ERR;

    return code;
}

as60x_ack_type_t fp_search(rt_uint16_t *page_id, rt_uint16_t *mat_score)
{
    as60x_ack_type_t code = AS60X_CMD_OK;

    if (flag_vfy != 1)
    {
        LOG_E("Please verify the password before using the fingerprint module!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x08;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x04;
    tx_buf[AS60X_FP_INS_PAR_BIT(0)] = 0x01;
    tx_buf[AS60X_FP_INS_PAR_BIT(1)] = 0x00;
    tx_buf[AS60X_FP_INS_PAR_BIT(2)] = 0x00;
    tx_buf[AS60X_FP_INS_PAR_BIT(3)] = 0x01;
    tx_buf[AS60X_FP_INS_PAR_BIT(4)] = 0x2C;

    rt_err_t ret = make_prefix(__func__);

    if (-RT_ETIMEOUT == ret)
    {
        LOG_E("Function fp_search timeout!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    if (cnt_checksum(rx_buf) == 1)
        code = (as60x_ack_type_t)rx_buf[AS60X_FP_REP_ACK_BIT(0)]; /* 校验和正确则返回模块确认码 */
    else
    {
        code = AS60X_DAT_ERR;
        return code;
    }

    rt_uint8_t page_id_tmp = rx_buf[AS60X_FP_REP_ACK_BIT(1)] << 8;
    *page_id = page_id_tmp + rx_buf[AS60X_FP_REP_ACK_BIT(2)];
    rt_uint8_t mat_score_tmp = rx_buf[AS60X_FP_REP_ACK_BIT(3)] << 8;
    *mat_score = mat_score_tmp + rx_buf[AS60X_FP_REP_ACK_BIT(4)];
    LOG_D("*page_id=%lx, *mat_score=%lx", *page_id, *mat_score);

    return code;
}

as60x_ack_type_t fp_reg_model(void)
{
    as60x_ack_type_t code = AS60X_CMD_OK;

    if (flag_vfy != 1)
    {
        LOG_E("Please verify the password before using the fingerprint module!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x03;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x05;

    rt_err_t ret = make_prefix(__func__);

    if (-RT_ETIMEOUT == ret)
    {
        LOG_E("Function fp_gen_char timeout!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    if (cnt_checksum(rx_buf) == 1)
        code = (as60x_ack_type_t)rx_buf[AS60X_FP_REP_ACK_BIT(0)]; /* 校验和正确则返回模块确认码 */
    else
        code = AS60X_DAT_ERR;

    return code;
}

as60x_ack_type_t fp_str_char(rt_uint8_t buff_id, rt_uint16_t page_id)
{
    as60x_ack_type_t code = AS60X_CMD_OK;

    if (flag_vfy != 1)
    {
        LOG_E("Please verify the password before using the fingerprint module!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    if (buff_id < 0x01)
    {
        buff_id = 0x01;
        LOG_W("buff id out of range(0x01-0x02)!");
    }
    if (buff_id > 0x02)
    {
        buff_id = 0x02;
        LOG_W("buff id out of range(0x01-0x02)!");
    }

    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x06;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x06;
    tx_buf[AS60X_FP_INS_PAR_BIT(0)] = buff_id;
    tx_buf[AS60X_FP_INS_PAR_BIT(1)] = (rt_uint8_t)((page_id&0xff00)>>8);
    tx_buf[AS60X_FP_INS_PAR_BIT(2)] = (rt_uint8_t)(page_id&0x00ff);

    rt_err_t ret = make_prefix(__func__);

    if (-RT_ETIMEOUT == ret)
    {
        LOG_E("Function fp_search timeout!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    if (cnt_checksum(rx_buf) == 1)
        code = (as60x_ack_type_t)rx_buf[AS60X_FP_REP_ACK_BIT(0)]; /* 校验和正确则返回模块确认码 */
    else
        code = AS60X_DAT_ERR;

    return code;
}

void as60x_str_fp_to_flash(rt_uint16_t page_id)
{
    rt_uint32_t rec = 0;
    as60x_ack_type_t code = AS60X_CMD_OK;

    LOG_I("Wating for touch 1...");
//    rt_event_recv(&event_fp, EVENT_AS60X_TOUCH, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &rec);
    rt_event_recv(evt_fp_p, evt_touch_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &rec);
    LOG_D("detect touch signal!");
    code = get_img_gen_ch(0x01);

    LOG_I("Wating for touch 2...");
//    rt_event_recv(&event_fp, EVENT_AS60X_TOUCH, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &rec);
    rt_event_recv(evt_fp_p, evt_touch_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &rec);
    LOG_D("detect touch signal!");
    code = get_img_gen_ch(0x02);

    code = fp_reg_model();
    if (code == AS60X_MERGE_FAIL)
    {
        LOG_E("Two fingerprints do not belong to the same finger! Please try again.");
        return;
    }

    code = fp_str_char(0x02, page_id);
    if (code != AS60X_CMD_OK)
    {
        if (AS60X_OVER_SIZE == code)
        {
            LOG_E("PageID is out of the fingerprint library!");
            return;
        }
        else if (AS60X_RW_FLASH_ERR == code)
        {
            LOG_E("Write FLASH error!");
            return;
        }
    }
    else
        LOG_I("Fingerprint store successful!");
}

void as60x_str_fp_to_flash_test(void)
{
    LOG_I("store finger print in 0x02");
    as60x_str_fp_to_flash(0x02);
}
MSH_CMD_EXPORT(as60x_str_fp_to_flash_test, "store fingerprint in flash");

as60x_ack_type_t as60x_search_fp_in_flash(rt_uint16_t *page_id, rt_uint16_t *mat_score)
{
    as60x_ack_type_t code = AS60X_CMD_OK;

    rt_kprintf("Wating for touch 1...\r\n");
//    rt_event_recv(&event_fp, EVENT_AS60X_TOUCH, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, RT_NULL);
    rt_event_recv(evt_fp_p, evt_touch_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, RT_NULL);
    LOG_D("detect touch signal!");
    code = get_img_gen_ch(0x01);

    code = fp_search(page_id, mat_score);
    LOG_I("Find the fingerprint! page_id=%lx, mat_score=%lx", *page_id, *mat_score);

    return code;
}
MSH_CMD_EXPORT(as60x_search_fp_in_flash, "find the fingerprint");

as60x_ack_type_t as60x_delet_fp_n_id(rt_uint16_t page_id, rt_uint16_t n)
{
    as60x_ack_type_t code = AS60X_CMD_OK;

    if (flag_vfy != 1)
    {
        LOG_E("Please verify the password before using the fingerprint module!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x07;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x0C;
    tx_buf[AS60X_FP_INS_PAR_BIT(0)] = (rt_uint8_t)((page_id&0xff00)>>8);
    tx_buf[AS60X_FP_INS_PAR_BIT(1)] = (rt_uint8_t)(page_id&0x00ff);
    tx_buf[AS60X_FP_INS_PAR_BIT(2)] = (rt_uint8_t)((n&0xff00)>>8);
    tx_buf[AS60X_FP_INS_PAR_BIT(3)] = (rt_uint8_t)(n&0x00ff);

    rt_err_t ret = make_prefix(__func__);
    
    if (-RT_ETIMEOUT == ret)
    {
        LOG_E("Function as60x_delet_fp_n_id timeout!");
        code = AS60X_UNDEF_ERR;
        return code;
    }

    if (cnt_checksum(rx_buf) == 1)
        code = (as60x_ack_type_t)rx_buf[AS60X_FP_REP_ACK_BIT(0)]; /* 校验和正确则返回模块确认码 */
    else
        code = AS60X_DAT_ERR;

    return code;
}

void as60x_set_hand_shake_baud(rt_uint32_t baud)
{
    hs_baud = baud;
}

void as60x_init(const char *name)
{
    rt_err_t ret = RT_EOK;
    if (flag_init == 1)
        goto _END;
    flag_init = 1;

    /* 查找系统中的串口设备 */
    as60x_dev = rt_device_find(name);
    if (!as60x_dev)
    {
        LOG_E("find %s failed!\n", name);
    }

    rt_pin_mode(PKG_AS608_WAK_PIN, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_attach_irq(PKG_AS608_WAK_PIN, PIN_IRQ_MODE_RISING, as60x_wak_handle, RT_NULL);
    rt_pin_irq_enable(PKG_AS608_WAK_PIN, PIN_IRQ_ENABLE);

    ret = rt_event_init(&event_fp, "ent-fp", RT_IPC_FLAG_FIFO);
    if (ret != RT_EOK)
        LOG_E("event fp creat error!");

    as60x_hand_shake();
_END:
    ;
}

