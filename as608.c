/**
 * @author greedyhao (hao_kr@163.com)
 * @version 0.0.1
 * @date 2019-11-24
 *  
 */

#include "as608.h"
#include <dfs_posix.h>

#define DBG_TAG              "fp.dev"
// #define DBG_LVL              DBG_INFO
#define DBG_LVL              DBG_LOG
#include <rtdbg.h>

#define BUF_SIZE    50U
static rt_uint8_t tx_buf[BUF_SIZE] = {AS60X_FP_HEAD_H, AS60X_FP_HEAD_L, AS60X_FP_ADDR_0,
                                AS60X_FP_ADDR_1, AS60X_FP_ADDR_2, AS60X_FP_ADDR_3};
static rt_uint8_t rx_buf[BUF_SIZE];

#define EVENT_AS60X_RX      (1 << 0)
#define EVENT_AS60X_TOUCH   (1 << 1)
static struct rt_event event_fp;
static rt_device_t as60x_dev;
static rt_uint8_t flag_vfy = 0; /* 握手成功标志 */

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

    for (i = 0; i < pkg_len; i++)
    {
        checksum += buf[i+AS60X_FP_LEN_BIT];
    }
    checksum += buf[AS60X_FP_TOK_BIT];

    if ((*(buf+AS60X_PREFIX_SIZE+pkg_len-2) == checksum&0xff00)
    && (*(buf+AS60X_PREFIX_SIZE+pkg_len-1) == checksum&0x00ff))
        checksum_is_ok = 1;

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

    *(buf+AS60X_PREFIX_SIZE+pkg_len-2) = checksum&0xff00;
    *(buf+AS60X_PREFIX_SIZE+pkg_len-1) = checksum&0x00ff;
}

static void cnt_tx_pkg_size(rt_size_t *size)
{
    rt_enter_critical();
    rt_uint8_t size_tmp = tx_buf[AS60X_FP_LEN_BIT] << 8;
    *size = size_tmp + tx_buf[AS60X_FP_LEN_BIT+1] + AS60X_PREFIX_SIZE;
    rt_exit_critical();
}

static void cnt_rx_pkg_size(rt_size_t *size)
{
    rt_enter_critical();
    rt_uint8_t size_tmp = rx_buf[AS60X_FP_LEN_BIT] << 8;
    *size = size_tmp + rx_buf[AS60X_FP_LEN_BIT+1];
    rt_exit_critical();
    if (*size > BUF_SIZE)
    {
        *size = (rt_size_t)(BUF_SIZE - AS60X_PREFIX_SIZE);
        LOG_W("Next packet is out of buf size!");
    }
}

static rt_err_t as60x_rx(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_event_send(&event_fp, EVENT_AS60X_RX);

    return RT_EOK;
}
MSH_CMD_EXPORT(as60x_rx,"as60x_rx");

static rt_err_t as60x_hand_shake(void)
{
    rt_uint8_t i;
    rt_err_t ret = RT_EOK;
    struct serial_configure as60x_cfg = RT_SERIAL_CONFIG_DEFAULT;
    rt_uint32_t baud_table[12] = {BAUD_RATE_57600, BAUD_RATE_115200, BAUD_RATE_38400, BAUD_RATE_19200,
                                 BAUD_RATE_9600, 105600, 96000, 86400,
                                 76800, 67200, 48000, 28800};

    rt_device_open(as60x_dev, RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(as60x_dev, as60x_rx);

    // for (i = 0; i < 12; i++) /* 使用 N*9600 的波特率进行握手测试 */
    // {
    //     as60x_cfg.baud_rate = baud_table[i%12];
    //     rt_device_control(as60x_dev, RT_DEVICE_CTRL_CONFIG, &as60x_cfg);
    //     // rt_device_open(as60x_dev, RT_DEVICE_FLAG_INT_RX);
    //     // rt_device_set_rx_indicate(as60x_dev, as60x_rx);
    //     ret = vfy_password();
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

    as60x_cfg.baud_rate = baud_table[1];
    rt_device_control(as60x_dev, RT_DEVICE_CTRL_CONFIG, &as60x_cfg);
    ret = vfy_password();
    if (ret == -RT_ETIMEOUT)
    {
        LOG_I("Hand shake in %d timeout.", as60x_cfg.baud_rate);
        rt_device_close(as60x_dev);
    }
    else if (ret == -RT_EINVAL)
    {
        LOG_I("Recived PKG no vaild!");
    }
    else if (ret == -RT_ERROR)
    {
        LOG_E("Hand shake error!");
        rt_device_close(as60x_dev);
    }
    else if (ret == RT_EOK)
    {
        LOG_I("Establish connection in %d successfully!", as60x_cfg.baud_rate);
        return RT_EOK;
    }

    ret = -RT_ERROR;
    return ret;    
}
MSH_CMD_EXPORT(as60x_hand_shake, "as60x hand shake");

// static void as60x_thread_entry(void *param)
// {
//     while (1)
//     {
        
//     }
    
// }

static rt_err_t master_get_rx(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t rec = 0;
    rt_size_t size = 0;
    rt_uint8_t rx_cnt = 0;

    ret = rt_event_recv(&event_fp, EVENT_AS60X_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 1000, &rec);
    if (ret != RT_EOK)
        return ret;

    memset(rx_buf, 0x00, BUF_SIZE);
    LOG_D("after clear rx_buf[0]=%x",rx_buf[0]);

    do
    {
        rt_device_read(as60x_dev, -1, rx_buf, (rt_size_t)1);
        rx_cnt++;
    } while ((rx_buf[0] != 0xEF)&&(rx_cnt < 20)); /* 清除串口缓冲中的脏数据 */

    if (rx_cnt >= 20)
    {
        ret = -RT_ETIMEOUT;
        return ret;
    }

    rt_device_read(as60x_dev, -1, rx_buf+1, (rt_size_t)AS60X_PREFIX_SIZE-1); /* 获取包头 */
    LOG_D("after first read rx_buf[0]=%x",rx_buf[0]);
    LOG_D("after first read rx_buf[AS60X_FP_LEN_BIT+1]=%x",rx_buf[AS60X_FP_LEN_BIT+1]);

    rt_event_recv(&event_fp, EVENT_AS60X_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &rec);
    cnt_rx_pkg_size(&size); /* 计算包长度 */
    rt_device_read(as60x_dev, -1, rx_buf+AS60X_PREFIX_SIZE, size);  /* 获取包剩下数据 */
    rt_event_recv(&event_fp, EVENT_AS60X_RX, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &rec);

#if DBG_LVL == DBG_LOG
    rt_kprintf("rx_size:%d rx: ", size+AS60X_PREFIX_SIZE);
    for (int i = 0; i < size+AS60X_PREFIX_SIZE; i++)
    {
        rt_kprintf("%x ", rx_buf[i]);
    }
    rt_kprintf("\r\n");
#endif

    return ret;
}

rt_err_t vfy_password(void)
{
    rt_err_t ret = -1;
    rt_size_t size = 0;
    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x07;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x13;
    tx_buf[AS60X_FP_INS_PAR_BIT(0)] = 0x00;
    tx_buf[AS60X_FP_INS_PAR_BIT(1)] = 0x00;
    tx_buf[AS60X_FP_INS_PAR_BIT(2)] = 0x00;
    tx_buf[AS60X_FP_INS_PAR_BIT(3)] = 0x00;
    tx_buf_add_checksum(tx_buf);
    cnt_tx_pkg_size(&size);

#if DBG_LVL == DBG_LOG
    rt_kprintf("func:%s tx_size:%d tx: ", __func__, size);
    for (int i = 0; i < size; i++)
    {
        rt_kprintf("%x ", tx_buf[i]);
    }
    rt_kprintf("\r\n");
#endif

    rt_device_write(as60x_dev, 0, tx_buf, size);
    ret = master_get_rx();
    if (ret != RT_EOK)
        return ret;

    if ((rx_buf[AS60X_FP_HEAD_BIT] == AS60X_FP_HEAD_H) && (rx_buf[AS60X_FP_HEAD_BIT+1] == AS60X_FP_HEAD_L))
    {
        if ((rx_buf[AS60X_FP_REP_ACK_BIT(0)] == 0x00) && (rx_buf[AS60X_FP_REP_ACK_BIT(1)] == 0x00))
        {
            flag_vfy = 1;
            ret = RT_EOK;
        }
    }
    else
    {
        ret = -RT_EINVAL;
        return ret;
    }

    return ret;
}

/**
 * @brief Get the image object
 *        获取传感器图像
 * 
 * @return as60x_ack_type_t 模块确认码
 */
as60x_ack_type_t get_image(void)
{
    as60x_ack_type_t code = AS60X_CMD_OK;
    rt_err_t ret = -1;
    rt_size_t size = 0;

    if (flag_vfy != 1)
    {
        LOG_E("Please verify the password before using the fingerprint module!");
        code = AS60X_UNDEF_ERR;
        goto _exit;
    }

    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x03;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x01;
    tx_buf_add_checksum(tx_buf);
    cnt_tx_pkg_size(&size);

#if DBG_LVL == DBG_LOG
    rt_kprintf("func:%s tx_size:%d tx: ", __func__, size);
    for (int i = 0; i < size; i++)
    {
        rt_kprintf("%x ", tx_buf[i]);
    }
    rt_kprintf("\r\n");
#endif

    rt_device_write(as60x_dev, 0, tx_buf, size);
    ret = master_get_rx();
    if (-RT_ETIMEOUT == ret)
    {
        LOG_E("Function get_image timeout!");
        code = AS60X_UNDEF_ERR;
        goto _exit;
    }

    if ((rx_buf[AS60X_FP_HEAD_BIT] == AS60X_FP_HEAD_H) && (rx_buf[AS60X_FP_HEAD_BIT+1] == AS60X_FP_HEAD_L))
    {
        if (cnt_checksum(rx_buf) != 1)
            code = AS60X_DAT_ERR;
        else
            code = (as60x_ack_type_t)rx_buf[AS60X_FP_REP_ACK_BIT(0)]; /* 校验和正确则返回模块确认码 */
    }
    else
    {
        code = AS60X_DAT_ERR;
        return ret;
    }

_exit:
    return code;
}
MSH_CMD_EXPORT(get_image, "get image");

as60x_ack_type_t img_gen_char(void)
{
    as60x_ack_type_t code = AS60X_CMD_OK;
    rt_err_t ret = -1;
    rt_size_t size = 0;

    if (flag_vfy != 1)
    {
        LOG_E("Please verify the password before using the fingerprint module!");
        code = AS60X_UNDEF_ERR;
        goto _exit;
    }

    tx_buf[AS60X_FP_TOK_BIT] = 0x01;
    tx_buf[AS60X_FP_LEN_BIT] = 0x00;
    tx_buf[AS60X_FP_LEN_BIT+1] = 0x04;
    tx_buf[AS60X_FP_INS_CMD_BIT] = 0x02;
    tx_buf[AS60X_FP_INS_PAR_BIT(0)] = 0x01;
    tx_buf_add_checksum(tx_buf);
    cnt_tx_pkg_size(&size);

#if DBG_LVL == DBG_LOG
    rt_kprintf("func:%s tx_size:%d tx: ", __func__, size);
    for (int i = 0; i < size; i++)
    {
        rt_kprintf("%x ", tx_buf[i]);
    }
    rt_kprintf("\r\n");
#endif

    rt_device_write(as60x_dev, 0, tx_buf, size);
    ret = master_get_rx();
    if (-RT_ETIMEOUT == ret)
    {
        LOG_E("Function get_image timeout!");
        code = AS60X_UNDEF_ERR;
        goto _exit;
    }

    if ((rx_buf[AS60X_FP_HEAD_BIT] == AS60X_FP_HEAD_H) && (rx_buf[AS60X_FP_HEAD_BIT+1] == AS60X_FP_HEAD_L))
    {
        if (cnt_checksum(rx_buf) != 1)
            code = AS60X_DAT_ERR;
        else
            code = (as60x_ack_type_t)rx_buf[AS60X_FP_REP_ACK_BIT(0)]; /* 校验和正确则返回模块确认码 */
    }
    else
    {
        code = AS60X_DAT_ERR;
        return ret;
    }

_exit:
    return code;
}
MSH_CMD_EXPORT(img_gen_char, "image generate char");

void as60x_init(const char *name)
{
    rt_err_t ret = RT_EOK;
    /* 查找系统中的串口设备 */
    as60x_dev = rt_device_find(name);
    if (!as60x_dev)
    {
        LOG_E("find %s failed!\n", name);
    }

    ret = rt_event_init(&event_fp, "ent-fp", RT_IPC_FLAG_FIFO);
    // event_fp = rt_event_create("ent-fp", RT_IPC_FLAG_FIFO);
    // if (event_fp == RT_NULL)
    if (ret != RT_EOK)
        LOG_E("event fp creat error!");

    // rt_thread_t thread = rt_thread_create("as60x", as60x_thread_entry, RT_NULL, 1024, 25, 10);
    // if (thread != RT_NULL)
    // {
    //     rt_thread_startup(thread);
    // }
    // else
    // {
    //     ret = RT_ERROR;
    // }

    as60x_hand_shake();
}

