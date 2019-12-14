/**
 * @author greedyhao (hao_kr@163.com)
 * @version 1.0.1
 * @date 2019-11-24
 *  
 */

#ifndef _AS608_H
#define _AS608_H

#include "rtthread.h"
#include <rtdevice.h>
#include <rtdef.h>

#define AS60X_HEAD_SIZE         2U
#define AS60X_PREFIX_SIZE       9U

#define AS60X_FP_HEAD_BIT       0U  /* 包头位 */
#define AS60X_FP_ADDR_BIT       2U  /* 地址位 */
#define AS60X_FP_TOK_BIT        6U  /* 标识位 */
#define AS60X_FP_LEN_BIT        7U  /* 长度位 */

#define AS60X_FP_INS_CMD_BIT    9U  /* 指令包指令码位 */
#define AS60X_FP_INS_PAR_BIT(n) ((n)+10U) /* 指令包参数位 */
#define AS60X_FP_INS_SUM_BIT(n) AS60X_FP_INS_PAR_BIT(n) /* 指令包校验和位 */

#define AS60X_FP_REP_ACK_BIT(n) ((n)+9U)  /* 应答包确认码位 */
#define AS60X_FP_REP_SUM_BIT(n) AS60X_FP_REP_ACK_BIT(n) /* 应答包校验和位 */

#define AS60X_FP_HEAD_H     0xEF    /* 包头为 0xEF01 */
#define AS60X_FP_HEAD_L     0x01
#define AS60X_FP_ADDR_0     0xFF    /* 默认地址为 0xFFFFFFFF */
#define AS60X_FP_ADDR_1     0xFF
#define AS60X_FP_ADDR_2     0xFF
#define AS60X_FP_ADDR_3     0xFF
#define AS60X_FP_TOK_CMD    0x01
#define AS60X_FP_TOK_DAT    0x02
#define AS60X_FP_TOK_END    0x08

/* AS60x 系列命令列表 */
#define AS60X_FP_CMD_GET_IMG    0x01 /* 从传感器上读入图像存于图像缓冲区 */
#define AS60X_FP_CMD_GEN_CHR    0x02 /* 根据原始图像生成指纹特征存于 CharBuffer1 或 CharBuffer2 */
#define AS60X_FP_CMD_MATCH      0x03 /* 精确比对 CharBuffer1 与 CharBuffer2 中的特征文件 */
#define AS60X_FP_CMD_SEARCH     0x04 /* 以 CharBuffer1 或 CharBuffer2 中的特征文件搜索整个或部分指纹库 */
#define AS60X_FP_CMD_REG_MODEL  0x05 /* 将 CharBuffer1 与 CharBuffer2 中的特征文件合并生成模板存于 CharBuffer2 */
#define AS60X_FP_CMD_STR_CHR    0x06 /* 将特征缓冲区中的文件储存到 flash 指纹库中 */
#define AS60X_FP_CMD_LDR_CHR    0x07 /* 从 flash 指纹库中读取一个模板到特征缓冲区 */
#define AS60X_FP_CMD_UP_CHR     0x08 /* 将特征缓冲区中的文件上传给上位机 */
#define AS60X_FP_CMD_DOWN_CHR   0x09 /* 从上位机下载一个特征文件到特征缓冲区 */
#define AS60X_FP_CMD_UP_IMG     0x0A /* 上传原始图像 */
#define AS60X_FP_CMD_DOWN_IMG   0x0B /* 下载原始图像 */
#define AS60X_FP_CMD_DEL_CHR    0x0C /* 删除 flash 指纹库中的一个特征文件 */
#define AS60X_FP_CMD_EMPTY      0x0D /* 清空 flash 指纹库 */
#define AS60X_FP_CMD_WR_REG     0x0E /* 写 SOC 系统寄存器 */
#define AS60X_FP_CMD_RD_PARAM   0x0F /* 读系统基本参数 */
#define AS60X_FP_CMD_ENROLL     0x10 /* 注册模板 */
#define AS60X_FP_CMD_IDENTIFY   0x11 /* 验证指纹 */
#define AS60X_FP_CMD_SET_PWD    0x12 /* 设置设备握手口令 */
#define AS60X_FP_CMD_VFY_PWD    0x13 /* 验证设备握手口令 */
#define AS60X_FP_CMD_GET_RAMD   0x14 /* 采样随机数 */
#define AS60X_FP_CMD_SET_ADDR   0x15 /* 设置芯片地址 */
#define AS60X_FP_CMD_RD_INF_PAG 0x16 /* 读取 FLASH Information Page 内容 */
#define AS60X_FP_CMD_PORT_CNTL  0x17 /* 通讯端口（UART/USB）开关控制 */
#define AS60X_FP_CMD_WR_NOTEPAD 0x18 /* 写记事本 */
#define AS60X_FP_CMD_RD_NOTEPAD 0x19 /* 读记事本 */
#define AS60X_FP_CMD_BURN_CODE  0x1A /* 烧写片内 FLASH */
#define AS60X_FP_CMD_HS_SEARCH  0x1B /* 高速搜索 FLASH */
#define AS60X_FP_CMD_GEN_BIN_IM 0x1C /* 生成二值化指纹图像 */

/**
 * @brief 定义了指令应答的确认码
 * 
 */
enum as60x_ack_type
{
    AS60X_CMD_OK        = 0x00, /* 表示指令执行完毕或 OK */
    AS60X_DAT_ERR       = 0x01, /* 表示数据包接收错误 */
    AS60X_NO_FINGER     = 0x02, /* 表示传感器上没有手指 */
    AS60X_INPUT_FIN     = 0x03, /* 表示录入指纹图像失败 */
    AS60X_TOO_DRY       = 0x04, /* 表示指纹图像太干、太淡而生不成特征 */
    AS60X_TOO_WET       = 0x05, /* 表示指纹图像太湿、太糊而生不成特征 */
    AS60X_TOO_MESS      = 0x06, /* 表示指纹图像太乱而生不成特征 */
    AS60X_FEATURE_PT    = 0x07, /* 表示指纹图像正常，但特征点太少（或面积太小）而生不成特征 */
    AS60X_FINGER_MIS    = 0x08, /* 表示指纹不匹配 */
    AS60X_SERCH_ERR     = 0x09, /* 表示没搜索到指纹 */
    AS60X_MERGE_FAIL    = 0x0A, /* 表示特征合并失败 */
    AS60X_OVER_SIZE     = 0x0B, /* 表示访问指纹库时地址序号超出指纹库范围 */
    AS60X_RD_TMPL_ERR   = 0x0C, /* 表示从指纹库读模板出错或无效 */
    AS60X_UP_FEAT_ERR   = 0x0D, /* 表示上传特征失败 */
    AS60X_NO_SUBSEQ_PKG = 0x0E, /* 表示模块不能接受后续数据包 */
    AS60X_UP_IMG_ERR    = 0x0F, /* 表示上传图像失败 */
    AS60X_DEL_TMPL_ERR  = 0x10, /* 表示删除模板失败 */
    AS60X_CLR_FP_ERR    = 0x11, /* 表示清空指纹库失败 */
    AS60X_LP_MODE_ERR   = 0x12, /* 表示不能进入低功耗状态 */
    AS60X_PWD_ERR       = 0x13, /* 表示口令不正确 */
    AS60X_SYS_RST_ERR   = 0x14, /* 表示系统复位失败 */
    AS60X_BUF_NO_IMG    = 0x15, /* 表示缓冲区内没有有效原始图而生不成图像 */
    AS60X_OTA_ERR       = 0x16, /* 表示在线升级失败 */
    AS60X_RESIDUAL_FP   = 0x17, /* 表示残留指纹或两次采集之间手指没有移动过 */
    AS60X_RW_FLASH_ERR  = 0x18, /* 表示读写 FLASH 出错 */
    AS60X_UNDEF_ERR     = 0x19, /* 未定义错误 */
    AS60X_INVALID_REG   = 0x1A, /* 无效寄存器号 */
    AS60X_CFG_REG_ERR   = 0x1B, /* 寄存器设定内容错误号 */
    AS60X_NP_PAGE_ERR   = 0x1C, /* 记事本页码指定错误 */
    AS60X_PORT_CNTL_ERR = 0x1D, /* 端口操作失败 */
    AS60X_ENROLL_ERR    = 0x1E, /* 自动注册（enroll）失败 */
    AS60X_FP_FULL       = 0x1F, /* 指纹库满 */
    /* 0x20—0xefh：Reserved */
    AS60X_SUBSEQ_OK     = 0xF0, /* 有后续数据包的指令，正确接收后用 0xf0 应答 */
    AS60X_SUBSEQ_CMD    = 0xF1, /* 有后续数据包的指令，命令包用 0xf1 应答 */
    AS60X_BURN_SUM_ERR  = 0xF2, /* 表示烧写内部 FLASH 时，校验和错误 */
    AS60X_BURN_TOK_ERR  = 0xF3, /* 表示烧写内部 FLASH 时，包标识错误 */
    AS60X_BURN_LEN_ERR  = 0xF4, /* 表示烧写内部 FLASH 时，包长度错误 */
    AS60X_BURN_COD_EXSZ = 0xF5, /* 表示烧写内部 FLASH 时，代码长度太长 */
    AS60X_BURN_FUNC_ERR = 0xF6, /* 表示烧写内部 FLASH 时，烧写 FLASH 失败 */ 
};

typedef enum as60x_ack_type as60x_ack_type_t;

/**
 * @brief Get the sensor type object
 *        获取传感器驱动类型
 * 
 * @param param 
 * @return rt_err_t 
 */
rt_err_t get_sensor_type(void *param);      // TODO

rt_err_t get_database_size(void *param);    // TODO

rt_err_t get_secure_level(void *param);     // TODO

rt_err_t set_device_addr(void *param);      // TODO
rt_err_t get_device_addr(void *param);      // TODO

rt_err_t set_packet_size(void *param);      // TODO
rt_err_t get_packet_size(void *param);      // TODO

rt_err_t set_baud_rate(void *param);        // TODO
rt_err_t get_baud_rate(void *param);        // TODO

rt_err_t get_usb_vid(void *param);          // TODO
rt_err_t get_usb_pid(void *param);          // TODO

rt_err_t get_product_sn(void *param);       // TODO
rt_err_t get_software_ver(void *param);     // TODO
rt_err_t get_manufacturer(void *param);     // TODO

rt_err_t get_sensor_name(void *param);      // TODO

/**
 * @brief Set the password object
 * 
 * @param param 
 * @return rt_err_t 
 * @note 最好别改模块这个握手密码
 */
rt_err_t set_password(void *param);         // TODO

/**
 * @brief verify password
 *        验证密码
 * 
 * @param param 
 * @return rt_err_t 
 */
rt_err_t fp_vfy_password(void);

/**
 * @brief Get the image object
 *        获取传感器图像
 * 
 * @return as60x_ack_type_t 模块确认码
 */
as60x_ack_type_t fp_get_image(void);

/**
 * @brief 将 ImageBuffer 中的原始图像生成指纹特征文件存于 CharBuffer1 或 CharBuffer2
 * 
 * @param buff_id 
 * @return as60x_ack_type_t 
 */
as60x_ack_type_t fp_gen_char(rt_uint8_t buff_id);

/**
 * @brief  以 CharBuffer1 或 CharBuffer2 中的特征文件搜索整个或部分指纹库。若搜索到，则返回页码
 * 
 * @return as60x_ack_type_t 
 */
as60x_ack_type_t fp_search(rt_uint16_t *page_id, rt_uint16_t *mat_score);

/**
 * @brief  将 CharBuffer1 与 CharBuffer2 中的特征文件合并生成模板，结果存于 CharBuffer1 与 CharBuffer2
 * 
 * @return as60x_ack_type_t 
 */
as60x_ack_type_t fp_reg_model(void);

/**
 * @brief  将 CharBuffer1 或 CharBuffer2 中的模板文件存到 PageID 号 flash 数据库位置
 * 
 * @param buff_id 需要存储的 CharBuffer 号
 * @param page_id 存储位置
 * @return as60x_ack_type_t 
 */
as60x_ack_type_t fp_str_char(rt_uint8_t buff_id, rt_uint16_t page_id);

/* 上层 API */

/**
 * @brief 模块初始化函数
 * 
 * @param name 
 */
void as60x_init(const char *name);

/**
 * @brief 设置通信使用的波特率，未来版本会弃用
 * 
 * @param baud 
 * @note 模块初始化前必须先设置波特率
 */
void as60x_set_hand_shake_baud(rt_uint32_t baud);

/**
 * @brief 指纹录入函数
 * 
 * @param page_id 指纹存储页
 */
void as60x_str_fp_to_flash(rt_uint16_t page_id);

/**
 * @brief 指纹搜索函数
 * 
 */
void as60x_search_fp_in_flash(void);

#endif
