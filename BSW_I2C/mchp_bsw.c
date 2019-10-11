/*
 *  mchp_bsw.c - microchip BSW Touch Sense Controller
 *
 *  Copyright (C) 2009
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/firmware.h>

#include "tiny1617_fw_v25.h"

#define MCHP_BSW_CHIPID          0x??
#define MCHP_BSW_WAKEUP_TIME         25

#define MCHP_BSW_CMD_CHIPID     0
#define MCHP_BSW_CMD_CODEVER    1
#define MCHP_BSW_CMD_SUBVER     7
#define MCHP_BSW_CMD_GSTAT      2
#define MCHP_BSW_CMD_KEYS3      3
#define MCHP_BSW_CMD_KEYS4      4
#define MCHP_BSW_CMD_SLIDE      5
#define MCHP_BSW_CMD_SUFCE      6
#define MCHP_BSW_CALIBRATE      10

#define MCHP_BSW_IRQ_TYPE       IRQF_TRIGGER_FALLING

#define  MCHP_BSW_INFO            (1<<0)
#define  MCHP_BSW_DEBUG            (1<<1)
#define  MCHP_BSW_ERROR            (1<<2)
#define  MCHP_BSW_WARN            (1<<3)

static uint8_t bsw_log_level = MCHP_BSW_INFO | MCHP_BSW_ERROR | MCHP_BSW_WARN | MCHP_BSW_DEBUG;

#define MCHP_BSW_LOG_INFO(fmt, arg...)\
do{\
        printk("mchp bsw info %s:%d:  " fmt "\n",__FUNCTION__, __LINE__, ##arg);\
}while(0)
    
#define MCHP_BSW_LOG_DBG(fmt, arg...)\
do{\
    if(bsw_log_level&MCHP_BSW_DEBUG)\
        printk("mchp bsw debug %s:%d:  " fmt "\n",__FUNCTION__, __LINE__, ##arg);\
}while(0)
    
#define MCHP_BSW_LOG_ERR(fmt, arg...)\
do{\
    if(bsw_log_level&MCHP_BSW_ERROR)\
        printk("mchp bsw error %s:%d:  " fmt "\n",__FUNCTION__, __LINE__, ##arg);\
}while(0)
    
#define MCHP_BSW_LOG_WARN(fmt, arg...)\
do{\
    if(bsw_log_level&MCHP_BSW_WARN)\
        printk("mchp bsw warning %s:%d:  " fmt "\n",__FUNCTION__, __LINE__, ##arg);\
}while(0)
    
struct mchp_bsw_debug
{
    bool flag;
    uint8_t     slave_addr;
    uint16_t    reg_addr;
    uint16_t    len;
    uint8_t     frame;
};

struct mchp_bsw_data
{
    struct i2c_client *client;
    struct input_dev *input;
    struct mutex i2c_lock;
    struct mchp_bsw_debug debug;
    uint32_t keystatus;
    uint8_t fw_version;
    uint8_t chip_id;
};


/*=============================================
iic read/write
in normal and boot loader mode, the salve address and reg address len may be different
=============================================*/
#define BOOT_LOADER_ADDR            0x26
#define BOOT_LOADER_REG_ADDR_LEN    0x2
/*===================================
iic read in bootloader
client
reg
len
*val
===================================*/
static int mchp_bsw_read_in_bootloader(struct i2c_client *client, u16 reg, u16 len, void *val)
{
    struct i2c_msg xfer[2];
    u8 buf[2];
    int ret,i;
    bool retry = false;
    
    //dev_warn(&client->dev,"i2c read: ID: %x; Reg: %x; Len: %x", client->addr, reg, len);

#if (BOOT_LOADER_REG_ADDR_LEN == 1)
    buf[0] = reg & 0xff;
#else
    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;
#endif

    /* Write register */
    xfer[0].addr = BOOT_LOADER_ADDR;
    xfer[0].flags = 0;
    xfer[0].len = BOOT_LOADER_REG_ADDR_LEN;
    xfer[0].buf = buf;

    /* Read data */
    xfer[1].addr = BOOT_LOADER_ADDR;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = len;
    xfer[1].buf = val;
    
    MCHP_BSW_LOG_DBG("i2c read: ID: %x; Reg: %x; Len: %x \n", xfer[0].addr, reg, len);

retry_read:
    ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
    if (ret != ARRAY_SIZE(xfer)) {
        if (!retry) {
            dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
            msleep(MCHP_BSW_WAKEUP_TIME);
            retry = true;
            //retry = false;
            
            goto retry_read;
        } else {
            dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
                __func__, ret);
            return -EIO;
        }
    }
    else
    {
        ret = 0;
    }
    
    if(bsw_log_level&MCHP_BSW_DEBUG)
    {
        for(i = 0; i < len; i++)
        {
            printk("0x%2x,", *(uint8_t *)(val+i));
        }
        printk("\n");
    }
    
    return ret;
}


/*===================================
iic write in bootloader
client
reg
len
&val
===================================*/
static int mchp_bsw_write_in_bootloader(struct i2c_client *client, u16 reg, u16 len, const void *val)
{
    
    u8 *buf;
    size_t count;
    int ret,i;
    struct i2c_msg msg;
    bool retry = false;
    

#if (BOOT_LOADER_REG_ADDR_LEN == 1)
    count = len + 1;
#else
    count = len + 2;
#endif
    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
    {
        MCHP_BSW_LOG_ERR("malloc buf failed");
        return -ENOMEM;
    }


#if (BOOT_LOADER_REG_ADDR_LEN == 1)
    buf[0] = reg & 0xff;
    if(count > 1)
        memcpy(&buf[1], val, len);
#else
    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;
    if(count > 2)
        memcpy(&buf[2], val, len);
#endif
        
    //write address 
    msg.addr = BOOT_LOADER_ADDR;
    msg.flags = 0;
    msg.len = count;
    msg.buf = (u8 *)buf;
    
    MCHP_BSW_LOG_DBG("i2c write: ID: %x; Reg: %x; Len: %x \n", msg.addr, reg, len);
    
retry_write:
    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret != 1) {
        if (!retry) {
            dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
            msleep(MCHP_BSW_WAKEUP_TIME);
            retry = true;
            goto retry_write;
        } else {
            dev_err(&client->dev, "%s: i2c send failed (%d)\n",
                __func__, ret);
            ret = -EIO;
        }
    } else {
        ret = 0;
    }    
    
    if(bsw_log_level&MCHP_BSW_DEBUG)
    {
            
        if(count > 2)
        {
            for(i = 2; i < count; i++)
            {
                printk("0x%2x,", *(uint8_t *)(buf+i));
            }
            printk("\n");
        }
    }
    kfree(buf);
    return ret;
}


//#define APPLICATION_ADDR            client->addr
#define APPLICATION_REG_ADDR_LEN    0x1
/*===================================
iic read in application
client
reg
len
*val
===================================*/
static int mchp_bsw_read_in_app(struct i2c_client *client, u16 reg, u16 len, void *val)
{
    struct i2c_msg xfer[2];
    u8 buf[2];
    int ret,i;
    bool retry = false;
    
    //dev_warn(&client->dev,"i2c read: ID: %x; Reg: %x; Len: %x", client->addr, reg, len);

#if (APPLICATION_REG_ADDR_LEN == 1)
    buf[0] = reg & 0xff;
#else
    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;
#endif

    /* Write register */
    xfer[0].addr = client->addr;
    xfer[0].flags = 0;
    xfer[0].len = APPLICATION_REG_ADDR_LEN;
    xfer[0].buf = buf;

    /* Read data */
    xfer[1].addr = client->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = len;
    xfer[1].buf = val;
    
    MCHP_BSW_LOG_DBG("i2c read: ID: %x; Reg: %x; Len: %x \n", xfer[0].addr, reg, len);

retry_read:
    ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
    if (ret != ARRAY_SIZE(xfer)) {
        if (!retry) {
            dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
            msleep(MCHP_BSW_WAKEUP_TIME);
            retry = true;
            //retry = false;
            
            goto retry_read;
        } else {
            dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
                __func__, ret);
            return -EIO;
        }
    }
    else
    {
        ret = 0;
    }
    
    if(bsw_log_level&MCHP_BSW_DEBUG)
    {
        for(i = 0; i < len; i++)
        {
            printk("0x%2x,", *(uint8_t *)(val+i));
        }
        printk("\n");
    }
    
    return ret;
}


/*===================================
iic write in application
client
reg
len
&val
===================================*/
static int mchp_bsw_write_in_app(struct i2c_client *client, u16 reg, u16 len, const void *val)
{
    
    u8 *buf;
    size_t count;
    int ret,i;
    struct i2c_msg msg;
    bool retry = false;
    

#if (APPLICATION_REG_ADDR_LEN == 1)
    count = len + 1;
#else
    count = len + 2;
#endif
    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
    {
        MCHP_BSW_LOG_ERR("malloc buf failed");
        return -ENOMEM;
    }


#if (APPLICATION_REG_ADDR_LEN == 1)
    buf[0] = reg & 0xff;
    if(count > 1)
        memcpy(&buf[1], val, len);
#else
    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;
    if(count > 2)
        memcpy(&buf[2], val, len);
#endif
        
    //write address 
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = count;
    msg.buf = (u8 *)buf;
    
    MCHP_BSW_LOG_DBG("i2c write: ID: %x; Reg: %x; Len: %x \n", msg.addr, reg, len);
    
retry_write:
    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret != 1) {
        if (!retry) {
            dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
            msleep(MCHP_BSW_WAKEUP_TIME);
            retry = true;
            goto retry_write;
        } else {
            dev_err(&client->dev, "%s: i2c send failed (%d)\n",
                __func__, ret);
            ret = -EIO;
        }
    } else {
        ret = 0;
    }    
    
    if(bsw_log_level&MCHP_BSW_DEBUG)
    {
            
        if(count > 2)
        {
            for(i = 2; i < count; i++)
            {
                printk("0x%2x,", *(uint8_t *)(buf+i));
            }
            printk("\n");
        }
    }
    kfree(buf);
    return ret;
}

/*=============================================
key report,

key setting
    
key report
    input_event(input_dev, EV_KEY,KeyValue, 1);  1: down; 0: up
type 1:
        init:   input_set_capability(input_dev, EV_KEY, keymap[keynum]);
        report: input_event(input_dev, EV_KEY, keymap[keynum], keyval);  keyval 1: down; 0: up
                input_sync(input);
           
type 2:
        init:   __set_bit(EV_KEY, input->evbit);
                _set_bit(keymap[keynum], input->keybit);
        report: input_report_key(input, keymap[keynum], keyval);    keyval 1:down, 0: release_firmware
                input_sync(input);
    
=============================================*/
#define KEY_NUM     7
static const unsigned short mchp_bsw_keymap[] = 
{
    0,1,2,3,4,5,6
};

/*============================================================
setup keys
============================================================*/
static int mchp_bsw_key_setup(struct mchp_bsw_data *data)
{
    int key_cnt;

    data->keystatus = 0;
    for(key_cnt = 0; key_cnt < KEY_NUM; key_cnt ++)
    {
        input_set_capability(data->input, EV_KEY,mchp_bsw_keymap[key_cnt]);
    }
    return 0;
}


/*============================================================
report keys
============================================================*/
static int mchp_bsw_report_key(struct mchp_bsw_data *data, uint16_t keystatus)
{
	struct input_dev *input_dev = data->input;
    uint16_t new_keystatus, old_key_status;
    uint8_t key_cnt;
    bool sync;
        
    for (key_cnt = 0; key_cnt < KEY_NUM; key_cnt++) 
    {
        old_key_status = test_bit(key_cnt, (unsigned long*)&data->keystatus);
        new_keystatus = test_bit(key_cnt, (unsigned long*)&keystatus);
        
        if (!old_key_status && new_keystatus) 
        {
            MCHP_BSW_LOG_DBG("key press: %u\n", key_cnt);
            __set_bit(key_cnt, (unsigned long*)&data->keystatus);
            input_event(input_dev, EV_KEY, mchp_bsw_keymap[key_cnt], 1);
            sync = true;
        } 
        else if (old_key_status && !new_keystatus) 
        {
            MCHP_BSW_LOG_DBG("key release: %u\n", key_cnt);
            __clear_bit(key_cnt, (unsigned long*)&data->keystatus);
            input_event(input_dev, EV_KEY, mchp_bsw_keymap[key_cnt], 0);
            sync = true;
        }
    }
    
    if (sync)
        input_sync(input_dev);
        
    return 0;
}


/*===================================
bsw irq
===================================*/
static irqreturn_t mchp_bsw_irq(int irq, void *dev_id)
{
    int ret = 0;
    struct mchp_bsw_data *data = dev_id;
    uint8_t keys[10];
    MCHP_BSW_LOG_DBG("+");
    
    //read data in interrup
    mutex_lock(&data->i2c_lock);
    ret = mchp_bsw_read_in_app(data->client, 0, 10,keys);
    mutex_unlock(&data->i2c_lock);
    MCHP_BSW_LOG_INFO("read key status: 0x%2x", keys[2]);
    data->fw_version = keys[4];
    mchp_bsw_report_key(data, keys[2]);
    MCHP_BSW_LOG_DBG("-");
    return IRQ_HANDLED;
}


/*===================================
mchp_bsw_idenify
check chip 
===================================*/
#define DEVICE_INFO_ADDR    4
#define DEVICE_INFO_SIZE    2
static int mchp_bsw_idenify(struct mchp_bsw_data *data)
{
    int ret = 0;
    uint8_t info[DEVICE_INFO_SIZE];
    ret = mchp_bsw_read_in_app(data->client, DEVICE_INFO_ADDR, DEVICE_INFO_SIZE,info);
    if(ret)
    {
        //failed to find application
        MCHP_BSW_LOG_ERR("failed to find any mchp bsw device");
        return -1;
    }
    data->fw_version = info[0];
    data->chip_id = info[1];
    MCHP_BSW_LOG_INFO("device found ver: %x.%x", (data->fw_version >> 4) & 0xF, data->fw_version &  0xF);
    
    return ret;
}


#define FW_MAX_SIZE             (14*1024)
#define FW_PAGE_SIZE            64
#define ERR_ENTER_BOOTLOADER     -1


#define BOOT_LOADER_FW_VERSION_CMD          0x8000
#define BOOT_LOADER_UNLOCK_FLASH_CMD        0x8001
#define BOOT_LOADER_LOCK_FLASH_CMD            0x8002
#define BOOT_LOADER_CHECK_FLASH_STATUS_CMD    0x8005
#define BOOT_LOADER_REBOOT_CMD                0X8008


#define BOOT_LOADER_FLAG_ADDR                  10
#define BOOT_LOADER_FLAG_STR                   "ATBL"

static int mchp_bsw_write_bl_cmd(struct i2c_client *client, uint16_t cmd)
{
    
    return mchp_bsw_write_in_bootloader(client, cmd, 0, NULL);
}


static int mchp_bsw_check_bootloader(struct i2c_client *client)
{
    int ret = 0;
    uint8_t buf;
    ret = mchp_bsw_read_in_bootloader(client, BOOT_LOADER_FW_VERSION_CMD, 1, &buf);
    if(!ret)
        MCHP_BSW_LOG_INFO("get boot loader version: %x.%x", (buf&0xF0) >> 4, buf&0xF);
    
    return ret;
}

static int mchp_bsw_force_bootloader(struct i2c_client *client)
{
    int ret = 0;
    uint8_t times = 0;
    //
    MCHP_BSW_LOG_DBG("send force cmd");
    ret = mchp_bsw_write_in_app(client, BOOT_LOADER_FLAG_ADDR, 4, BOOT_LOADER_FLAG_STR);
    if(ret)
    {
        MCHP_BSW_LOG_ERR("send force bl failed ");
    }
    for(times = 0; times < 5; times ++)
    {
        msleep(100);
        ret = mchp_bsw_check_bootloader(client);
        if(!ret)
        {
            MCHP_BSW_LOG_DBG("enter bl success at time: %d", times);
            break;
        }
        MCHP_BSW_LOG_ERR("enter bl fail at time: %d", times);
    }
    return ret;
}

static int mchp_bsw_unlock_flash(struct i2c_client *client)
{
    
    return mchp_bsw_write_bl_cmd(client, BOOT_LOADER_UNLOCK_FLASH_CMD);
}

static int mchp_bsw_lock_flash(struct i2c_client *client)
{
    return mchp_bsw_write_bl_cmd(client, BOOT_LOADER_LOCK_FLASH_CMD);
}

static int mchp_bsw_reboot_to_app(struct i2c_client *client)
{
    return mchp_bsw_write_bl_cmd(client, BOOT_LOADER_REBOOT_CMD);
}

static int mchp_bsw_write_firmware(struct i2c_client *client, uint8_t *data, unsigned int len)
{
    int ret = 0;
    unsigned int op_len, wr_len, reg_addr = 0;
    uint8_t rd_buf[FW_PAGE_SIZE], retry = 0, retry1 = 0; 
    
    
    memset(rd_buf, 0x0, FW_PAGE_SIZE);
    
    for(wr_len = 0; wr_len < len; wr_len+=FW_PAGE_SIZE)
    {
        
            //repeat 5 for each page
            if(len - wr_len >= FW_PAGE_SIZE)
            {
                op_len = FW_PAGE_SIZE;
            }
            else
            {
                op_len = len - wr_len;
            }
re_write:            
            //check flash status
            do{
                ret = mchp_bsw_read_in_bootloader(client, BOOT_LOADER_CHECK_FLASH_STATUS_CMD, 1, (uint8_t *)rd_buf);
                if(!rd_buf[0])
                {
                    retry1 = 0;
                    break;
                }
                MCHP_BSW_LOG_DBG("flash is busy %x", rd_buf[0]);
                if(retry1 ++ > 10)
                {
                    MCHP_BSW_LOG_ERR("check flash busy error");
                    return -1;
                }
                msleep(10);
            }while(1);
            
            
            ret = mchp_bsw_write_in_bootloader(client, reg_addr, op_len, data);
            if(ret)
            {
                MCHP_BSW_LOG_ERR("write firmware error at addr: %x", reg_addr);
                return ret;
            }
            ret = mchp_bsw_read_in_bootloader(client, reg_addr, op_len, (uint8_t*)rd_buf);
            if(ret)
            {
                MCHP_BSW_LOG_ERR("read failed at addr: %x", reg_addr);
                return ret;
            }
            ret = memcmp(data, rd_buf, op_len);
            if(!ret)
            {
                MCHP_BSW_LOG_INFO("wrote at addr: %x, len: %d, total len: %x", reg_addr, op_len, wr_len);
                
                reg_addr += op_len;
                data += op_len;
                retry = 0;
                continue;
            }
            else
            {
                MCHP_BSW_LOG_ERR("verify fail @addr: %x retry: %d \n write data: 0x%2x, read data: 0x%2x ", reg_addr, retry, data[ret], rd_buf[ret]);
                if(retry++ < 5)
                    goto re_write;
                MCHP_BSW_LOG_ERR("update fw fail.");
                return -1;
            }
            
        
    }
    
    return ret = 0;
}

static void calc_crc24(uint32_t *crc, u8 firstbyte, u8 secondbyte)
{
    static const unsigned int crcpoly = 0x80001B;
    uint32_t result;
    uint32_t data_word;

    data_word = (secondbyte << 8) | firstbyte;
    result = ((*crc << 1) ^ data_word);

    if (result & 0x1000000)
        result ^= crcpoly;

    *crc = result;
}

static int fw_crc_check(uint8_t *start_addr, uint8_t *end_addr)
{
    uint32_t crc = 0;
    uint8_t *pdata = start_addr;
    if(end_addr < start_addr)
        return -1;
    while(pdata < end_addr)
    {
        calc_crc24(&crc, *pdata, *(pdata + 1));
        pdata += 2;
    }
    if(pdata == end_addr)
       calc_crc24(&crc, *pdata, 0);
    
    crc &= 0xFFFFFF;
    return crc;    
}

static int mchp_bsw_update_byH(struct mchp_bsw_data *data)
{
    int ret = 0; 
    uint8_t *fw_buf;
    unsigned int fw_length = sizeof(tiny1617_fw) - CRC_SIZE;
    int fw_crc_read,fw_crc_cal;

    
    MCHP_BSW_LOG_DBG("+");
    
    fw_crc_read = (tiny1617_fw[sizeof(tiny1617_fw) - 3] << 16)
                      | (tiny1617_fw[sizeof(tiny1617_fw) - 2] << 8)
                      | (tiny1617_fw[sizeof(tiny1617_fw) - 1] << 0);
    fw_crc_cal = fw_crc_check((uint8_t*)tiny1617_fw, (uint8_t*)(tiny1617_fw + fw_length - 1));
    MCHP_BSW_LOG_INFO("fw crc check result \n  read : %x \t cal: %x", fw_crc_read, fw_crc_cal);
    if(fw_crc_read != fw_crc_cal)
    {
        //crc check error
        return -1;
    }
        
    fw_buf = (uint8_t*)tiny1617_fw;
    
// enter bootloader mode
    if(mchp_bsw_check_bootloader(data->client) != 0)
    {
        //force to bootloader        
        ret = mchp_bsw_force_bootloader(data->client);
        if(ret)
        {
        //  
            MCHP_BSW_LOG_ERR("can not enter bl");
            return ERR_ENTER_BOOTLOADER;
        }
    }
    
    ret = mchp_bsw_unlock_flash(data->client);
    if(ret)
    {
        goto mchp_bsw_update_exit;
    }
    //updating
    ret  = mchp_bsw_write_firmware(data->client, fw_buf, fw_length);
    if(ret)
    {
        MCHP_BSW_LOG_ERR("update firmware failed");
        // do not reboot to application if update failed, because app may be crashed 
    }
    //updating finished reboot to app
    else
    {
        MCHP_BSW_LOG_INFO("reboot to application");
        ret = mchp_bsw_reboot_to_app(data->client);
    }
    
mchp_bsw_update_exit:
    //kfree(fw_buf);
    MCHP_BSW_LOG_DBG("-");
    return ret;
}


static int mchp_bsw_update(struct mchp_bsw_data *data, const char *file_name)
{
    int ret = 0; 
    uint8_t *fw_buf;
    const struct firmware *fw_entry;
    unsigned int fw_length;
    
    MCHP_BSW_LOG_DBG("+");

//request firmware    
    ret = request_firmware(&fw_entry, file_name, &data->client->dev);
    if(ret)
    {
        MCHP_BSW_LOG_ERR("request FW failed at %s", file_name);
        return -1;
    }
    
    fw_buf = (uint8_t  *)fw_entry->data;
    fw_length = (unsigned int)fw_entry->size;
    
    MCHP_BSW_LOG_INFO("fw size is:%x", fw_length);
    
    if(bsw_log_level&MCHP_BSW_DEBUG)
    {
        for(ret = 0; ret < 10; ret ++)
        {
            printk(  "0x%2x", fw_buf[ret]);
        }
        printk(  "\n");
    }
    
    if(fw_length != FW_MAX_SIZE)
    {        
        MCHP_BSW_LOG_ERR("fw size is incorrect:0x%x", fw_length);
        goto mchp_bsw_update_exit;
    }
    
  
// enter bootloader mode
    if(mchp_bsw_check_bootloader(data->client) != 0)
    {
        //force to bootloader        
        ret = mchp_bsw_force_bootloader(data->client);
        if(ret)
        {
        //  
            MCHP_BSW_LOG_ERR("can not enter bl");
            ret = ERR_ENTER_BOOTLOADER;
            goto mchp_bsw_update_exit;
        }
    }
    
    ret = mchp_bsw_unlock_flash(data->client);
    if(ret)
    {
        goto mchp_bsw_update_exit;
    }
    //updating
    ret  = mchp_bsw_write_firmware(data->client, fw_buf, fw_length);
    if(ret)
    {
        MCHP_BSW_LOG_ERR("update firmware failed");
        // do not reboot to application if update failed, because app may be crashed 
    }
    //updating finished reboot to app
    else
    {
        MCHP_BSW_LOG_INFO("reboot to application");
        ret = mchp_bsw_reboot_to_app(data->client);
    }
    
mchp_bsw_update_exit:
    //kfree(fw_buf);
    release_firmware(fw_entry);
    MCHP_BSW_LOG_DBG("-");
    return ret;
}


static ssize_t update_fw_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct mchp_bsw_data *data = dev_get_drvdata(dev);
    char *file_name;
    int ret;
    MCHP_BSW_LOG_INFO("file name %s, %d", buf, count);
    
    if (count > 64)
    {
        MCHP_BSW_LOG_ERR("name is too long");
        return -EINVAL;
    }
    
    file_name = kmalloc(count + 1, GFP_KERNEL);
    
    memcpy(file_name, buf, count);
    
    /* Echo into the sysfs entry may append newline at the end of buf */
    if (buf[count - 1] == '\n')
        file_name[count - 1] = '\0';
    else
        file_name[count] = '\0';
    ret = mchp_bsw_update(data, file_name);
    if(!ret)
    {
        MCHP_BSW_LOG_INFO("1617 firmware updated success");
    }
    kfree(file_name);
    return count;
}


static ssize_t update_fw_byH_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct mchp_bsw_data *data = dev_get_drvdata(dev);
    int ret;
    
    ret = mchp_bsw_update_byH(data);
    if(!ret)
    {
        MCHP_BSW_LOG_INFO("1617 firmware by H updated success");
    }
    return count;
}



/*show log level*/
static ssize_t set_loglevel_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "log level: 0x%2x \n", bsw_log_level);
}

/*set log level*/
static ssize_t set_loglevel_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    sscanf (buf,"%d", (int *)&bsw_log_level);
    MCHP_BSW_LOG_INFO("log level set to 0x%x", bsw_log_level);
    
    return count;
}

/*
byte0: slave addressb
byte1: len
byte2: frame
*/
static ssize_t app_read_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct mchp_bsw_data *data = dev_get_drvdata(dev);
    uint8_t *r_buf;
    int count = 0,i,frame;
    
    if((!(data->debug.flag)) || (data->debug.len < 1))
        return scnprintf(buf, PAGE_SIZE, "set param first \n");
    
    r_buf = kmalloc(data->debug.len, GFP_KERNEL);
    if(!r_buf)
    {
        return scnprintf(buf, PAGE_SIZE, "alloc error\n");
    }
    
    frame = data->debug.frame;
    while(--frame > 0)
    {
        mchp_bsw_read_in_app(data->client, data->debug.reg_addr, data->debug.len, r_buf);        
        for(i = 0; i < data->debug.len; i ++)
        {
            count += scnprintf(buf + count, PAGE_SIZE - count, "0x%2x,", r_buf[i]);
        }
        count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
    }
    kfree(r_buf);
    
    return count;
    
}

/*
byte0: slave addressb
byte1: len
byte2: frame
*/
static ssize_t app_read_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct mchp_bsw_data *data = dev_get_drvdata(dev);
    uint16_t reg_addr,len,frame;

    if(count < 2)
        return count;
    
    
    sscanf (buf,"%d %d %d", (int *)&reg_addr, (int *)&len, (int *)&frame);
    data->debug.reg_addr = reg_addr;
    data->debug.len = len;
    data->debug.frame = frame;
    data->debug.flag = true;
    MCHP_BSW_LOG_DBG("reg_addr: %x, len: %x, frame: %x", reg_addr, len, frame);    
    return count;
}


/*
*/
#define STRING_ENABLE_ADDR  0xE

static ssize_t string_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct mchp_bsw_data *data = dev_get_drvdata(dev);
    uint8_t w_data = 1;
    uint16_t reg_addr = STRING_ENABLE_ADDR;
    int flag;
    
    sscanf (buf,"%d", &flag);
    
    if(flag)
        w_data = 1;
    else
        w_data = 0;
    
    mchp_bsw_write_in_app(data->client,reg_addr, 1, &w_data);
    return count;
}

/*
byte0,1: slave addressb 0x12 0x34 = 1234
byte3: len

*/
/*
static ssize_t bl_read_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct mchp_bsw_data *data = dev_get_drvdata(dev);
    uint16_t reg_addr,len,frame,cnt;
    uint8_t *r_buf;
    
    if(count < 3)
        return count;
    
    
    sscanf (buf,"%d %d %d", (int*)&reg_addr, (int*)&len, (int*)&frame);
//    sscanf (buf, "%d %d %d", &p1,&p2,&p3);
    
    MCHP_BSW_LOG_DBG("reg_addr: %x, len: %x, frame: %x", reg_addr, len, frame);
    
    r_buf = kmalloc(len, GFP_KERNEL);
    if(!r_buf)
    {
        MCHP_BSW_LOG_ERR("-");
        return count;
    }
        
    for(cnt = 0; cnt < frame; cnt ++)
        mchp_bsw_read_in_bootloader(data->client, reg_addr, len - 2, r_buf);
    
    kfree(r_buf);
    return count;
}*/

static ssize_t app_fw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mchp_bsw_data *data = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE, "firmware version: %x.%x", (data->fw_version >> 4) & 0xF, data->fw_version &  0xF);
}


static ssize_t chip_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mchp_bsw_data *data = dev_get_drvdata(dev);
    
    return scnprintf(buf, PAGE_SIZE, "chip id is %d\n", data->chip_id);
}

static DEVICE_ATTR(fw_version, S_IRUGO, app_fw_version_show, NULL);
static DEVICE_ATTR(chip_type, S_IRUGO, chip_type_show, NULL);

static DEVICE_ATTR(update_fw, S_IWUSR, NULL, update_fw_store);
static DEVICE_ATTR(update_fw_byH, S_IWUSR, NULL, update_fw_byH_store);
static DEVICE_ATTR(loglevel_set, S_IRUGO | S_IWUSR, set_loglevel_show, set_loglevel_store);
static DEVICE_ATTR(app_iic_read, S_IRUGO | S_IWUSR, app_read_show, app_read_store);
//static DEVICE_ATTR(bl_iic_read, S_IWUSR, NULL, bl_read_store);
static DEVICE_ATTR(string_enable, S_IWUSR, NULL, string_enable_store);



static struct attribute *mchp_bsw_attrs[] = {
    &dev_attr_fw_version.attr,
    &dev_attr_chip_type.attr,
    &dev_attr_update_fw.attr, /*update fw*/
    &dev_attr_update_fw_byH.attr, /*update fw*/
    &dev_attr_loglevel_set.attr, /*set loglevel*/
    &dev_attr_app_iic_read.attr, /*read in app*/
//    &dev_attr_bl_iic_read.attr, /*read in bl*/
    &dev_attr_string_enable.attr, /*on/off string*/
    NULL
};

static const struct attribute_group mchp_bsw_attr_group = {
    .attrs = mchp_bsw_attrs,
};

static int mchp_bsw_sysfs_init(struct mchp_bsw_data *data)
{
    struct i2c_client *client = data->client;
    int ret;

    ret = sysfs_create_group(&client->dev.kobj, &mchp_bsw_attr_group);
    if (ret) {        
        MCHP_BSW_LOG_ERR("Failure %d creating sysfs group\n",ret);
        return ret;
    }
    return 0;

}

static void mchp_bsw_sysfs_remove(struct mchp_bsw_data *data)
{
    struct i2c_client *client = data->client;

    sysfs_remove_group(&client->dev.kobj, &mchp_bsw_attr_group);
}



static int mchp_bsw_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct mchp_bsw_data *data;
    struct input_dev *input_dev;
    
    MCHP_BSW_LOG_DBG("+");
    
    //i2c check
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        //check i2c failed 
        //boot loader check here ? or....
        MCHP_BSW_LOG_ERR("check i2c error");
        return -ENODEV;
    }
        
    //alloc mem and input device
    data = kzalloc(sizeof(struct mchp_bsw_data), GFP_KERNEL);
    if(!data)
    {
        MCHP_BSW_LOG_ERR("alloc mchp bsw data failed");
        return -ENODEV;
    }
    input_dev = input_allocate_device();
    if(!input_dev)
    {
        MCHP_BSW_LOG_ERR("alloc input device failed");
        ret = -ENODEV;
        goto err_free_mem;
    }
    
    
    data->client = client;
    data->input = input_dev;
    input_dev->name = "Microchip BSW";
    input_dev->id.bustype = BUS_I2C;
    
    
    //device check
    ret = mchp_bsw_idenify(data);
    if(ret)
    {
        //if read chip fail, try to enter bootloader and update
        ret = mchp_bsw_update_byH(data);
        if(ret)
            goto err_free_mem;
    }
    
    //register device
    ret =  input_register_device(data->input);
    if(ret)
    {
        MCHP_BSW_LOG_ERR("failed to register input device");
        goto err_free_irq;
    }
    
    i2c_set_clientdata(client,data);
    
    mutex_init(&data->i2c_lock);
    
    
    //request irq
    if(client->irq)
    {
        ret = request_threaded_irq(client->irq, NULL, mchp_bsw_irq, MCHP_BSW_IRQ_TYPE | IRQF_ONESHOT, "MCHP_BSW", data);
        if(ret)
        {
            MCHP_BSW_LOG_ERR("irq request failed %d", client->irq);
            goto err_free_mem;
        }
    }
    else
    {
        MCHP_BSW_LOG_ERR("none irq assign to client");
        goto err_free_mem;
    }
    
    mchp_bsw_key_setup(data);
    ret = mchp_bsw_sysfs_init(data);
    if (ret) 
    {
        goto err_free_irq;
    }
    
    MCHP_BSW_LOG_DBG("-");
    return 0;

err_free_irq:
    if(client->irq)
        free_irq(client->irq, data);
err_free_mem:
    if(input_dev)
        input_free_device(input_dev);
    kfree(data);
    return ret;
}

static int mchp_bsw_remove(struct i2c_client *client)
{
    struct mchp_bsw_data *data = i2c_get_clientdata(client);

    /* Release IRQ so no queue will be scheduled */
    if(client->irq)
        free_irq(client->irq, data);
    if(data->input)
        input_unregister_device(data->input);
    mchp_bsw_sysfs_remove(data);
    if(data)
        kfree(data);
    

    return 0;
}


static const struct i2c_device_id mchp_bsw_idtable[] = {
    { "mchp_bsw", 0, },
    { }
};

MODULE_DEVICE_TABLE(i2c, mchp_bsw_idtable);

static struct i2c_driver mchp_bsw_driver = {
    .driver = {
        .name    = "mchp_bsw",
    },

    .id_table    = mchp_bsw_idtable,
    .probe        = mchp_bsw_probe,
    .remove        = mchp_bsw_remove,
};

module_i2c_driver(mchp_bsw_driver);

MODULE_AUTHOR("roger zhu");
MODULE_DESCRIPTION("Driver for MCHP IIC BSW ");
MODULE_LICENSE("GPL");

