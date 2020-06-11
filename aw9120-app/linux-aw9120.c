#include <stdio.h>
#include <fcntl.h>
#include <error.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <assert.h>

#define ASSERT(x) assert(x)
typedef enum
{
    LedMaxC_0,//0mA
    LedMaxC_3_5,//3.5mA
    LedMaxC_7_0,//7.0mA
    LedMaxC_10_5,//10.5mA
    LedMaxC_14_0,//14.0mA
    LedMaxC_17_5,//17.5mA
    LedMaxC_21_0,//21.0mA
    LedMaxC_24_5//24.5mA
} e9120LedMaxCurrent;

#define I2C_FILE_NAME   "/dev/i2c-9"
#define I2C_ADDR        0x2d
static int fd_i2c;

typedef unsigned char uint8;
typedef unsigned int  uint16;
#define AW9120_LER1_CTRL_PINS (12)
#define AW9120_MAX_PINS (20)

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW9120_I2C_NAME "aw9120_led"

#define AW9120_VERSION "v1.0.3"

#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 5
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 5

#define REG_RSTR    0x00
#define REG_GCR     0x01

#define REG_LER1    0x50
#define REG_LER2    0x51
#define REG_LCR     0x52
#define REG_PMR     0x53
#define REG_RMR     0x54
#define REG_CTRS1   0x55
#define REG_CTRS2   0x56
#define REG_IMAX1   0x57
#define REG_IMAX2   0x58
#define REG_IMAX3   0x59
#define REG_IMAX4   0x5a
#define REG_IMAX5   0x5b
#define REG_TIER    0x5c
#define REG_INTVEC  0x5d
#define REG_LISR2   0x5e
#define REG_SADDR   0x5f

#define REG_PCR     0x60
#define REG_CMDR    0x61
#define REG_RA      0x62
#define REG_RB      0x63
#define REG_RC      0x64
#define REG_RD      0x65
#define REG_R1      0x66
#define REG_R2      0x67
#define REG_R3      0x68
#define REG_R4      0x69
#define REG_R5      0x6a
#define REG_R6      0x6b
#define REG_R7      0x6c
#define REG_R8      0x6d
#define REG_GRPMSK1 0x6e
#define REG_GRPMSK2 0x6f

#define REG_TCR     0x70
#define REG_TAR     0x71
#define REG_TDR     0x72
#define REG_TDATA   0x73
#define REG_TANA    0x74
#define REG_TKST    0x75
#define REG_FEXT    0x76
#define REG_WP      0x7d
#define REG_WADDR   0x7e
#define REG_WDATA   0x7f

/* aw9120 register read/write access*/
#define REG_NONE_ACCESS                 0
#define REG_RD_ACCESS                   (1 << 0)
#define REG_WR_ACCESS                   (1 << 1)
#define AW9120_REG_MAX                  0xFF

const unsigned char aw9120_reg_access[AW9120_REG_MAX] =
{
    [REG_RSTR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_GCR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_LER1] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_LER2] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_LCR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_PMR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_RMR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_CTRS1] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_CTRS2] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_IMAX1] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_IMAX2] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_IMAX3] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_IMAX4] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_IMAX5] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_TIER] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_INTVEC] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_LISR2] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_SADDR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_PCR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_CMDR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_RA] = REG_RD_ACCESS,
    [REG_RB] = REG_RD_ACCESS,
    [REG_RC] = REG_RD_ACCESS,
    [REG_RD] = REG_RD_ACCESS,
    [REG_R1] = REG_RD_ACCESS,
    [REG_R2] = REG_RD_ACCESS,
    [REG_R3] = REG_RD_ACCESS,
    [REG_R4] = REG_RD_ACCESS,
    [REG_R5] = REG_RD_ACCESS,
    [REG_R6] = REG_RD_ACCESS,
    [REG_R7] = REG_RD_ACCESS,
    [REG_R8] = REG_RD_ACCESS,
    [REG_GRPMSK1] = REG_RD_ACCESS,
    [REG_GRPMSK2] = REG_RD_ACCESS,
    [REG_TCR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_TAR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_TDR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_TDATA] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_TANA] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_TKST] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_FEXT] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_WP] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_WADDR] = REG_RD_ACCESS | REG_WR_ACCESS,
    [REG_WDATA] = REG_RD_ACCESS | REG_WR_ACCESS
};

/******************************************************
 *
 * aw9120 led parameter
 *
 ******************************************************/
/* The definition of each time described as shown in figure.
 *        /-----------\
 *       /      |      \
 *      /|      |      |\
 *     / |      |      | \-----------
 *       |hold_time_ms |      |
 *       |             |      |
 * rise_time_ms  fall_time_ms |
 *                       off_time_ms
 */
#define ROM_CODE_MAX 255

/*
 * rise_time_ms = 1500
 * hold_time_ms = 500
 * fall_time_ms = 1500
 * off_time_ms = 1500
 */
static int led_code_len = 7;
static int led_code[ROM_CODE_MAX] =
{
 //   0xbf00, 0x9f05, 0xfffa, 0x3c7d, 0xdffa, 0x3cbb, 0x2,
   0xbf00, 
   0x9f03,
   //0xe080, 
   0xfffa,

   0x3c20, 

   //0xc080, 
   0xdffa,

   0x3c38, 
   0x0002,
};


int i2c_open(void)
{
    int ret;
    int val;

    fd_i2c = open(I2C_FILE_NAME, O_RDWR);

    if(fd_i2c < 0)
    {
        perror("Unable to open bq25703 i2c control file");

        return -1;
    }

    printf("open bq25703 i2c file success,fd is %d\n",fd_i2c);

    ret = ioctl(fd_i2c, I2C_SLAVE_FORCE, I2C_ADDR);
    if (ret < 0)
    {
        perror("i2c: Failed to set i2c device address\n");
        return -1;
    }

    printf("i2c: set i2c device address success\n");

    val = 3;
    ret = ioctl(fd_i2c, I2C_RETRIES, val);
    if(ret < 0)
    {
        printf("i2c: set i2c retry times err\n");
    }

    printf("i2c: set i2c retry times %d\n",val);

    return 0;
}

static int iic_write(unsigned char dev_addr, unsigned char reg, unsigned char *val, unsigned char len)
{
    int ret;
    int i;
    
    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg messages;
    unsigned char *buf=(unsigned char *)malloc(sizeof(unsigned char)*len);
    buf[0] = reg;
    memcpy(buf+1,val,len);

    messages.addr = dev_addr;  //device address
    messages.flags = 0;    //write
    messages.len = len+1;
    messages.buf = buf;  //data

    data.msgs = &messages;
    data.nmsgs = 1;

    if(ioctl(fd_i2c, I2C_RDWR, &data) < 0)
    {
        printf("write ioctl err %d\n",fd_i2c);
        return -1;
    }

    return 0;
}

static int iic_read(unsigned char dev_addr, unsigned char reg, unsigned char *val, unsigned char len)
{
    int ret;
    int i;

    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg messages[2];

    messages[0].addr = dev_addr;  //device address
    messages[0].flags = 0;    //write
    messages[0].len = 1;
    messages[0].buf = &reg;  //reg address

    messages[1].addr = dev_addr;       //device address
    messages[1].flags = I2C_M_RD;  //read
    messages[1].len = len;
    messages[1].buf = val;

    data.msgs = messages;
    data.nmsgs = 2;

    if(ioctl(fd_i2c, I2C_RDWR, &data) < 0)
    {
        perror("---");
        printf("read ioctl err %d\n",fd_i2c);

        return -1;
    }

    return 0;
}

static int aw9120_i2c_write(unsigned char reg_addr, unsigned int reg_data)
{
    int ret = -1;
    unsigned char cnt = 0;
    unsigned char wbuf[2];
    wbuf[0] = (unsigned char )(reg_data >> 8);
    wbuf[1] = (unsigned char )(reg_data & 0x00ff);

    while (cnt < AW_I2C_RETRIES)
    {
    
        ret = iic_write(I2C_ADDR,reg_addr,wbuf,2);
        if (ret < 0)
        {
            printf("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
                   ret);
        }
        else
        {
            break;
        }
        cnt++;
        usleep(AW_I2C_RETRY_DELAY * 1000);
    }
    usleep(AW_I2C_RETRY_DELAY * 1000);
    return ret;
}

static int aw9120_i2c_read(unsigned char reg_addr, unsigned int *reg_data)
{
    int ret = -1;
    unsigned int get_data;
    unsigned char rbuf[2];
    unsigned char cnt = 0;

    while (cnt < AW_I2C_RETRIES)
    {
        //ret = i2c_read(reg_addr,reg_data);
        ret = iic_read(I2C_ADDR,reg_addr,rbuf,2);
        if (ret < 0)
        {
            printf("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
                   ret);
        }
        else
        {
            break;
        }
        cnt++;
        usleep(AW_I2C_RETRY_DELAY * 1000);
    }
    
    get_data = (unsigned int) (rbuf[0] << 8);
    get_data |= (unsigned int) (rbuf[1]);
    *reg_data = get_data;

    return ret;
}

static void aw9120_ic_reset(){
    aw9120_i2c_write(REG_RSTR, 0x55AA);
}

static void aw9120_set_maxcurrent(uint8 pin, e9120LedMaxCurrent maxCurrent)
{
    ASSERT(pin < AW9120_MAX_PINS);
    uint16 data = 0;
    uint16 setValue = 0;
    setValue = (uint16)maxCurrent;

    if(pin < 4)
    {
        aw9120_i2c_read(REG_IMAX1, &data);
        data &= (~(0x0007 << (pin * 4)));
        data |= (setValue << (pin * 4));
        aw9120_i2c_write(REG_IMAX1, data);
    }

    else if(pin < 8)
    {
        aw9120_i2c_read(REG_IMAX2, &data);
        data &= (~(0x0007 << ((pin - 4) * 4)));
        data |= (setValue << ((pin - 4) * 4));
        aw9120_i2c_write(REG_IMAX2, data);
    }

    else if(pin < AW9120_LER1_CTRL_PINS)
    {
        aw9120_i2c_read(REG_IMAX3, &data);
        data &= (~(0x0007 << ((pin - 8) * 4)));
        data |= (setValue << ((pin - 8) * 4));
        aw9120_i2c_write(REG_IMAX3, data);
    }

    else if(pin < 16)
    {
        aw9120_i2c_read(REG_IMAX4, &data);
        data &= (~(0x0007 << ((pin - AW9120_LER1_CTRL_PINS) * 4)));
        data |= (setValue << ((pin - AW9120_LER1_CTRL_PINS) * 4));
        aw9120_i2c_write(REG_IMAX4, data);
    }

    else if(pin < AW9120_MAX_PINS)
    {
        aw9120_i2c_read(REG_IMAX5, &data);
        data &= (~(0x0007 << ((pin - 16) * 4)));
        data |= (setValue << ((pin - 16) * 4));
        aw9120_i2c_write(REG_IMAX5, data);
    }

    else
    {
        ASSERT(0);
    }
}

static void aw9120_set_all_maxcurrent(e9120LedMaxCurrent maxCurrent)
{
    uint16 imax = 0;
    for (int i = 0; i < 4; i++)
    {
        /* code */
        imax |= (maxCurrent << (i * 4));
    }

    aw9120_i2c_write(REG_IMAX1, imax);  /* IMAX1-LED1~LED4 Current */
    aw9120_i2c_write(REG_IMAX2, imax);  /* IMAX2-LED5~LED8 Current */
    aw9120_i2c_write(REG_IMAX3, imax);  /* IMAX3-LED9~LED12 Current */
    aw9120_i2c_write(REG_IMAX4, imax);  /* IMAX4-LED13~LED16 Current */
    aw9120_i2c_write(REG_IMAX5, imax);  /* IMAX5-LED17~LED20 Current */

}

static void aw9120_set_brightness(uint8 pin, uint8 brightness)
{
    uint16 data = 0;
    if(pin < AW9120_LER1_CTRL_PINS)
    {
        //enable driver output
        aw9120_i2c_read(REG_LER1, &data);
        printf("AW9120_LER1_CTRL_PINS data: 0x%x\r\n",data);
        data |= (1 << pin);
        aw9120_i2c_write(REG_LER1, data);
        //Set control source:mcu
        aw9120_i2c_read(REG_CTRS1, &data);
        data |= (1 << pin);
        aw9120_i2c_write(REG_CTRS1, data);
    }

    else if(pin < AW9120_MAX_PINS)
    {
        //enable driver output
        aw9120_i2c_read(REG_LER2, &data);
        printf("AW9120_MAX_PINS data: 0x%x\r\n",data);
        data |= (1 << (pin - AW9120_LER1_CTRL_PINS));
        printf("AW9120_MAX_PINS write data : 0x%x\r\n",data);
        aw9120_i2c_write(REG_LER2, data);
        //Set control source:mcu
        aw9120_i2c_read(REG_CTRS2, &data);
        data |= (1 << (pin - AW9120_LER1_CTRL_PINS));
        aw9120_i2c_write(REG_CTRS2, data);
    }

    else
    {
        ASSERT(0);
    }

    aw9120_i2c_write(REG_GCR, 0x0001);//enable LED module
    //send SETPWMI cmd
    data = 0xA000; //SETPWMI Ch Im
    data |= ((uint16)pin) << 8;
    data |= ((uint16)brightness);
    aw9120_i2c_write(REG_CMDR, data);
}

static void aw9120_turn_off(uint8 pin)
{
    uint16 data = 0;

    //send SETPWMI cmd
    data = 0xA000;
    data |= ((uint16)pin) << 8;
    data |= (0x0000);
    aw9120_i2c_write(REG_CMDR, data);

    if(pin < 12)
    {
        //disable driver output
        aw9120_i2c_read(REG_LER1, &data);
        printf("data: 0x%x\r\n",data);
        data &= (~(1 << pin));
        aw9120_i2c_write(REG_LER1, data);
        //Set control source:mcu
        aw9120_i2c_read(REG_CTRS1, &data);
        data |= (1 << pin);
        aw9120_i2c_write(REG_CTRS1, data);
    }

    else if(pin < 20)
    {
        aw9120_i2c_read(REG_LER2, &data);
        data &= (~(1 << (pin - 12)));
        aw9120_i2c_write(REG_LER2, data);
        //Set control source:mcu
        aw9120_i2c_read(REG_CTRS2, &data);
        data |= (1 << (pin - 12));
        aw9120_i2c_write(REG_CTRS2, data);
    }

    else
    {
        ASSERT(0);
    }
}

static void aw9120_led_blink(unsigned int this_imax, unsigned char blink)
{
    unsigned char i;
    unsigned int reg_val;
    unsigned int imax = 0;

    imax = (this_imax << 12) | (this_imax << 8) |
           (this_imax << 4) | (this_imax << 0);
    printf("[DEBUG] imax: 0x%x\r\n",imax);

    /*Disable LED Module */
    aw9120_i2c_read(REG_GCR, &reg_val);
    printf("[DEBUG] Read REG_GCR[0x%x]: 0x%x\r\n",REG_GCR, reg_val);
    reg_val &= 0xFFFE;
    aw9120_i2c_write(REG_GCR, reg_val); /* GCR-Disable LED Module */
    printf("[DEBUG] Write REG_GCR[0x%x]: 0x%x\r\n",REG_GCR, reg_val);

    if (blink)
    {
        /*Enable LED Module */
        reg_val |= 0x0001;
        aw9120_i2c_write(REG_GCR, reg_val); /* GCR-Enable LED Module */
        printf("[DEBUG] Write REG_GCR[0x%x]: 0x%x\r\n",REG_GCR, reg_val);

        /*LED Config */
        aw9120_i2c_write(REG_LER1, 0x0FFF); /* LER1-LED1~LED12 Enable */
        aw9120_i2c_write(REG_LER2, 0x00FF); /* LER2-LED13~LED20 Enable */
        aw9120_i2c_write(REG_IMAX1, imax);  /* IMAX1-LED1~LED4 Current */
        aw9120_i2c_write(REG_IMAX2, imax);  /* IMAX2-LED5~LED8 Current */
        aw9120_i2c_write(REG_IMAX3, imax);  /* IMAX3-LED9~LED12 Current */
        aw9120_i2c_write(REG_IMAX4, imax);  /* IMAX4-LED13~LED16 Current */
        aw9120_i2c_write(REG_IMAX5, imax);  /* IMAX5-LED17~LED20 Current */

#if 1
        aw9120_i2c_write(REG_CTRS1, 0x0000);    /* CTRS1-LED1~LED12: SRAM Control */
        aw9120_i2c_write(REG_CTRS2, 0x0000);    /* CTRS2-LED13~LED20: SRAM Control */

        /* LED SRAM Hold Mode */
        aw9120_i2c_write(REG_PMR, 0x0000);  /* PMR-Load SRAM with I2C */
        aw9120_i2c_write(REG_RMR, 0x0000);  /* RMR-Hold Mode */

        /* Load LED SRAM */
        aw9120_i2c_write(REG_WADDR, 0x0000);    /* WADDR-SRAM Load Addr */
        for (i = 0; i < led_code_len; i++)
            aw9120_i2c_write(REG_WDATA, led_code[i]);
        /* LED SRAM Run */
        aw9120_i2c_write(REG_SADDR, 0x0000);    /* SADDR-SRAM Run Start Addr:0 */
        aw9120_i2c_write(REG_PMR, 0x0001);  /* PMR-Reload and Excute SRAM */
        aw9120_i2c_write(REG_RMR, 0x0002);  /* RMR-Run */
#endif

    }
}

static void aw9120_one_led_blink(uint8 pin, unsigned int *blink_code, unsigned int code_len)
{
    uint16 data = 0;

    if(pin < AW9120_LER1_CTRL_PINS)
    {
        //enable driver output
        aw9120_i2c_read(REG_LER1, &data);
        data |= (1 << pin);
        aw9120_i2c_write(REG_LER1, data);
        //Set control source: SRAM Control
        aw9120_i2c_read(REG_CTRS1, &data);
        data |= (0 << pin);
        aw9120_i2c_write(REG_CTRS1, data);
    }

    else if(pin < AW9120_MAX_PINS)
    {
        //enable driver output
        aw9120_i2c_read(REG_LER2, &data);
        data |= (1 << (pin - AW9120_LER1_CTRL_PINS));
        aw9120_i2c_write(REG_LER2, data);
        //Set control source: SRAM Control
        aw9120_i2c_read(REG_CTRS2, &data);
        data |= (0 << (pin - AW9120_LER1_CTRL_PINS));
        aw9120_i2c_write(REG_CTRS2, data);
    }

    else
    {
        ASSERT(0);
    }

    aw9120_i2c_write(REG_GCR, 0x0001);//enable LED module

    /* LED SRAM Hold Mode */
    aw9120_i2c_write(REG_PMR, 0x0000);  /* PMR-Load SRAM with I2C */
    aw9120_i2c_write(REG_RMR, 0x0000);  /* RMR-Hold Mode */

    /* Load LED SRAM */
    aw9120_i2c_write(REG_WADDR, 0x0000);    /* WADDR-SRAM Load Addr */
    for (int i = 0; i < code_len; i++)
        aw9120_i2c_write(REG_WDATA, blink_code[i]);
    /* LED SRAM Run */
    aw9120_i2c_write(REG_SADDR, 0x0000);    /* SADDR-SRAM Run Start Addr:0 */
    aw9120_i2c_write(REG_PMR, 0x0001);  /* PMR-Reload and Excute SRAM */
    aw9120_i2c_write(REG_RMR, 0x0002);  /* RMR-Run */

}

#if 0
int main(void)
{

    unsigned int reg_val;

    i2c_open();

    e9120LedMaxCurrent mc;
    mc = LedMaxC_14_0;
    aw9120_i2c_write(REG_RSTR, 0x55AA);

    /*LED Config */
    set_all_max_current(3);

    /*
        IoExpanderDrv_SetBrightness_aw9120(14,255);

        IoExpanderDrv_SetBrightness_aw9120(1,255);
        //IoExpanderDrv_SetBrightness_aw9120(2,255);
        //IoExpanderDrv_SetBrightness_aw9120(3,255);

        //IoExpanderDrv_SetBrightness_aw9120(0,255);
        IoExpanderDrv_SetBrightness_aw9120(12,255);
        //IoExpanderDrv_SetBrightness_aw9120(13,255);

        IoExpanderDrv_SetBrightness_aw9120(9,255);
        IoExpanderDrv_SetBrightness_aw9120(10,255);
        //IoExpanderDrv_SetBrightness_aw9120(11,255);
    */

    aw9120_one_led_blink(14,mc);
    aw9120_one_led_blink(1,mc);
    //aw9120_one_led_blink(2,mc);
    //aw9120_one_led_blink(3,mc);

    //aw9120_one_led_blink(0,mc);
    aw9120_one_led_blink(12,mc);
    //aw9120_one_led_blink(13,mc);

    aw9120_one_led_blink(9,mc);
    aw9120_one_led_blink(10,mc);
    aw9120_one_led_blink(11,mc);
    sleep(1);

    printf("------------\r\n");
    aw9120_i2c_write(REG_RSTR, 0x55AA);
    set_all_max_current(3);
 
    aw9120_i2c_write(REG_GCR,0x0001);
    aw9120_i2c_write(REG_LER1,0x0fff);
    aw9120_i2c_write(REG_LER2,0x0ff);
    aw9120_i2c_write(REG_PMR,0x0000);
    aw9120_i2c_write(REG_RMR,0x0000);

    aw9120_i2c_write(REG_CTRS1,0xfff0); 
    aw9120_i2c_write(REG_CTRS2,0x00ff);

/*
    aw9120_i2c_write(REG_WADDR,0x0000);
    aw9120_i2c_write(REG_WDATA,0x9F03);
    aw9120_i2c_write(REG_WDATA,0xBF00);
    aw9120_i2c_write(REG_WDATA,0xE080);
    aw9120_i2c_write(REG_WDATA,0x3C20);
    aw9120_i2c_write(REG_WDATA,0xc080);
    aw9120_i2c_write(REG_WDATA,0x3c38);
    aw9120_i2c_write(REG_WDATA,0x0002);
*/

        /* Load LED SRAM */
    aw9120_i2c_write(REG_WADDR, 0x0000);    /* WADDR-SRAM Load Addr */
    for (int i = 0; i < led_code_len; i++)
            aw9120_i2c_write(REG_WDATA, led_code[i]);

    aw9120_i2c_write(REG_SADDR,0x0000);
    aw9120_i2c_write(REG_PMR,0x0001);
	aw9120_i2c_write(REG_RMR,0x0002);	

    int data;
    data = 0xA000;
    data |= ((uint16)14) << 8;
    data |= ((uint16)255);
    aw9120_i2c_write(REG_CMDR, data);
    /*
    while(1){

     
        usleep(1000*50);
    }
    */
    return 0;
}
#endif

int main(void){

    e9120LedMaxCurrent mc;
    mc = LedMaxC_14_0;
    
    i2c_open();

    aw9120_ic_reset();
    aw9120_set_all_maxcurrent(mc);

    //aw9120_set_maxcurrent(14,mc);
    aw9120_set_brightness(14,255);
    aw9120_set_brightness(13,255);

    aw9120_one_led_blink(0, led_code, led_code_len);
    //sleep(5);
    //aw9120_turn_off(14);

    return 0;
}