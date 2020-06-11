#ifndef IOEXPANDERrDRV_9120_PRIV_H
#define IOEXPANDERrDRV_9120_PRIV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "IoExpanderDrv.h"

#define AW9120_MAX_PINS       (20)
#define AW9120_LER1_CTRL_PINS (12)//1~12 controled by LER1, others controled by LER2
    
/*Software reset and chip ID register
 *Read this register will get chip ID:0xB223
 *Write 0x55AA to IDRST, reset whole device*/
#define AW9120_REG_IDRST      (0x00)
#define AW9120_DEV_ID         (0xB223)
#define AW9120_RST_VALUE      (0x55AA)

/* Global Control Register, wrtie 1 to enable LED driver*/
#define AW9120_REG_GCR        (0x01)

/* LED Driver Enable Register */
#define AW9120_REG_LER1       (0x50)
#define AW9120_REG_LER2       (0x51)

#define AW9120_REG_LCR        (0x52)/* LED Effect Configuration Register */
#define AW9120_REG_PMR        (0x53)/* Program Mode Register */
#define AW9120_REG_RMR        (0x54)/* Program Run Mode Register */
#define AW9120_REG_CTRS1      (0x55)/* LED Control Source Selection Register */
#define AW9120_REG_CTRS2      (0x56)/* LED Control Source Selection Register */

/* LEDx Maximum Output Current Register */
#define AW9120_REG_IMAX1      (0x57)
#define AW9120_REG_IMAX2      (0x58)
#define AW9120_REG_IMAX3      (0x59)
#define AW9120_REG_IMAX4      (0x5A)
#define AW9120_REG_IMAX5      (0x5B)

#define AW9120_REG_LISR       (0x5E)/* LED Interrupt Status Register */
#define AW9120_REG_SADDR      (0x5F)/* Program Start Address Register */
#define AW9120_REG_PCR        (0x60)/* LED Program Control Pointer Register */
#define AW9120_REG_CMDR       (0x61)/* LED Command Register */

/* LED Internal Program Register, READ ONLY, for debug usage */
#define AW9120_REG_RA         (0x62)
#define AW9120_REG_RB         (0x63)
#define AW9120_REG_RC         (0x64)
#define AW9120_REG_RD         (0x65)

/* LED Internal Data Register, READ ONLY, for debug usage */
#define AW9120_REG_R1         (0x66)
#define AW9120_REG_R2         (0x67)
#define AW9120_REG_R3         (0x68)
#define AW9120_REG_R4         (0x69)
#define AW9120_REG_R5         (0x6A)
#define AW9120_REG_R6         (0x6B)
#define AW9120_REG_R7         (0x6C)
#define AW9120_REG_R8         (0x6D)

#define AW9120_REG_GRPR       (0x6E)/* LED Group Operation Register */
#define AW9120_REG_WPR        (0x7D)/* Writing Protection Register */
#define AW9120_REG_WADDR      (0x7E)/* LED Program Loading Address Register */
#define AW9120_REG_WDATA      (0x7F)/* LED Program Loading Data Register */
#define AW9120_REG_WPR        (0x7D)/* Writing Protection Register */

/* ASP CMD define */
#define RAMPI_FADEIN_STEP     (0xFF00)// low 8-bit should be modified as step
#define RAMPI_FADEOUT_STEP    (0xDF00)// low 8-bit should be modified as step
#define WAITI_16MS_TIME       (0x3C00)// low 8-bit should be modified as interval

/******************************************************************************
 *
 * Config parameter define
 *
 ******************************************************************************/
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

typedef enum
{
    LedMode_AutoBlink,
    LedMode_AutoFade
} e9120LedMode;

typedef struct tIoePattern_9120
{
    uint32 led_mask;
    e9120LedMode led_mode;
    uint8 fade_in_step;//fade in time = fade_in_step * 16ms
    uint8 fade_out_step;//fade out time = fade_out_step * 16ms
    uint8 interval;//interval between two blink/fade,real interval time = interval * 16ms
    //interval time should >= fade time
} tIoePattern_9120;

/******************************************************************************
 *
 * Private functions
 *
 ******************************************************************************/
static void IoExpanderDrv_I2cWrite_aw9120(cIoExpanderDrv *me, uint8 reg, uint16 value);
static void IoExpanderDrv_I2cRead_aw9120(cIoExpanderDrv *me, uint8 reg, uint16 * value);
static void IoExpanderDrv_SetAllLedMaxCurrent_aw9120(cIoExpanderDrv *me, e9120LedMaxCurrent maxCurrent);
static void IoExpanderDrv_SetLedMaxCurrent_aw9120(cIoExpanderDrv *me, uint8 pin, e9120LedMaxCurrent maxCurrent);
static void IoExpanderDrv_Programming_aw9120(cIoExpanderDrv *me, const uint16 * code, uint8 codelength);


#ifdef __cplusplus
}
#endif

#endif
