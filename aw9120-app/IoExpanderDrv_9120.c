/**
*  @file      IoExpanderDrv_9120.c
*  @brief     babababala
*  @version   v0.1
*  @author    Gavin Lee, Alex.Li
*  @date      2017/8/18
*  @copyright Tymphany Ltd.
*/

#include "IoExpanderDrv_9120.config"
#include "./IoExpanderDrv_9120_priv.h"
#include "gpioDrv.h"
#include "I2CDrv.h"
#include "trace.h"
#include "bsp.h"
#include "attachedDevices.h"

#ifdef IOEXPANDERDRV_RST_CONTROL
static cGpioDrv gpioResetIoExpanderDrv;
#endif

#define FADE_CODE_LENGTH 7
uint16 fadecode[] =
{
    0xBF00, 0x9F80, 0xFFFF, 0x3C20, 0xDFFF, 0x3C38, 0X0002
};
#define BLINK_CODE_LENGTH 6
uint16 blinkcode[] =
{
    0xBF00, 0xBFFF, 0x3C3E, 0xBF00, 0x3C3E, 0x0001
};

/******************************************************************************
 *
 * Public functions
 *
 ******************************************************************************/

/**
  * @brief  IO Expander driver constructor
  * @param  me: pointer to IO-Expander driver object.
  * @param  pIoeLedConfig: device configuration (AW9120 is I2C device).
  * @retval None
  */
void IoExpanderDrv_Ctor_aw9120(cIoExpanderDrv *me, tIoeLedDevice *pIoeLedConfig)
{
    ASSERT(me);

    if(!me->i2cDrv.isReady)
    {
        /*
        * All project ioexpander config should be pre-defined in IoExpanderDrv.config
        * These io configurations normally will not be changed, thus ioexpander only Ctor once,
        */

#ifdef IOEXPANDERDRV_RST_CONTROL
        /* Ctor gpio for ioexpander reset control */
        tGPIODevice *ioexpanderRstCtrl = NULL;
        ioexpanderRstCtrl = (tGPIODevice*)getDevicebyIdAndType(IO_EXPANDER_DEV_ID, GPIO_DEV_TYPE, NULL);
        ASSERT(ioexpanderRstCtrl);
        GpioDrv_Ctor(&gpioResetIoExpanderDrv, ioexpanderRstCtrl);

        /* Gave Power to the touch (572 + 360), and IO-Expendor.
         * Both two drivers initialize this pin.
         */

        /* Reset ioexpander before init started */
        for(int i = 0; i < ioexpanderRstCtrl->usedGPIOPinNum; i++)
        {
            GpioDrv_ClearBit(&gpioResetIoExpanderDrv, (ioexpanderRstCtrl->pGPIOPinSet + i)->gpioId);
        }

        /* FS1/FS2 exeption handler will ctor IO-Expender, but interrupt already disable on exception,
         * to let ctor() work on both normal and exception mode, let it support two blocking delay.
         */
        if(BSP_InExp())
            BSP_ExpBlockingDelayMs(50);

        else
            BSP_BlockingDelayMs(50);

        for(int i = 0; i < ioexpanderRstCtrl->usedGPIOPinNum; i++)
        {
            GpioDrv_SetBit(&gpioResetIoExpanderDrv, (ioexpanderRstCtrl->pGPIOPinSet + i)->gpioId);
        }

        if(BSP_InExp())
            BSP_ExpBlockingDelayMs(50);

        else
            BSP_BlockingDelayMs(50);

#ifdef GPIO_OUT_TCH_POWER
        GpioDrv_SetBit(&gpioResetIoExpanderDrv, GPIO_OUT_TCH_POWER);
#endif

#endif

        I2CDrv_Ctor(&me->i2cDrv, (tI2CDevice*)pIoeLedConfig->i2cDeviceConf);
#ifndef NDEBUG
        uint16 deviceID = 0;
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_IDRST, &deviceID);
        ASSERT(deviceID == AW9120_DEV_ID);//check if chip ID matched
#endif

        // software reset
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IDRST, AW9120_RST_VALUE);

        //set max current
        IoExpanderDrv_SetAllLedMaxCurrent_aw9120(me, LED_MAX_CURRENT);

#ifdef IOEXPANDERDRV_HAS_ALWAYS_ON

        for(int i = 0; i < AW9120_MAX_PINS; i++)
        {
            if(EXPANDER_ALWAYS_ON_PINS & (1 << i))
            {
                IoExpanderDrv_SetLedMaxCurrent_aw9120(me, i, EXPANDER_ALWAYS_ON_MAX_CURRENT);
                IoExpanderDrv_TurnLedOn_aw9120(me, i);
            }
        }

#endif
    }
}

/**
  * @brief  IO Expander driver destructor
  * @param  me: pointer to IO-Expander driver object.
  * @retval None
  */
void IoExpanderDrv_Xtor_aw9120(cIoExpanderDrv *me, tIoeLedDevice *pIoeLedConfig)
{
    if(me->i2cDrv.isReady)
    {
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IDRST, AW9120_RST_VALUE);
        I2CDrv_Xtor(&me->i2cDrv);
        me->i2cDrv.isReady = FALSE;

#ifdef IOEXPANDERDRV_RST_CONTROL
        tGPIODevice *ioexpanderRstCtrl = NULL;
        ASSERT(pIoeLedConfig);
        ioexpanderRstCtrl = pIoeLedConfig->pResetGpioDevice;
        ASSERT(ioexpanderRstCtrl);

        for(int i = 0; i < ioexpanderRstCtrl->usedGPIOPinNum; i++)
        {
            GpioDrv_ClearBit(&gpioResetIoExpanderDrv, (ioexpanderRstCtrl->pGPIOPinSet + i)->gpioId);
        }
#endif
    }
}

/**
  * @brief  IO Expander driver re-constructor.use it when chip has no response
  * @param  None
  * @retval None
  */
void IoExpanderDrv_ReCtor_aw9120()
{
#ifdef IOEXPANDERDRV_RST_CONTROL
    tGPIODevice *ioexpanderRstCtrl = NULL;
    ioexpanderRstCtrl = (tGPIODevice*)getDevicebyIdAndType(IO_EXPANDER_DEV_ID, GPIO_DEV_TYPE, NULL);
    ASSERT(ioexpanderRstCtrl);

    for(int i = 0; i < ioexpanderRstCtrl->usedGPIOPinNum; i++)
    {
        GpioDrv_ClearBit(&gpioResetIoExpanderDrv, (ioexpanderRstCtrl->pGPIOPinSet + i)->gpioId);
    }

    BSP_BlockingDelayMs(50);

    for(int i = 0; i < ioexpanderRstCtrl->usedGPIOPinNum; i++)
    {
        GpioDrv_SetBit(&gpioResetIoExpanderDrv, (ioexpanderRstCtrl->pGPIOPinSet + i)->gpioId);
    }

    BSP_BlockingDelayMs(50);
#endif
    tIoeLedDevice *pIoeLedConfig = (tIoeLedDevice*) getDevicebyIdAndType(LED_DEV_ID, IO_EXPANDER_DEV_TYPE, NULL);
    cIoExpanderDrv ioeDrv = {0};
    IoExpanderDrv_Ctor_aw9120(&ioeDrv, pIoeLedConfig);
}

/**
  * @brief  Turn on specified LED to max brightness.
  * @param  me: pointer to IO-Expander driver object.
  * @param  pin: specified pin,which can be 0~19.
  * @retval None
  */
void IoExpanderDrv_TurnLedOn_aw9120(cIoExpanderDrv *me, uint8 pin)
{
    //send SETPWMI cmd
    IoExpanderDrv_SetBrightness_aw9120(me, pin, 0xFF);
}

/**
  * @brief  Turn off specified LED.
  * @param  me: pointer to IO-Expander driver object.
  * @param  pin: specified pin,which can be 0~19.
  * @retval None
  */
void IoExpanderDrv_TurnLedOff_aw9120(cIoExpanderDrv *me, uint8 pin)
{
    uint16 data = 0;

    //send SETPWMI cmd
    data = 0xA000;
    data |= ((uint16)pin) << 8;
    data |= (0x0000);
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_CMDR, data);

    if(pin < AW9120_LER1_CTRL_PINS)
    {
        //disable driver output
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_LER1, &data);
        data &= (~(1 << pin));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_LER1, data);
        //Set control source:mcu
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_CTRS1, &data);
        data |= (1 << pin);
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_CTRS1, data);
    }

    else if(pin < AW9120_MAX_PINS)
    {
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_LER2, &data);
        data &= (~(1 << (pin - AW9120_LER1_CTRL_PINS)));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_LER2, data);
        //Set control source:mcu
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_CTRS2, &data);
        data |= (1 << (pin - AW9120_LER1_CTRL_PINS));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_CTRS2, data);
    }

    else
    {
        ASSERT(0);
    }
}

/**
  * @brief  Set specified LED's brightness.Only usefull when control source is mcu
  * @param  me: pointer to IO-Expander driver object.
  * @param  pin: specified pin,which can be 0~19.
  * @param  brightness: brightness can be 0~255.0 means Turn off,255 means max brightness
  * @retval None
  */
void IoExpanderDrv_SetBrightness_aw9120(cIoExpanderDrv *me, uint8 pin, uint8 brightness)
{
    uint16 data = 0;

    if(pin < AW9120_LER1_CTRL_PINS)
    {
        //enable driver output
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_LER1, &data);
        data |= (1 << pin);
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_LER1, data);
        //Set control source:mcu
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_CTRS1, &data);
        data |= (1 << pin);
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_CTRS1, data);
    }

    else if(pin < AW9120_MAX_PINS)
    {
        //enable driver output
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_LER2, &data);
        data |= (1 << (pin - AW9120_LER1_CTRL_PINS));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_LER2, data);
        //Set control source:mcu
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_CTRS2, &data);
        data |= (1 << (pin - AW9120_LER1_CTRL_PINS));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_CTRS2, data);
    }

    else
    {
        ASSERT(0);
    }

    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_GCR, 0x0001);//enable LED module
    //send SETPWMI cmd
    data = 0xA000;
    data |= ((uint16)pin) << 8;
    data |= ((uint16)brightness);
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_CMDR, data);
}

/**
  * @brief  Set AW9120 into ASP mode and auto blink.
  * @param  me: pointer to IO-Expander driver object.
  * @param  patt: Pattern of auto blink
  * @retval None
  */
void IoExpanderDrv_AutoBlink_aw9120(cIoExpanderDrv *me, eIoeAutoPatt patt)
{
    tIoePattern_9120 *pPatt = &autoBlinkPatt_aw9120[patt];
    uint32 led_mask;
    uint16 data;

    if(!pPatt->led_mask)
    {
        return;
    }

    led_mask = pPatt->led_mask;
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IDRST, AW9120_RST_VALUE);//Reset chip
    IoExpanderDrv_SetAllLedMaxCurrent_aw9120(me, LED_MAX_CURRENT);//set max current

#ifdef IOEXPANDERDRV_HAS_ALWAYS_ON

    for(int i = 0; i < AW9120_MAX_PINS; i++)
    {
        if(EXPANDER_ALWAYS_ON_PINS & (1 << i))
        {
            IoExpanderDrv_SetLedMaxCurrent_aw9120(me, i, EXPANDER_ALWAYS_ON_MAX_CURRENT);
            IoExpanderDrv_TurnLedOn_aw9120(me, i);
        }
    }

#endif

    for(int i = 0; i < AW9120_MAX_PINS; i++)
    {
        if(led_mask & (1 << i))
        {
            if(i < AW9120_LER1_CTRL_PINS)
            {
                //enable driver output
                IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_LER1, &data);
                data |= (1 << i);
                IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_LER1, data);
                //Set control source:ASP
                IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_CTRS1, &data);
                data &= (~(1 << i));
                IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_CTRS1, data);
            }

            else if(i < AW9120_MAX_PINS)
            {
                //enable driver output
                IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_LER2, &data);
                data |= (1 << (i - AW9120_LER1_CTRL_PINS));
                IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_LER2, data);
                //Set control source:ASP
                IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_CTRS2, &data);
                data &= (~(1 << (i - AW9120_LER1_CTRL_PINS)));
                IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_CTRS2, data);
            }
        }
    }

    if(pPatt->led_mode == LedMode_AutoBlink)
    {
        blinkcode[2] = WAITI_16MS_TIME | ((uint16)pPatt->interval);
        blinkcode[4] = WAITI_16MS_TIME | ((uint16)pPatt->interval);
        IoExpanderDrv_Programming_aw9120(me, blinkcode, BLINK_CODE_LENGTH);
    }

    else
    {
        fadecode[2] = RAMPI_FADEIN_STEP | ((uint16)pPatt->fade_in_step);
        fadecode[3] = WAITI_16MS_TIME | ((uint16)pPatt->interval);
        fadecode[4] = RAMPI_FADEOUT_STEP | ((uint16)pPatt->fade_out_step);
        fadecode[5] = WAITI_16MS_TIME | ((uint16)pPatt->interval);
        IoExpanderDrv_Programming_aw9120(me, fadecode, FADE_CODE_LENGTH);
    }

}

/******************************************************************************
 *
 * Private functions
 *
 ******************************************************************************/

static void IoExpanderDrv_I2cWrite_aw9120(cIoExpanderDrv *me, uint8 reg, uint16 value)
{
    cI2CDrv* pi2cObj = &me->i2cDrv;
    uint8 data[3] = {reg, (uint8)((value >> 8) & 0x00ff), (uint8)(value & 0x00ff)};
    tI2CMsg i2cMsg =
    {
        .devAddr    = pi2cObj->pConfig->devAddress,
        .regAddr    = NULL,
        .length     = 3,
        .pMsg       = data,
    };

    if(TRUE == me->i2cDrv.isReady)
    {
        if(TP_SUCCESS != I2CDrv_MasterWrite(&me->i2cDrv, &i2cMsg))
        {
            me->i2cDrv.isReady = FALSE;
        }
    }
}

static void IoExpanderDrv_I2cRead_aw9120(cIoExpanderDrv *me, uint8 reg, uint16 * value)
{
    cI2CDrv* pi2cObj = &me->i2cDrv;
    uint8 data[2] = {0};
    tI2CMsg i2cMsg =
    {
        .devAddr    = pi2cObj->pConfig->devAddress,
        .regAddr    = reg,
        .length     = 2,
        .pMsg       = data,
    };

    if(TRUE == me->i2cDrv.isReady)
    {
        if(TP_SUCCESS != I2CDrv_MasterRead(&me->i2cDrv, &i2cMsg))
        {
            me->i2cDrv.isReady = FALSE;
        }
    }

    * value = (uint16)((data[0] << 8) | data[1]);
}

/**
  * @brief  Set all LED's max current.
  * @param  me: pointer to IO-Expander driver object.
  * @param  maxCurrent: e9120LedMaxCurrent type value.
  * @retval None
  */
static void IoExpanderDrv_SetAllLedMaxCurrent_aw9120(cIoExpanderDrv *me, e9120LedMaxCurrent maxCurrent)
{
    uint16 data = 0;

    for(int i = 0; i < 4; i++)
    {
        data |= (maxCurrent << (i * 4));
    }

    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX1, data);
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX2, data);
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX3, data);
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX4, data);
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX5, data);
}

/**
  * @brief  Set specified LED's max current.
  * @param  me: pointer to IO-Expander driver object.
  * @param  pin: specified pin,which can be 1~AW9120_MAX_PINS.
  * @param  maxCurrent: e9120LedMaxCurrent type value.
  * @retval None
  */
void IoExpanderDrv_SetLedMaxCurrent_aw9120(cIoExpanderDrv *me, uint8 pin, e9120LedMaxCurrent maxCurrent)
{
    ASSERT(pin < AW9120_MAX_PINS);
    uint16 data = 0;
    uint16 setValue = 0;
    setValue = (uint16)maxCurrent;

    if(pin < 4)
    {
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_IMAX1, &data);
        data &= (~(0x0007 << (pin * 4)));
        data |= (setValue << (pin * 4));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX1, data);
    }

    else if(pin < 8)
    {
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_IMAX2, &data);
        data &= (~(0x0007 << ((pin - 4) * 4)));
        data |= (setValue << ((pin - 4) * 4));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX2, data);
    }

    else if(pin < AW9120_LER1_CTRL_PINS)
    {
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_IMAX3, &data);
        data &= (~(0x0007 << ((pin - 8) * 4)));
        data |= (setValue << ((pin - 8) * 4));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX3, data);
    }

    else if(pin < 16)
    {
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_IMAX4, &data);
        data &= (~(0x0007 << ((pin - AW9120_LER1_CTRL_PINS) * 4)));
        data |= (setValue << ((pin - AW9120_LER1_CTRL_PINS) * 4));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX4, data);
    }

    else if(pin < AW9120_MAX_PINS)
    {
        IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_IMAX5, &data);
        data &= (~(0x0007 << ((pin - 16) * 4)));
        data |= (setValue << ((pin - 16) * 4));
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_IMAX5, data);
    }

    else
    {
        ASSERT(0);
    }
}

/**
  * @brief  Download specified machine code(fading or blink) into aw9120 and run.
  * @param  me: pointer to IO-Expander driver object.
  * @param  code: specified code pointer.
  * @param  codelength: specified code length.
  * @retval None
  */
static void IoExpanderDrv_Programming_aw9120(cIoExpanderDrv *me, const uint16 * code, uint8 codelength)
{
    uint16 data;
    //Disable LED driver
    IoExpanderDrv_I2cRead_aw9120(me, AW9120_REG_GCR, &data);
    data &= 0xFFFE;
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_GCR, data);

    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_LCR, 0x0100);//Log(e) dimming mode

    //Enable LED driver
    data |= 0x0001;
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_GCR, data);

    //Download program
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_PMR, 0x0000);//Stop runing program
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_RMR, 0x0000);//Load program via I2C
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_WADDR, 0x0000);//Set LED program loading address

    //Programming
    for(int i = 0; i < codelength; i++)
    {
        IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_WDATA, code[i]);
    }

    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_SADDR, 0x0000);//Set program start address
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_RMR, 0x0002);//Execution mode change to run mode
    IoExpanderDrv_I2cWrite_aw9120(me, AW9120_REG_PMR, 0x0001);//Start program from SADDR
}
