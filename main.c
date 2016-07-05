/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "lis302dl.h"

#include "usbcfg.h"

/* Accelerometer configuration bits - not included in lis302dl.h */
#define LIS302DL_CTRL_XYZ_EN        0x07
#define LIS302DL_CTRL_POWER         0x40
#define LIS302DL_CTRL_400HZ         0x80
#define LIS302DL_CTRL_DATAREADY1    0x04

#define CHUNK 20

/* Accel data - common for all threads, only modified in AccelThread */
static int8_t xbuf[CHUNK], ybuf[CHUNK], zbuf[CHUNK];

/* static struct AccelData
{
   int8_t x, y, z;

} accd; */
//static int8_t x, y, z;

/* Thread structure pointer used when switching threads */
static thread_t *tp_accel;

/*===========================================================================*/
/* Accelerometer related.                                                    */
/*===========================================================================*/

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static const PWMConfig pwmcfg = {
  100000,                                   /* 100kHz PWM clock frequency.  */
  128,                                      /* PWM period is 128 cycles.    */
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  /* HW dependent part.*/
  0,
  0
};

/*
 * SPI1 configuration structure.
 * Speed 5.25MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
 * The slave select line is the pin GPIOE_CS_SPI on the port GPIOE.
 */
static const SPIConfig spi1cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOE,
  GPIOE_CS_SPI,
  SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

/*
 * Hardware interrupt which wakes accel thread
 */
static void extCallback(EXTDriver *extp, expchannel_t channel)
{
  (void)extp;
  (void)channel;

  chSysLockFromISR();
  chEvtSignalI(tp_accel, (eventmask_t)1); /* Wake thread */
  chSysUnlockFromISR();
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOE, extCallback},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};


/*===========================================================================*/
/* Threads.                                                                  */
/*===========================================================================*/
#define TEST_WA_SIZE    THD_WORKING_AREA_SIZE(256)

/*
 * USB writer. This thread writes accelerometer data to the USB at maximum rate.
 */
static THD_WORKING_AREA(waWriter, 128);
static THD_FUNCTION(Writer, arg) {

  (void)arg;
  chRegSetThreadName("writer");
#if CYCLIC
  systime_t time = chVTGetSystemTime();
#endif

  // TODO: STM32_OTG1_EP1OUT_HANDLER

  while (true) {
    /* Concatenate accelerometer data */
    uint8_t xyzbuf[3*CHUNK];
    memcpy(xyzbuf                            , &xbuf, sizeof(xbuf));
    memcpy(xyzbuf+sizeof(xbuf)               , &ybuf, sizeof(ybuf));
    memcpy(xyzbuf+sizeof(xbuf)+sizeof(ybuf)  , &zbuf, sizeof(zbuf));

    // TODO: USB interrupts
    // TODO: ep1outstate.thread

    msg_t msg = usbTransmit(&USBD1, USBD1_DATA_REQUEST_EP,
                            xyzbuf, sizeof(xyzbuf));
                            //txbuf, sizeof (txbuf) - 1);
    //if (msg == MSG_RESET)

    /* Operate at 400Hz, not accurately (time error cumulation) */
    chThdSleepMilliseconds(2.5);

#if CYCLIC
    /* Accurately 400Hz: Waiting until the next 2.5 milliseconds time interval (400Hz)*/
    chThdSleepUntil(time += MS2ST(2.5));
#endif
  }
}

/*
   * LIS302DL initialization: X,Y,Z axes with 400Hz rate,
   * enable DataReady signal INT1 (PE0 pin) by setting I1CFG in CTRL_REG3 to "100"
   * Sensor is in "Power down" state until first dummy read
   */
static void lis302init(void)
{
  /* Reboot memory content */
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG2, 0x40);
  while(lis302dlReadRegister(&SPID1, LIS302DL_CTRL_REG2) & 0x40)
    ;

  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG1,
           LIS302DL_CTRL_XYZ_EN | LIS302DL_CTRL_400HZ);
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG3, LIS302DL_CTRL_DATAREADY1);

  /* dummy read to put down accelerometer interrupt */
  lis302dlReadRegister(&SPID1, LIS302DL_STATUS_REG);
  lis302dlReadRegister(&SPID1, LIS302DL_OUTX);
  lis302dlReadRegister(&SPID1, LIS302DL_OUTY);
  lis302dlReadRegister(&SPID1, LIS302DL_OUTZ);

  /* enable external interrupt on STM32 */
  extChannelEnable(&EXTD1, 0);  // PE0

  /* Power up sensor */
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG1,
             LIS302DL_CTRL_XYZ_EN | LIS302DL_CTRL_POWER | LIS302DL_CTRL_400HZ);
}


/*
 * This is a data-ready-triggered thread that reads accelerometer and outputs
 * result to PWM. Read data is available for USB Writer thread.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(AccelThread, arg) {
#if 0
  static int8_t xbuf[CHUNK], ybuf[CHUNK], zbuf[CHUNK];  /* Last accelerometer data.*/
  static uint8_t usbCnt;
#endif

  (void)arg;
  chRegSetThreadName("accelReader");
  tp_accel = chThdGetSelfX();

  /* Accelerometer reader thread loop.*/
  while (true) {

    /* Checks if an IRQ happened else wait.*/
    chEvtWaitAny((eventmask_t)1);

    unsigned i;
    /* Keeping an history of the latest accelerometer readings.*/
    for (i = CHUNK-1; i > 0; i--) {
      xbuf[i] = xbuf[i - 1];
      ybuf[i] = ybuf[i - 1];
      zbuf[i] = zbuf[i - 1];
    }

    /* Reading MEMS accelerometer X, Y and Z registers.*/
    xbuf[0] = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTX);
    ybuf[0] = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTY);
    zbuf[0] = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTZ);

    /* Reprogramming the four PWM channels using the accelerometer data.*/
    if (ybuf[0] < 0) {
      pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)-ybuf[0]);
      pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)0);
    }
    else {
      pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)ybuf[0]);
      pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)0);
    }
    if (xbuf[0] < 0) {
      pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)-xbuf[0]);
      pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)0);
    }
    else {
      pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)xbuf[0]);
      pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)0);
    }

    // if not cyclic, wake Writer thread here
  }
}


/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(1500);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

  /*
   * Initializes the SPI driver 1 in order to access the MEMS. The signals
   * are already initialized in the board file.
   */
  spiStart(&SPID1, &spi1cfg);

  /*
   * Initializes the EXT Driver to react to LIS302 interrupt on PE0 pin of Discovery board.
   */
  extStart(&EXTD1, &extcfg);

  /*
   * Initializes accelerometer with PE0 external interrupt on data ready.
   */
  lis302init();

  /*
   * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
   */
  pwmStart(&PWMD4, &pwmcfg);
  palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));      /* Green.   */
  palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));      /* Orange.  */
  palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_ALTERNATE(2));      /* Red.     */
  palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));      /* Blue.    */

  /*
   * Starting threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, AccelThread, NULL);
  chThdCreateStatic(waWriter, sizeof(waWriter), NORMALPRIO, Writer, NULL);

  /*
   * Normal main() thread activity
   */
  while (true) {
    chThdSleepMilliseconds(200);
  }
}
