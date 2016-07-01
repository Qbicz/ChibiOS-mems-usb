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
#include "test.h"

#include "lis302dl.h"

#include "usbcfg.h"

/* Accelerometer configuration bits - not included in lis302dl.h */
#define LIS302DL_STATUS_XYZ_READY   0x08
#define LIS302DL_CTRL_XYZ_EN        0x07
#define LIS302DL_CTRL_POWER         0x40
#define LIS302DL_CTRL_400HZ         0x80
#define LIS302DL_CTRL_DATAREADY1    0x04

/* use synchronous (blocking) thread waking */
#define SYNCHRONOUS
#undef SYNCHRONOUS

/* Accel data - common for all threads, only modified in AccelThread */
static int8_t x, y, z;
// TODO: use Chibi Mailboxes instead

//static uint8_t rxbuf[8];

#ifndef SYNCHRONOUS
  static thread_t *tp_accel;
#else
  static thread_reference_t trp = NULL;
#endif
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
  /* Wake thread */
#ifndef SYNCHRONOUS
  //chEvtSignalI(tp_accel, (eventmask_t)1);



  /* Reading MEMS accelerometer X, Y and Z registers.*/
      x = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTX);
      y = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTY);
      z = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTZ);

  #if 0
      /* Calculating average of the latest four accelerometer readings.*/
      x = ((int32_t)xbuf[0] + (int32_t)xbuf[1] +
           (int32_t)xbuf[2] + (int32_t)xbuf[3]) / 4;
      y = ((int32_t)ybuf[0] + (int32_t)ybuf[1] +
           (int32_t)ybuf[2] + (int32_t)ybuf[3]) / 4;
      z = ((int32_t)zbuf[0] + (int32_t)zbuf[1] +
           (int32_t)zbuf[2] + (int32_t)zbuf[3]) / 4;
  #endif
      /* Reprogramming the four PWM channels using the accelerometer data.*/
      if (y < 0) {
        pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)-y);
        pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)0);
      }
      else {
        pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)y);
        pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)0);
      }
      if (x < 0) {
        pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)-x);
        pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)0);
      }
      else {
        pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)x);
        pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)0);
      }



#else
  chThdResumeI(&trp, (msg_t)0xFEED);
#endif
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


   // TODO: check STM32_OTG2_EP1OUT_HANDLER

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

  while (true) {
    /* Concatenate accelerometer data */
    uint8_t xyzbuf[3];
    memcpy(xyzbuf                       , &x, sizeof(x));
    memcpy(xyzbuf+sizeof(x)             , &y, sizeof(y));
    memcpy(xyzbuf+sizeof(x)+sizeof(y)   , &z, sizeof(z));

    // TODO: USB interrupts
    msg_t msg = usbTransmit(&USBD1, USBD1_DATA_REQUEST_EP,
                            xyzbuf, sizeof(xyzbuf));
                            //txbuf, sizeof (txbuf) - 1);
    if (msg == MSG_RESET)
      chThdSleepMilliseconds(50);
  }
}
#ifdef READER
/*
 * USB reader. This thread reads data from the USB at maximum rate.
 * Can be measured using:
 *   dd if=bigfile of=/dev/xxx bs=512 count=10000
 */
static THD_WORKING_AREA(waReader, 128);
static THD_FUNCTION(Reader, arg) {

  (void)arg;
  chRegSetThreadName("reader");
  while (true) {
    msg_t msg = usbReceive(&USBD1, USBD1_DATA_AVAILABLE_EP,
                           rxbuf, sizeof (rxbuf) - 1);
    if (msg == MSG_RESET)
      chThdSleepMilliseconds(50);
  }
}
#endif

static void lis302init(void)
{
  /*
   * LIS302DL initialization: X,Y,Z axes with 400Hz rate,
   * enable DataReady signal INT1 (PE0 pin) by setting I1CFG in CTRL_REG3 to "100"
   * Sensor is in "Power down" state until first dummy read
   */
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG2, 0x40);
  while(lis302dlReadRegister(&SPID1, LIS302DL_CTRL_REG2) & 0x40)
    ;

  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG1,
           LIS302DL_CTRL_XYZ_EN | LIS302DL_CTRL_400HZ);
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG3, LIS302DL_CTRL_DATAREADY1);

  /* dummy read to put down accelerometer interrupt */
  lis302dlReadRegister(&SPID1, LIS302DL_STATUS_REG);

  /* Power up sensor and enable external interrupt on STM32 */
  lis302dlWriteRegister(&SPID1, LIS302DL_CTRL_REG1,
             LIS302DL_CTRL_XYZ_EN | LIS302DL_CTRL_POWER | LIS302DL_CTRL_400HZ);
  extChannelEnable(&EXTD1, 0);  // PE0
}

/*
 * This is a data-ready-triggered thread that reads accelerometer and outputs
 * result to PWM.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(AccelThread, arg) {
#if 0
  static int8_t xbuf, ybuf, zbuf;  /* Last accelerometer data.*/
  systime_t time;                           /* Next deadline.*/
#endif

  /* Initialize accelerometer with PE0 external interrupt on data ready */
  lis302init();

  (void)arg;
  chRegSetThreadName("accelReader");
#ifndef SYNCHRONOUS
  tp_accel = chThdGetSelfX();
#endif

  /* Reader thread loop.*/
  //time = chVTGetSystemTime();
  while (true) {
    /* Checks if an IRQ happened else wait.*/
#ifndef SYNCHRONOUS
    chEvtWaitAny((eventmask_t)1);
#else
    msg_t msg;

    /* Waiting for the IRQ to happen.*/
    chSysLock();
    msg = chThdSuspendS(&trp);
    chSysUnlock();
#endif

#if 0
    unsigned i;
    /* Keeping an history of the latest four accelerometer readings.*/
    for (i = 3; i > 0; i--) {
      xbuf[i] = xbuf[i - 1];
      ybuf[i] = ybuf[i - 1];
      zbuf[i] = zbuf[i - 1];
    }
#endif

    /* Reading MEMS accelerometer X, Y and Z registers.*/
    x = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTX);
    y = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTY);
    z = (int8_t)lis302dlReadRegister(&SPID1, LIS302DL_OUTZ);

#if 0
    /* Calculating average of the latest four accelerometer readings.*/
    x = ((int32_t)xbuf[0] + (int32_t)xbuf[1] +
         (int32_t)xbuf[2] + (int32_t)xbuf[3]) / 4;
    y = ((int32_t)ybuf[0] + (int32_t)ybuf[1] +
         (int32_t)ybuf[2] + (int32_t)ybuf[3]) / 4;
    z = ((int32_t)zbuf[0] + (int32_t)zbuf[1] +
         (int32_t)zbuf[2] + (int32_t)zbuf[3]) / 4;
#endif
    /* Reprogramming the four PWM channels using the accelerometer data.*/
    if (y < 0) {
      pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)-y);
      pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)0);
    }
    else {
      pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)y);
      pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)0);
    }
    if (x < 0) {
      pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)-x);
      pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)0);
    }
    else {
      pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)x);
      pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)0);
    }

    // TODO: use LIS302 accelerometer ready interrupt
    /* Waiting until the next 250 milliseconds time interval.*/
    //chThdSleepUntil(time += MS2ST(100));
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
  extStart(&EXTD1, &extcfg);    // GPIO E

  /*
   * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
   */
  pwmStart(&PWMD4, &pwmcfg);
  palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));      /* Green.   */
  palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));      /* Orange.  */
  palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_ALTERNATE(2));      /* Red.     */
  palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));      /* Blue.    */

  /* Initiate IRQ */
  //nvicEnableVector(EXTI0_IRQn, CORTEX_PRIO_MASK(1));

  /*
   * Starting threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, AccelThread, NULL);
  chThdCreateStatic(waWriter, sizeof(waWriter), NORMALPRIO, Writer, NULL);
  // chThdCreateStatic(waReader, sizeof(waReader), NORMALPRIO, Reader, NULL);


  /*
   * Normal main() thread activity
   */
  while (true) {
    chThdSleepMilliseconds(50);
  }
}
