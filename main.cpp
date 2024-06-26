#include "stm_lib/inc/misc.h"
#include "cmsis_boot/stm32f10x.h"
#include "stm_lib/inc/stm32f10x_dma.h"
#include "stm_lib/inc/stm32f10x_exti.h"
#include "stm_lib/inc/stm32f10x_flash.h"
#include "stm_lib/inc/stm32f10x_gpio.h"
#include "stm_lib/inc/stm32f10x_i2c.h"
#include "stm_lib/inc/stm32f10x_iwdg.h"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "stm_lib/inc/stm32f10x_adc.h"
// #include "stm_lib/inc/stm32f10x_can.h"


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "arduino.h"
#include "balanceController.hpp"
#include "boardController.hpp"
#include "drv/comms/communicator.hpp"
#include "drv/mpu6050/mpu.hpp"
#include "drv/settings/settings.hpp"
#include "global.h"
#include "guards/angleGuard.hpp"
#include "imu/imu.hpp"
#include "io/genericOut.hpp"
#include "io/i2c.hpp"
#include "io/usart.hpp"
#include "io/rx.h"
#include "lpf.hpp"
#include "pid.hpp"
#include "stateTracker.hpp"


extern "C" void EXTI15_10_IRQHandler(void) {
  if (EXTI_GetITStatus(MPU6050_INT_Exti))  // MPU6050_INT
  {
    EXTI_ClearITPendingBit(MPU6050_INT_Exti);
    mpuHandleDataReady();
  }
}

extern "C" void EXTI0_IRQHandler(void) {
	constexpr uint32_t line = EXTI_Line0;

  // GPIOB->BSRR = GPIO_Pin_3;
  if (EXTI_GetITStatus(line))
  {
    EXTI_ClearITPendingBit(line);
    on_ppm_interrupt();
  }
  
  // GPIOB->BRR = GPIO_Pin_3;
}

class InitWaiter : public UpdateListener {
 public:
  InitWaiter(GenericOut *status_led, IMU *imu, Guard *angle_guard)
      : status_led_(status_led), imu_(imu), angle_guard_(angle_guard) {}

  void processUpdate(const MpuUpdate &update) {
    if (!accGyro.calibrationComplete()) {
      imu_->compute(update, true);
    } else {
      imu_->compute(update);
    }
    angle_guard_->Update();
  }

  void waitForAccGyroCalibration() {
    uint16_t last_check_time = 0;
    while (!accGyro.calibrationComplete() || angle_guard_->CanStart()) {
      IWDG_ReloadCounter();
      if ((uint16_t)(millis() - last_check_time) > 200u) {
        last_check_time = millis();
        status_led_->toggle();
      }
    }
  }

 private:
  GenericOut *status_led_;
  IMU *imu_;
  Guard *angle_guard_;
};

static uint8_t scratch[512];

uint8_t write_pos = 0;
uint8_t read_pos = 0;
static uint8_t debug[200];

static Communicator comms;

void initRx() {
	/* GPIO configuration */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void initEncoders() {
  GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    // A2, A3, A6, A7, B0, B1
}

uint8_t debug_stream_type = 0;


typedef enum
{
    FAILED = 0, PASSED = !FAILED
} TestStatus;

GenericOut* status_led_ext;


extern uint8_t cf_data[] asm("_binary_build_descriptor_pb_bin_deflate_start");
extern uint8_t cf_data_e[] asm("_binary_build_descriptor_pb_bin_deflate_end");

static BoardController* global_main_ctrl;

void InitDecoderTimer() {
  // TIMER
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef timer_settings;
  TIM_TimeBaseStructInit(&timer_settings);
  timer_settings.TIM_Prescaler = SystemCoreClock / 1000000 - 1;  // 1us tick ;
  timer_settings.TIM_Period = 10;                                // 100kHz
  timer_settings.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &timer_settings);
  TIM_Cmd(TIM2, ENABLE);

  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 0;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

extern "C" void TIM2_IRQHandler() {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    global_main_ctrl->updateDecoders();
  }
}


int main(void) {
  SystemInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  FLASH_SetLatency(FLASH_Latency_1);

  RCC_LSICmd(ENABLE);
  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {
  }

  /* Enable Watchdog*/
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_8);  // 4, 8, 16 ... 256
  IWDG_SetReload(
      0x0FFF);  // This parameter must be a number between 0 and 0x0FFF.
  IWDG_ReloadCounter();
  IWDG_Enable();

  initArduino();

  i2c_init();
  Serial1.Init(USART1, 115200);
  Serial2.Init(USART2, 115200);



  Config cfg = Config_init_default;
  if (readSettingsFromFlash(&cfg)) {
    const char msg[] = "Config OK\n";
    Serial1.Send((uint8_t *)msg, sizeof(msg));
  } else {
    cfg = Config_init_default;
    const char msg[] = "Config DEFAULT\n";
    Serial1.Send((uint8_t *)msg, sizeof(msg));
  }

  // TODO: Consider running imu and balacning loop outside of interrupt. Set a
  // have_new_data flag in the interrupt, copy data to output if flag was
  // cleared. Ignore data if flag was not cleared. Reset flag in main loop when
  // data is copied.

  // TODO 2:  while(1) must run at specific loop time for any LPFs and pids
  // (footpad, etc to work correctly). Fix the loop time

  GenericOut status_led(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_4, 1);  // red
  status_led.init();
  status_led.setState(0);

  PwmOut::InitAll();
  PwmOut pmw1(1);
  pmw1.set(500);

  PwmOut pwm2(2);
  pwm2.set(500);

  PwmOut pwm3(3);
  pwm3.set(500);

  GenericOut dir1(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_7, false);
  dir1.init();

  GenericOut dir2(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_8, false);
  dir2.init();

  GenericOut dir3(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_9, false);
  dir3.init();

  IMU imu(&cfg);
  AngleGuard angle_guard(imu, &cfg.balance_settings);
  InitWaiter waiter(&status_led, &imu,
                    &angle_guard);  // wait for angle. Wait for pads too?
  accGyro.setListener(&waiter);
  accGyro.init(cfg.balance_settings.global_gyro_lpf);
  accGyro.setAccGyroOrientation(cfg.callibration.x_offset, cfg.callibration.y_offset, cfg.callibration.z_offset);

  GenericOut beeper(RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_12, true);
  beeper.init(true);

  waiter.waitForAccGyroCalibration();

  GenericOut green_led(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_3, true);
  green_led.init();

  initEncoders();

  //	GenericOut debug_out(RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_11, false);
  //	debug_out.init();

  Guard *guards[] = {&angle_guard};
  int guards_count = sizeof(guards) / sizeof(Guard *);

  static BoardController main_ctrl(&cfg, imu, status_led, beeper, guards,
                            guards_count, green_led, &pmw1, &dir1, &pwm2, &dir2, &pwm3, &dir3);

  global_main_ctrl = &main_ctrl;

  accGyro.setListener(&main_ctrl);

  InitDecoderTimer();

  comms.Init(&Serial1);
  uint16_t last_check_time = 0;

  write_pos = 0;
  read_pos = 0;
  while (1) {  // background work
    IWDG_ReloadCounter();

    // green_led.setState(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1));

    if ((uint16_t)(millis() - last_check_time) > 100u) {
      last_check_time = millis();

      switch (debug_stream_type) {
        case 1:
          debug[write_pos++] = (int8_t)imu.angles[ANGLE_DRIVE];
          break;
        case 2:
          debug[write_pos++] = (int8_t) (main_ctrl.right / 10);
          break;
        case 3:
          debug[write_pos++] = (int8_t) (main_ctrl.fwd / 10);
          break;
      }

      if (write_pos >= sizeof(debug)) write_pos = 0;
    }

    uint8_t comms_msg = comms.update();
    switch (comms_msg) {
      case RequestId_READ_CONFIG: {
        int16_t data_len = saveProtoToBuffer(scratch, sizeof(scratch),
                                             Config_fields, &cfg, &Serial1);
        if (data_len > 0) {
          comms.SendMsg(ReplyId_CONFIG, scratch, data_len);
        } else {
          comms.SendMsg(ReplyId_GENERIC_FAIL);
        }
        break;
      }

      case RequestId_SET_DEBUG_STREAM_ID:
      	if (comms.data_len() == 1) {
      		debug_stream_type = comms.data()[0];
      	} else {
      		comms.SendMsg(ReplyId_GENERIC_FAIL);
      	}
      	break;

      case RequestId_GET_DEBUG_BUFFER: {
        // TODO: make sure it all fits in TX buffer or an overrun will occur
        if (write_pos < read_pos) {
          int data_len_tail = sizeof(debug) - read_pos;
          if (data_len_tail > 0) {
            comms.SendMsg(ReplyId_DEBUG_BUFFER, debug + read_pos,
                          data_len_tail);
            read_pos = 0;
          }
        }

        int rem_len = write_pos - read_pos;
        if (rem_len > 0) {
          comms.SendMsg(ReplyId_DEBUG_BUFFER, debug + read_pos, rem_len);
          read_pos = write_pos;
        }
        break;
      }

      case RequestId_WRITE_CONFIG: {
        bool good =
            readSettingsFromBuffer(&cfg, comms.data(), comms.data_len());
        if (good) {
          comms.SendMsg(ReplyId_GENERIC_OK);
          accGyro.setAccGyroOrientation(cfg.callibration.x_offset, cfg.callibration.y_offset, cfg.callibration.z_offset);
        }
        else
        {
          comms.SendMsg(ReplyId_GENERIC_FAIL);
        }
        break;
      }

      case RequestId_GET_STATS: {
        Stats stats = Stats_init_default;
        stats.drive_angle = imu.angles[ANGLE_DRIVE];
        stats.stear_angle = imu.angles[ANGLE_STEER];
        stats.pad_pressure1 = main_ctrl.m1_speed_lpf_.getVal() * 100;
        stats.pad_pressure2 = main_ctrl.m2_speed_lpf_.getVal() * 100;
        stats.batt_current = main_ctrl.decoders_[0].errors() + main_ctrl.decoders_[1].errors() + main_ctrl.decoders_[2].errors();
        stats.batt_voltage = main_ctrl.m3_speed_lpf_.getVal() * 100;

        int16_t data_len =
            saveProtoToBuffer(scratch, sizeof(scratch), Stats_fields, &stats);
        if (data_len != -1) {
          comms.SendMsg(ReplyId_STATS, scratch, data_len);
        } else {
          comms.SendMsg(ReplyId_GENERIC_FAIL);
        }
        break;
      }


      case RequestId_SAVE_CONFIG:
        saveSettingsToFlash(cfg);
        comms.SendMsg(ReplyId_GENERIC_OK);
        break;

      case RequestId_GET_CONFIG_DESCRIPTOR:
        comms.SendMsg(ReplyId_CONFIG_DESCRIPTOR, (uint8_t*)cf_data, (int)cf_data_e - (int)cf_data);
        break;
    }
  }
}