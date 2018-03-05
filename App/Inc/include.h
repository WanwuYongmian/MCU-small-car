#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO�ڲ���
#include  "MK60_uart.h"     //����
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //�͹��Ķ�ʱ��(��ʱ)
#include  "MK60_i2c.h"      //I2C
#include  "MK60_spi.h"      //SPI
#include  "MK60_ftm.h"      //FTM
#include  "MK60_pit.h"      //PIT
#include  "MK60_rtc.h"      //RTC
#include  "MK60_adc.h"      //ADC
#include  "MK60_dac.h"      //DAC
#include  "MK60_dma.h"      //DMA
#include  "MK60_FLASH.h"    //FLASH


#include  "VCAN_LED.H"          //LED
#include  "VCAN_KEY.H"          //KEY
#include  "VCAN_MMA7455.h"      //������ٶ�MMA7455
#include  "VCAN_NRF24L0.h"      //����ģ��NRF24L01+
#include  "VCAN_RTC_count.h"    //RTC ʱ��ת��
#include  "VCAN_camera.h"       //����ͷ��ͷ�ļ�



#include "WJK_SD.h"
#include "LQ_SGP18T.h"
#include "bmp_lib.h"
#include "mma845x.h"
#include "MPU6050.h"
#include "l3g4200d_driver.h"
#include "buzz.h"
#include "key.h"
#include "SDS.h"
#include "FreeCars_uart.h"
#include "LQ12864.h"
#include "acc_gyro.h"
#include "PWM.h" 
#include "turn.h"
#include "speed.h"

#include "sd_use.h"
#include "bluetooth_use.h"

#include "initial.h"
#include "ccd.h"

#include  "VCAN_computer.h"     //�๦�ܵ�������
#endif  //__INCLUDE_H__
