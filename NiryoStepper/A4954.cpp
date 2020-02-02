/*
    A4954.cpp
    Copyright (C) 2017 Niryo

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "A4954.h"
#include "wiring_private.h"

static uint8_t pinState=0;

#pragma GCC push_options
#pragma GCC optimize ("-Ofast")

#define DAC_MAX (0x01FFL)
// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  //int32_t t0=1000;
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
  {
    //    t0--;
    //    if (t0==0)
    //    {
    //      break;
    //    }
    //    delay(1);
  }
}

static inline void bridge1(int state)
{
  if (state==0)
  {
    PORT->Group[g_APinDescription[IN_1].ulPort].PINCFG[g_APinDescription[IN_1].ulPin].bit.PMUXEN = 0;
    GPIO_OUTPUT(IN_1);//pinMode(IN_1,OUTPUT);
    GPIO_OUTPUT(IN_2);//pinMode(IN_2,OUTPUT);
    GPIO_HIGH(IN_1);// digitalWrite(IN_1, HIGH);
    GPIO_LOW(IN_2);//digitalWrite(IN_2, LOW);
    //pinPeripheral(IN_2, PIO_TIMER_ALT);
    pinState=(pinState & 0x0C) | 0x1;
  }
  if (state==1)
  {
    PORT->Group[g_APinDescription[IN_2].ulPort].PINCFG[g_APinDescription[IN_2].ulPin].bit.PMUXEN = 0;
    GPIO_OUTPUT(IN_2);//pinMode(IN_2,OUTPUT);
    GPIO_OUTPUT(IN_1);pinMode(IN_1,OUTPUT);
    GPIO_LOW(IN_1);//digitalWrite(IN_1, LOW);
    GPIO_HIGH(IN_2);//digitalWrite(IN_2, HIGH);
    //pinPeripheral(IN_1, PIO_TIMER);
    pinState=(pinState & 0x0C) | 0x2;
  }
  if (state==3)
  {
    GPIO_LOW(IN_1);
    GPIO_LOW(IN_2);
    //digitalWrite(IN_1, LOW);
    //digitalWrite(IN_2, LOW);
  }
}

static inline void bridge2(int state)
{
  if (state==0)
  {
    PORT->Group[g_APinDescription[IN_3].ulPort].PINCFG[g_APinDescription[IN_3].ulPin].bit.PMUXEN = 0;
    GPIO_OUTPUT(IN_3); //pinMode(IN_3,OUTPUT);
    GPIO_OUTPUT(IN_4);//pinMode(IN_4,OUTPUT);
    GPIO_HIGH(IN_3);//digitalWrite(IN_3, HIGH);
    GPIO_LOW(IN_4);//digitalWrite(IN_4, LOW);
    //pinPeripheral(IN_4, PIO_TIMER_ALT);
    pinState=(pinState & 0x03) | 0x4;
  }
  if (state==1)
  {
    PORT->Group[g_APinDescription[IN_4].ulPort].PINCFG[g_APinDescription[IN_4].ulPin].bit.PMUXEN = 0;
    GPIO_OUTPUT(IN_4);//pinMode(IN_4,OUTPUT);
    GPIO_OUTPUT(IN_3);//pinMode(IN_3,OUTPUT);
    GPIO_LOW(IN_3);//digitalWrite(IN_3, LOW);
    GPIO_HIGH(IN_4);//digitalWrite(IN_4, HIGH);
    //pinPeripheral(IN_3, PIO_TIMER_ALT);
    pinState=(pinState & 0x03) | 0x8;
  }
  if (state==3)
  {
    GPIO_LOW(IN_3);
    GPIO_LOW(IN_4);
    //digitalWrite(IN_3, LOW);
    //digitalWrite(IN_4, LOW);
  }
}

static void enableTCC0(uint8_t percent)
{
#ifdef MECHADUINO_HARDWARE
  return;
#else
  Tcc* TCCx = TCC0 ;


  uint32_t ulValue=((uint32_t)(100-percent)*480)/100;
  //ERROR("Enable TCC0");

  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;

  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

  //ERROR("Setting TCC %d %d",ulValue,ulPin);
  TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  syncTCC(TCCx);

  // Set TCx as normal PWM
  TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
  syncTCC(TCCx);

  // Set TCx in waveform mode Normal PWM
  TCCx->CC[1].reg = (uint32_t)ulValue; //ch5 //IN3
  syncTCC(TCCx);

  TCCx->CC[2].reg = (uint32_t)ulValue; //ch6 //IN4
  syncTCC(TCCx);

  TCCx->CC[3].reg = (uint32_t)ulValue; //ch7  //IN2
  syncTCC(TCCx);

  TCCx->CC[1].reg = (uint32_t)ulValue; //ch1 == ch5 //IN1

  syncTCC(TCCx);

  // Set PER to maximum counter value (resolution : 0xFF)
  TCCx->PER.reg = DAC_MAX;
  syncTCC(TCCx);

  // Enable TCCx
  TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
  syncTCC(TCCx);
  //ERROR("Enable TCC0 DONE");
#endif
}

static void setDAC(uint32_t DAC1, uint32_t DAC2)
{
  TCC1->CC[1].reg = (uint32_t)DAC1; //D9 PA07 - VREF12
  syncTCC(TCC1);
  TCC1->CC[0].reg = (uint32_t)DAC2; //D4 - VREF34
  syncTCC(TCC1);


}

static void setupDAC(void)
{
  Tcc* TCCx = TCC1 ;

  pinPeripheral(VREF_2, PIO_TIMER_ALT);
  pinPeripheral(VREF_1, PIO_TIMER);

  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC0_TCC1 )) ;

  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

  //ERROR("Setting TCC %d %d",ulValue,ulPin);
  TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  syncTCC(TCCx);

  // Set TCx as normal PWM
  TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
  syncTCC(TCCx);

  // Set TCx in waveform mode Normal PWM
  TCCx->CC[1].reg = (uint32_t)0;
  syncTCC(TCCx);

  TCCx->CC[0].reg = (uint32_t)0;
  syncTCC(TCCx);

  // Set PER to maximum counter value (resolution : 0xFFF = 12 bits)
  // =48e6/2^12=11kHz frequency
  TCCx->PER.reg = DAC_MAX;
  syncTCC(TCCx);

  // Enable TCCx
  TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
  syncTCC(TCCx);

}

int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}

void init_driver()
{
  digitalWrite(VREF_2,LOW);
  pinMode(VREF_2, OUTPUT);
  digitalWrite(VREF_1,LOW);
  pinMode(VREF_1, OUTPUT);
  digitalWrite(IN_4,LOW);
  pinMode(IN_4, OUTPUT);
  digitalWrite(IN_3,LOW);
  pinMode(IN_3, OUTPUT);
  digitalWrite(IN_2,LOW);
  pinMode(IN_2, OUTPUT);
  digitalWrite(IN_1,LOW);
  pinMode(IN_1, OUTPUT);
  
  enableTCC0(90);
  setupDAC();
}

void output(long theta, int effort) {
  int angle_1;
  int angle_2;
  int v_coil_A;
  int v_coil_B;
  
  int sin_coil_A;
  int sin_coil_B;
  int phase_multiplier = 500;

  SerialUSB.print("theta : ");
  SerialUSB.print(theta);
  SerialUSB.print(", effort : ");
  SerialUSB.print(effort);
  
  angle_1 = mod((phase_multiplier * theta)/1000 , 3600);
  angle_2 = mod((phase_multiplier * theta)/1000 +900, 3600);

  SerialUSB.print(", angle_1 : ");
  SerialUSB.print(angle_1);
  SerialUSB.print(", angle_2 : ");
  SerialUSB.print(angle_2); 
  
  sin_coil_A = sin_1[angle_1];
  sin_coil_B = sin_1[angle_2];
  
  v_coil_A = ((effort * sin_coil_A) / 1024);
  v_coil_B = ((effort * sin_coil_B) / 1024);

  SerialUSB.print(", VREF_1 : ");
  SerialUSB.print(v_coil_A);
  SerialUSB.print(", VREF_2 : ");
  SerialUSB.println(v_coil_B); 
  setDAC(abs(v_coil_A), abs(v_coil_B));
  
  if (v_coil_A >= 0)  {
    bridge1(1);
  }
  else  {
    bridge1(0);
  }

  if (v_coil_B >= 0)  {
    bridge2(1);
  }
  else  {
    bridge2(0);
  }
}

/*
 * Setting all Pins to HIGH will
 * give more "resistance torque" to the motor
 * --> Useful on Niryo One so the axis of the motor
 * doesn't fall abruptely
 */
void relaxed_mode_with_resistance() {
  IN_1_HIGH();
  IN_2_HIGH();
  IN_3_HIGH();
  IN_4_HIGH();
}

#pragma GCC pop_options
