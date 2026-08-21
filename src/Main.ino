/*
// Created by PAZI88 | Pasi Kemppainen | Sipa's bimmer garage https://github.com/pazi88
This code controls BMW Electric Power Steering on E-Series BMWs (EPS Control). The way this works is that the Arduino is connected to CAN bus between the car and the EPS module. 
And sends the CAN messages normally between those. But then spoofs the messages that affect the steering assist amount, so that we can control it independently.
*/

// https://github.com/pazi88/STM32_CAN
#include "STM32_CAN.h"

//Bitsetting macros
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))

static CAN_message_t CAN_primaryMsg;
static CAN_message_t CAN_secondaryMsg;
static CAN_message_t CAN_modMsg;

STM32_CAN CanPrimary( CAN1, ALT_2, RX_SIZE_64, TX_SIZE_64 );  //Use PD0/1 pins for CAN1.
STM32_CAN CanSecondary( CAN2, DEF, RX_SIZE_64, TX_SIZE_64 );  //Use PB12/13 pins for CAN2.

int analogPin = PA0;
int analogLevel;
uint8_t AssistLevel;

void ReadAssistLevel()
{
  analogLevel = analogRead(analogPin);
  Serial.println(analogLevel);
  
  if ( analogLevel < 105 )
  {
    AssistLevel = 0;
  } 
  else if ( analogLevel < 300 )
  {
    AssistLevel = 1;
  }
  else if ( analogLevel < 505 )
  {
    AssistLevel = 2;
  }
  else if ( analogLevel < 710 )
  {
    AssistLevel = 3;
  }
  else if ( analogLevel < 915 )
  {
    AssistLevel = 4;
  }
  else
  {
    AssistLevel = 5;
  }
}

static uint8_t calcCanChecksum8(uint16_t base, const uint8_t *buf, uint8_t len)
{
  // Created by https://github.com/Juseus
  // BN2000 8-bit checksums in the validated DSC/SZL/KOMBI messages use one
  // end-around fold only. Do not repeatedly fold: sums such as 0x1FF must
  // become 0x00, which matches the E61/E65/E81 logs.
  uint16_t sum = base;

  for (uint8_t i = 0U; i < len; i++)
  {
    sum = (uint16_t)(sum + buf[i]);
  }

  sum = (uint16_t)((sum >> 8) + (sum & 0xFFU));

  return (uint8_t)(sum & 0xFFU);
}
 
void SetAssist()  // Adjusts the CAN speed messaga to set the adjust level
{
  CAN_modMsg = CAN_primaryMsg;
  // https://www.ms4x.net/index.php?title=CAN_Bus_ID_0x1A0
  switch (AssistLevel) {
    case 0:  // 150KM/H
      CAN_modMsg.buf[0] =  0xDC;
      CAN_modMsg.buf[1] =  0x95;
      break;
    case 1:  // 100KM/H
      CAN_modMsg.buf[0] =  0xE8;
      CAN_modMsg.buf[1] =  0x93;
      break;
    case 2:  // 75KM/H
      CAN_modMsg.buf[0] =  0xEE;
      CAN_modMsg.buf[1] =  0x92;
      break;
    case 3:  // 50KM/H
      CAN_modMsg.buf[0] =  0xF4;
      CAN_modMsg.buf[1] =  0x91;
      break;
    case 4:  // 25KM/H
      CAN_modMsg.buf[0] =  0xFA;
      CAN_modMsg.buf[1] =  0x90;
      break;
    case 5:  // 0KM/H
      CAN_modMsg.buf[0] =  0x00;
      CAN_modMsg.buf[1] =  0x80;
      break;
    default:   // 0KM/H
      CAN_modMsg.buf[0] =  0x00;
      CAN_modMsg.buf[1] =  0x80;
      break;
  // checksum
  CAN_modMsg.buf[7] = calcCanChecksum8(CAN_modMsg.id, CAN_modMsg.buf, 7U);
  }
}

void EnableAssist()  // Tell the EPS that Engine is running, so that we have constant assist
{
  // https://www.ms4x.net/index.php?title=Siemens_MS45_CAN_Bus_(BN2000)#ENGINE_1_0x1D0
  CAN_modMsg = CAN_primaryMsg;
  BIT_SET(CAN_modMsg.buf[2],5);
  BIT_CLEAR(CAN_modMsg.buf[2],4);
}

void setup(){  
  CanPrimary.begin();
  CanPrimary.setBaudRate(500000);
  CanSecondary.begin();
  CanSecondary.setBaudRate(500000);
  
  Serial.begin(115200);  // for debug
  AssistLevel = 0;

  // setup hardware timer to read the analog input to set assist level in 5Hz pace (doesn't need to be fast)
#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif
  HardwareTimer *SendTimer = new HardwareTimer(Instance);
  SendTimer->setOverflow(5, HERTZ_FORMAT); // 5 Hz
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  SendTimer->attachInterrupt(1, ReadAssistLevel);
  SendTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else //2.0 forward
  SendTimer->attachInterrupt(ReadAssistLevel);
#endif
  SendTimer->resume();
}
 

// main loop
void loop() {
  // if there is messages on car side CAN bus, read those and send those to EPS side CAN bus
  if (CanPrimary.read(CAN_primaryMsg) ) {
    Serial.print("Car Msg ID: ");
    Serial.print(CAN_primaryMsg.id, HEX);
	// The VSS message needs to be spoofed to adjust assist
	if (CAN_primaryMsg.id == 0x1A0) {
		SetAssist();
		CanSecondary.write(CAN_modMsg);
	}
	// Engine status bit needs to be set running, to enable assist all the time
	else if (CAN_primaryMsg.id == 0x1D0) {
		EnableAssist();
		CanSecondary.write(CAN_modMsg);
	}
	else {
	  CanSecondary.write(CAN_primaryMsg);
	}
  }
  
  // if there is messages on EPS side CAN bus, read those and send those to car side CAN bus
  if (CanSecondary.read(CAN_secondaryMsg) ) {
    Serial.print("Car Msg ID: ");
    Serial.print(CAN_secondaryMsg.id, HEX);
	CanPrimary.write(CAN_secondaryMsg);
  }
}