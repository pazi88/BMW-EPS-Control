# 1 "C:\\Users\\kemppain\\AppData\\Local\\Temp\\tmph1wqxa5c"
#include <Arduino.h>
# 1 "C:/Users/kemppain/Documents/GitHub/BMW-EPS-Control/src/Main.ino"





#include "STM32_CAN.h"

static CAN_message_t CAN_primaryMsg;
static CAN_message_t CAN_secondaryMsg;
static CAN_message_t CAN_modMsg;

STM32_CAN CanPrimary( CAN1, ALT_2, RX_SIZE_64, TX_SIZE_64 );
STM32_CAN CanSecondary( CAN2, DEF, RX_SIZE_64, TX_SIZE_64 );

int analogPin = PA0;
int analogLevel;
uint8_t AssistLevel;
void ReadAssistLevel();
void SetAssist();
void setup();
void loop();
#line 19 "C:/Users/kemppain/Documents/GitHub/BMW-EPS-Control/src/Main.ino"
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

void SetAssist()
{
  CAN_modMsg = CAN_primaryMsg;

  switch (AssistLevel) {
    case 0:
      CAN_modMsg.buf[0] = 0xDC;
      CAN_modMsg.buf[1] = 0x95;
      break;
    case 1:
      CAN_modMsg.buf[0] = 0xE8;
      CAN_modMsg.buf[1] = 0x93;
      break;
    case 2:
      CAN_modMsg.buf[0] = 0xEE;
      CAN_modMsg.buf[1] = 0x92;
      break;
    case 3:
      CAN_modMsg.buf[0] = 0xF4;
      CAN_modMsg.buf[1] = 0x91;
      break;
    case 4:
      CAN_modMsg.buf[0] = 0xFA;
      CAN_modMsg.buf[1] = 0x90;
      break;
    case 5:
      CAN_modMsg.buf[0] = 0x00;
      CAN_modMsg.buf[1] = 0x80;
      break;
    default:
      CAN_modMsg.buf[0] = 0x00;
      CAN_modMsg.buf[1] = 0x80;
      break;

  CAN_modMsg.buf[7] = ( CAN_modMsg.buf[0] + CAN_modMsg.buf[1] + CAN_modMsg.buf[2] + CAN_modMsg.buf[3] + CAN_modMsg.buf[4] + CAN_modMsg.buf[5] + CAN_modMsg.buf[6] + 0xA4 ) & 0xFF;
  }
}

void setup(){
  CanPrimary.begin();
  CanPrimary.setBaudRate(500000);
  CanSecondary.begin();
  CanSecondary.setBaudRate(500000);

  Serial.begin(115200);
  AssistLevel = 0;


#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif
  HardwareTimer *SendTimer = new HardwareTimer(Instance);
  SendTimer->setOverflow(5, HERTZ_FORMAT);
#if ( STM32_CORE_VERSION_MAJOR < 2 )
  SendTimer->attachInterrupt(1, ReadAssistLevel);
  SendTimer->setMode(1, TIMER_OUTPUT_COMPARE);
#else
  SendTimer->attachInterrupt(ReadAssistLevel);
#endif
  SendTimer->resume();
}



void loop() {

  if (CanPrimary.read(CAN_primaryMsg) ) {
    Serial.print("Car Msg ID: ");
    Serial.print(CAN_primaryMsg.id, HEX);

 if (CAN_primaryMsg.id == 0x1A0) {
  SetAssist();
  CanSecondary.write(CAN_modMsg);
 }
 else {
   CanSecondary.write(CAN_primaryMsg);
 }
  }


  if (CanSecondary.read(CAN_secondaryMsg) ) {
    Serial.print("Car Msg ID: ");
    Serial.print(CAN_secondaryMsg.id, HEX);
 CanPrimary.write(CAN_secondaryMsg);
  }
}