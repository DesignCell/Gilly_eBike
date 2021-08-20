#include <Arduino.h>

#define SERIAL_DEBUG

#define LATCH 2
#define BREAK 3
#define THROTTLE A0
#define THROTTLE_TOP A1
#define THROTTLE_PWR 6
#define THROTTLE_TOP_PWR 7


// BTS7960
#define R_EN  4  // R_EN : OUTPUT, Forward Drive Enable, Active High, Low Disable
#define R_PWM 5  // RPWM : OUTPUT, Forward PWM, Active High
//#define R_IS  6  // R_IS : INPUT, Forward Drive Side Current Alarm,

#define L_EN  8  // L_EN : OUTPUT, Reverse Drive Enable, Active High, Low Disable
#define L_PWM 9  // LPWM : OUTPUT, Reverse PWM, Active High
//#define L_IS  9  // L_IS : INPUT, Reverse Drive Side Current Alarm.

                  // Vcc  : INPUT, +5VDC Power Supply to Microcontroller
                  // Gnd  : INPUT, Gound Power Supply to Microcontroller


uint16_t interval_hz = 10;
uint32_t interval = 1000 / interval_hz; // Millis
uint32_t  pMillis = millis();

uint32_t idleTimeout = 20000;
uint32_t idleTime;

float accel_adder = 255 / 3 / interval_hz;  // Seconds to full speed
float accel_sub   = 255 / 1 / interval_hz;  // Seconds to full stop
float spdPWM = 0;

int spdUpper;
int spdLower = 20; // Adjust to min run speed
int spdStart = 0; // Adjust to Start from stall
int throttleLower = 0; // Adjust to min Throttle w/padding
int throttleUpper = 17; // Adjust to Max Throttle
int throttlePos;  // Analog Read Throttle Position
int throttleTop = 100;  // Analog Read Throttle Top Speed
bool breakStatus = false;  // DI break status

void setup() {
  pinMode(LATCH,OUTPUT);
  digitalWrite(LATCH,LOW); // Low = Allow Latch

  pinMode(BREAK,INPUT_PULLUP);

  pinMode(THROTTLE_PWR, OUTPUT);
  digitalWrite(THROTTLE_PWR,HIGH);
  pinMode(THROTTLE, INPUT);

  pinMode(THROTTLE_TOP_PWR, OUTPUT);
  digitalWrite(THROTTLE_TOP_PWR,HIGH);
  pinMode(THROTTLE_TOP, INPUT);

  pinMode(R_EN ,OUTPUT);  // R_EN : Forward Drive Enable, Active High, Low Disable
  pinMode(R_PWM,OUTPUT);  // RPWM : Forward PWM, Active High
  //pinMode(R_IS ,INPUT);   // R_IS : Forward Drive Side Current Alarm
  pinMode(L_EN ,OUTPUT);  // L_EN : Reverse Drive Enable, Active High, Low Disable
  pinMode(L_PWM,OUTPUT);  // LPWM : Reverse PWM, Active High
  //pinMode(L_IS ,INPUT);   // L_IS : Reverse Drive Side Current Alarm

  //Disable Motors
  analogWrite(R_PWM,0);
  analogWrite(L_PWM,0);
  digitalWrite(R_EN,LOW);
  digitalWrite(L_EN,LOW);

  // Debug console
  #ifdef SERIAL_DEBUG
  Serial.begin(115000);
  #endif

}

void serialPrint()
{
  char sPrint[50];
  int num1 = breakStatus;
  int num2 = analogRead(THROTTLE);
  int num3 = throttleTop;
  int num4 = throttlePos;
  int num5 = spdPWM;


  sprintf(sPrint, "%d, %d, %d, %d, %d", num1, num2, num3, num4, num5);
  
  //Print
	Serial.println(sPrint);
}

void DisconnectPwr()
{
  // Kill Outouts
  digitalWrite(THROTTLE_PWR, LOW);
  digitalWrite(THROTTLE_TOP_PWR, LOW);

  digitalWrite(R_EN,LOW);
  analogWrite(R_PWM,0);
  digitalWrite(L_EN,LOW);
  analogWrite(L_PWM,0);
  

  delay(10);
  digitalWrite(LATCH,HIGH);
}

void loop() {
  if (millis() - pMillis > interval)
  {
    pMillis = millis();

    // Read Inputs
    throttleTop = map(analogRead(THROTTLE_TOP),0, 1023, 0,254); // 0-1023
    throttlePos = map(analogRead(THROTTLE), throttleLower, throttleUpper, 0, throttleTop);     // min-max
    breakStatus = digitalRead(BREAK);

    if (throttlePos < spdLower)
      throttlePos = 0;
      
    if (throttlePos != 0 || breakStatus == 0)
      idleTime = millis();

    if (breakStatus == 1)
    {
      if (spdPWM < throttlePos) 
        {
          spdPWM += accel_adder;
          if (spdPWM > throttleTop) spdPWM = throttleTop;
          if (spdPWM < spdLower) spdPWM = spdLower;
        }
      else
      {
        spdPWM -= accel_adder / 1.5;
        if (spdPWM < spdLower) spdPWM = 0;
      }
    }
    else
    {
        spdPWM -= accel_sub;
        if (spdPWM < spdLower) spdPWM = 0;
    }

    digitalWrite(R_EN,HIGH);
    analogWrite(R_PWM,spdPWM); //FWD PWM
    digitalWrite(L_EN,HIGH);
    analogWrite(L_PWM,0);



    #ifdef SERIAL_DEBUG
    serialPrint();
    #endif
  }
  
  if(millis() - idleTime > idleTimeout) DisconnectPwr();
}