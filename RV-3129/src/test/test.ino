/*
  Test Arduino sketch for RV3129 RTC module.
*/

#include "RV3129.h"

RV3129 rtc;

int sec = 2;
int minute = 59;
uint8_t hour = 9;
int date = 25;
int month = 5;
int year = 2014;
int day = 5;

unsigned long startTime;

void setup() {

  Wire.begin();

  Serial.begin(115200);
  Serial.println("Read Time from RTC Example");

  if (rtc.begin() == false) {
    Serial.println("Something went wrong, check wiring");
  }

  rtc.set12Hour();

//  if (rtc.setToCompilerTime() == false) {
//    Serial.println("Something went wrong setting the time");
//  }

  if (rtc.setTime(sec, minute, hour, date, month, year, day) == false) {
    Serial.println("Something went wrong setting the time");
  }

  Serial.println("RTC online!");

  // set alarm for one minute after init time
  if (rtc.setAlarm(sec, (minute + 1), hour, date, day, month, year) == false) {
    Serial.println("Failed to set alarm.");
  }
  // set alarm mode
  uint8_t mode = 0b0000011;
  if (rtc.enableDisableAlarm(mode) == false) {
    Serial.println("Failed to set alarm enable/disable bits.");
  }
  // get alarm mode
  mode = rtc.getAlarmMode();
  if (mode > 0x7f) {
    Serial.println("Failed to get alarm mode.");
  }
  else {
    Serial.print("Alarm Mode: ");
    Serial.println(mode, BIN);
  }

  // enable alarm interrupt
  if (rtc.enableAlarmINT(true) == false) {
    Serial.println("Failed to enable alarm interrupt.");
  }
  // check alarm INT enabled
  Serial.print("Alarm INT Enabled: ");
  Serial.println(rtc.alarmINTEnabled());
  // disable alarm INT
  if (rtc.enableAlarmINT(false) == false) {
    Serial.println("Failed to enable alarm interrupt.");
  }
  // check alarm INT enabled
  Serial.print("Alarm INT Enabled: ");
  Serial.println(rtc.alarmINTEnabled());
  // enable alarm interrupt
  if (rtc.enableAlarmINT(true) == false) {
    Serial.println("Failed to enable alarm interrupt.");
  }
  // check alarm INT enabled
  Serial.print("Alarm INT Enabled: ");
  Serial.println(rtc.alarmINTEnabled());
}

void loop() {
  if (rtc.updateTime() == false) { //Updates the time variables from RTC
    Serial.println("RTC failed to update");
  }
  else {
    String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format
    String currentTime = rtc.stringTime(); //Get the time

    char currentTemp[15];
    sprintf(currentTemp, "Deg. C: %s", rtc.stringTemp());

    Serial.print(currentDate);
    Serial.print(" ");
    Serial.println(currentTime);
    Serial.println(currentTemp);

    bool alarmINT = rtc.getAlarmFlag();

    Serial.print("Alarm INT Flag State: ");
    Serial.println(alarmINT);

    if (alarmINT) {
      rtc.systemReset();
    }
  }

  delay(1000);
}
