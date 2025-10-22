#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <Servo.h>
#include <avr/pgmspace.h>

#define ONE_WIRE_BUS 2
#define SERVO_PIN 3

#define FAN_PORT PORTB
#define FAN_DDR DDRB
#define FAN_BIT PB3 // PIN D11

#define PH_LED_PORT PORTD
#define PH_LED_DDR DDRD
#define PH_LED_BIT PD7 // PIN D7

#define BUTTON_PORT PORTD
#define BUTTON_DDR DDRD
#define BUTTON_PIN PIND
#define BUTTON_BIT PD5 // PIN D5

#define ALERT_LED_PORT PORTD
#define ALERT_LED_DDR DDRD
#define ALERT_LED_BIT PD7 // PIN D7

#define SAMPLES 10
const float PH_SLOPE PROGMEM = -8.048480;
const float PH_OFFSET PROGMEM = 20.661462;

const uint8_t VALVE_CLOSED_POSITION = 0;
const uint8_t VALVE_OPEN_POSITION = 30;
const uint8_t FEEDING_DURATION = 75;

struct FeedingTime
{
  uint8_t hour;
  uint8_t minute;
};

const FeedingTime FEEDING_SCHEDULE[] = {
    {9, 30},
    {9, 31}};
const uint8_t NUM_SCHEDULED_FEEDINGS = 2;

uint8_t lastFeedDay = 0;
uint8_t dailyFeedingCount = 0;
bool scheduledFeedingDone[2];
uint8_t currentFeedingType = 0;

const char STR_RTC_ERROR[] PROGMEM = "RTC Error!";
const char STR_RTC_LOST_POWER[] PROGMEM = "RTC lost power - battery may be dead!";
const char STR_RTC_DIAG[] PROGMEM = "=== RTC Diagnostics ===";
const char STR_RTC_TICKING[] PROGMEM = "✓ RTC ticking normally";
const char STR_RTC_ISSUE[] PROGMEM = "✗ RTC timing issue detected!";
const char STR_FEEDING_MANUAL[] PROGMEM = "Manual feeding requested";
const char STR_FEEDING_SCHEDULED[] PROGMEM = "Scheduled feeding started";
const char STR_FEEDING_COMPLETED[] PROGMEM = "Feeding completed - daily count updated";
const char STR_FEEDING_SKIPPED[] PROGMEM = "Feeding skipped - already fed this schedule";
const char STR_VALVE_OPEN[] PROGMEM = "Feeding: Valve opened";
const char STR_VALVE_CLOSE[] PROGMEM = "Feeding: Valve closed";
const char STR_FAN_ON[] PROGMEM = "Fans ON - Temperature too high";
const char STR_FAN_OFF[] PROGMEM = "Fans OFF - Temperature normal";
const char STR_SYSTEM_INIT[] PROGMEM = "Smart Aquarium System Initialized";

void printPROGMEM(const char *str);
void printPROGMEMln(const char *str);
void printDateTime(DateTime dt);
void checkScheduledFeeding();
void handleNonBlockingRTCDiagnostics();
void setupLCDLabels();
void updatePhReading();
void updateTemperatureReading();
void handleFeeding();
void startFeeding(uint8_t feedingType);
void updateFanControl();
void updateAlertLED();
void updateDisplay();
void updateSerialOutput();
uint16_t readPH_ADC();

LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
RTC_DS3231 rtc;
Servo feederServo;

uint8_t systemFlags = 0;
#define FLAG_FAN_RUNNING 0
#define FLAG_FEEDING 1
#define FLAG_VALVE_OPEN 2
#define FLAG_TEMP_REQUESTED 3
#define FLAG_LCD_INIT 4
#define FLAG_RTC_DIAG_DONE 5

#define SET_FLAG(flag) (systemFlags |= _BV(flag))
#define CLEAR_FLAG(flag) (systemFlags &= ~_BV(flag))
#define CHECK_FLAG(flag) (systemFlags & _BV(flag))

uint32_t feedingStartTime = 0;
uint32_t lastPhSampleTime = 0;
uint32_t lastTempRequestTime = 0;
uint32_t lastDisplayUpdate = 0;
uint32_t lastSerialOutput = 0;
uint32_t rtcDiagStartTime = 0;

volatile uint8_t buttonPressedFlag = 0;
volatile uint8_t lastPortDState = 0;
volatile uint32_t lastInterruptTime = 0;

const uint16_t phSampleInterval = 50;
const uint16_t tempRequestInterval = 1000;
const uint16_t displayUpdateInterval = 500;
const uint16_t serialOutputInterval = 1000;
const uint32_t DEBOUNCE_DELAY_ISR = 300UL;

uint32_t phSampleSum = 0;
uint8_t phSampleCount = 0;
uint16_t currentPH_scaled = 700;

uint16_t currentTemp_scaled = 250;

uint8_t alertBlinks = 0;
uint32_t lastBlinkToggleMillis = 0;
uint8_t alertLedIsOn = 0;
uint8_t currentBlinkCount = 0;
uint32_t currentAlertCycleStart = 0;

DateTime rtcDiagStartDateTime;

inline void setFanOn()
{
  FAN_PORT |= _BV(FAN_BIT);
}

inline void setFanOff()
{
  FAN_PORT &= ~_BV(FAN_BIT);
}

inline void setAlertLedOn()
{
  ALERT_LED_PORT |= _BV(ALERT_LED_BIT);
}

inline void setAlertLedOff()
{
  ALERT_LED_PORT &= ~_BV(ALERT_LED_BIT);
}

inline uint8_t readButton()
{
  return (BUTTON_PIN & _BV(BUTTON_BIT)) ? 1 : 0;
}

uint16_t readPH_ADC()
{
  return analogRead(A7);
}

void printPROGMEM(const char *str)
{
  char c;
  while ((c = pgm_read_byte(str++)))
  {
    Serial.write(c);
  }
}

void printPROGMEMln(const char *str)
{
  printPROGMEM(str);
  Serial.println();
}

void printDateTime(DateTime dt)
{
  Serial.print(dt.year());
  Serial.write('/');
  Serial.print(dt.month());
  Serial.write('/');
  Serial.print(dt.day());
  Serial.write(' ');
  Serial.print(dt.hour());
  Serial.write(':');
  Serial.print(dt.minute());
  Serial.write(':');
  Serial.println(dt.second());
}

void checkScheduledFeeding()
{
  DateTime now = rtc.now();
  uint8_t currentDay = now.day();
  uint8_t currentHour = now.hour();
  uint8_t currentMinute = now.minute();

  if (currentDay != lastFeedDay)
  {
    dailyFeedingCount = 0;
    for (uint8_t i = 0; i < NUM_SCHEDULED_FEEDINGS; i++)
    {
      scheduledFeedingDone[i] = false;
    }
    lastFeedDay = currentDay;
    Serial.print(F("New day detected: "));
    Serial.print(currentDay);
    Serial.print(F(" - Feeding counters reset, today's target: "));
    Serial.print(NUM_SCHEDULED_FEEDINGS);
    Serial.println(F(" scheduled feedings"));
  }

  for (uint8_t i = 0; i < NUM_SCHEDULED_FEEDINGS; i++)
  {
    if (currentHour == FEEDING_SCHEDULE[i].hour &&
        currentMinute == FEEDING_SCHEDULE[i].minute &&
        !scheduledFeedingDone[i])
    {
      if (!CHECK_FLAG(FLAG_FEEDING))
      {
        Serial.print(F("Scheduled feeding started at "));
        if (FEEDING_SCHEDULE[i].hour < 10)
          Serial.write('0');
        Serial.print(FEEDING_SCHEDULE[i].hour);
        Serial.write(':');
        if (FEEDING_SCHEDULE[i].minute < 10)
          Serial.write('0');
        Serial.println(FEEDING_SCHEDULE[i].minute);
        printPROGMEMln(STR_FEEDING_SCHEDULED);
        currentFeedingType = 2;
        startFeeding(2);
        Serial.print(F("Feeding slot "));
        Serial.print(i + 1);
        Serial.print(F(" of "));
        Serial.print(NUM_SCHEDULED_FEEDINGS);
        Serial.println(F(" executed"));
      }
      else
      {
        printPROGMEMln(STR_FEEDING_SKIPPED);
      }
      break;
    }
  }
}

ISR(PCINT2_vect)
{
  uint8_t currentPortDState = PIND;
  uint8_t changedPins = currentPortDState ^ lastPortDState;

  if (changedPins & _BV(BUTTON_BIT))
  {
    uint32_t currentTime = millis();
    uint8_t currentButtonState = currentPortDState & _BV(BUTTON_BIT);
    uint8_t lastButtonState = lastPortDState & _BV(BUTTON_BIT);

    if (currentButtonState && !lastButtonState)
    {
      if (currentTime - lastInterruptTime > DEBOUNCE_DELAY_ISR)
      {
        buttonPressedFlag = 1;
        lastInterruptTime = currentTime;
      }
    }
  }

  lastPortDState = currentPortDState;
}

void setup()
{
  Serial.begin(9600);

  analogReference(EXTERNAL);

  delay(10);

  analogRead(A7);
  delay(10);

  Serial.println(F("=== pH Calibration ==="));
  Serial.print(F("PH_SLOPE: "));
  Serial.println(pgm_read_float(&PH_SLOPE), 6);
  Serial.print(F("PH_OFFSET: "));
  Serial.println(pgm_read_float(&PH_OFFSET), 6);
  Serial.println(F("Using External AREF (LM336BZ-5.0V)"));
  Serial.println(F("===================="));

  FAN_DDR |= _BV(FAN_BIT);
  ALERT_LED_DDR |= _BV(ALERT_LED_BIT);
  BUTTON_DDR &= ~_BV(BUTTON_BIT);
  BUTTON_PORT |= _BV(BUTTON_BIT);

  setFanOff();
  setAlertLedOff();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  sensors.begin();
  sensors.setResolution(12);

  if (!rtc.begin())
  {
    printPROGMEMln(STR_RTC_ERROR);
    lcd.setCursor(0, 0);
    lcd.print(F("RTC Error!"));
    while (1)
      ;
  }

  if (rtc.lostPower())
  {
    printPROGMEMln(STR_RTC_LOST_POWER);
  }

  printPROGMEMln(STR_RTC_DIAG);
  rtcDiagStartDateTime = rtc.now();
  rtcDiagStartTime = millis();
  Serial.print(F("Current time: "));
  printDateTime(rtcDiagStartDateTime);

  feederServo.attach(SERVO_PIN);
  feederServo.write(VALVE_CLOSED_POSITION);

  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(BUTTON_BIT);
  lastPortDState = PIND;
  sei();

  uint32_t currentTime = millis();
  lastPhSampleTime = currentTime;
  lastTempRequestTime = currentTime;
  lastDisplayUpdate = currentTime;
  lastSerialOutput = currentTime;
  currentAlertCycleStart = currentTime;

  DateTime now = rtc.now();
  lastFeedDay = now.day();
  currentFeedingType = 0;
  dailyFeedingCount = 0;

  for (uint8_t i = 0; i < NUM_SCHEDULED_FEEDINGS; i++)
  {
    bool isPastTime = (now.hour() > FEEDING_SCHEDULE[i].hour) ||
                      (now.hour() == FEEDING_SCHEDULE[i].hour && now.minute() > FEEDING_SCHEDULE[i].minute);
    scheduledFeedingDone[i] = isPastTime;
    if (scheduledFeedingDone[i])
    {
      dailyFeedingCount++;
    }
  }

  Serial.print(F("Feeding system initialized - "));
  Serial.print(NUM_SCHEDULED_FEEDINGS);
  Serial.print(F(" scheduled times: "));
  for (uint8_t i = 0; i < NUM_SCHEDULED_FEEDINGS; i++)
  {
    if (FEEDING_SCHEDULE[i].hour < 10)
      Serial.write('0');
    Serial.print(FEEDING_SCHEDULE[i].hour);
    Serial.write(':');
    if (FEEDING_SCHEDULE[i].minute < 10)
      Serial.write('0');
    Serial.print(FEEDING_SCHEDULE[i].minute);
    if (i < NUM_SCHEDULED_FEEDINGS - 1)
      Serial.print(F(", "));
  }
  Serial.println();
  Serial.print(F("Daily count: "));
  Serial.print(dailyFeedingCount);
  Serial.println(F(" feedings completed today"));

  printPROGMEMln(STR_SYSTEM_INIT);
}

void handleNonBlockingRTCDiagnostics()
{
  if (!CHECK_FLAG(FLAG_RTC_DIAG_DONE) && (millis() - rtcDiagStartTime >= 3000))
  {
    DateTime now2 = rtc.now();
    int16_t timeDiff = now2.second() - rtcDiagStartDateTime.second();
    if (timeDiff < 0)
      timeDiff += 60;

    if (timeDiff >= 2 && timeDiff <= 4)
    {
      printPROGMEMln(STR_RTC_TICKING);
    }
    else
    {
      printPROGMEMln(STR_RTC_ISSUE);
    }
    SET_FLAG(FLAG_RTC_DIAG_DONE);
  }
}

void setupLCDLabels()
{
  if (!CHECK_FLAG(FLAG_LCD_INIT))
  {
    lcd.setCursor(0, 0);
    lcd.print(F("pH:"));
    lcd.setCursor(0, 1);
    lcd.print(F("TEMP:"));
    SET_FLAG(FLAG_LCD_INIT);
  }
}

void updatePhReading()
{
  uint32_t currentMillis = millis();

  if (currentMillis - lastPhSampleTime >= phSampleInterval)
  {
    if (phSampleCount < SAMPLES)
    {
      uint16_t adcReading = readPH_ADC();
      phSampleSum += adcReading;
      phSampleCount++;
      lastPhSampleTime = currentMillis;
    }
    else
    {
      float avgADC = (float)phSampleSum / SAMPLES;

      float voltage = avgADC * (5.0 / 1024.0);
      float ph = (voltage * pgm_read_float(&PH_SLOPE)) + pgm_read_float(&PH_OFFSET);

      if (ph < 0.0)
        ph = 0.0;
      if (ph > 14.0)
        ph = 14.0;

      currentPH_scaled = (uint16_t)(ph * 100.0);

      if (phSampleCount == SAMPLES)
      {
        Serial.print(F("pH: ADC="));
        Serial.print(avgADC, 1);
        Serial.print(F(", V="));
        Serial.print(voltage, 3);
        Serial.print(F(", pH="));
        Serial.println((float)currentPH_scaled / 100.0, 2);
      }

      phSampleSum = 0;
      phSampleCount = 0;
    }
  }
}

void updateTemperatureReading()
{
  uint32_t currentMillis = millis();

  if (!CHECK_FLAG(FLAG_TEMP_REQUESTED) && (currentMillis - lastTempRequestTime >= tempRequestInterval))
  {
    sensors.requestTemperatures();
    SET_FLAG(FLAG_TEMP_REQUESTED);
    lastTempRequestTime = currentMillis;
  }

  if (CHECK_FLAG(FLAG_TEMP_REQUESTED) && (currentMillis - lastTempRequestTime >= 100))
  {
    float tempC = sensors.getTempCByIndex(0);
    if (tempC != DEVICE_DISCONNECTED_C)
    {
      currentTemp_scaled = (uint16_t)(tempC * 10.0);
    }
    CLEAR_FLAG(FLAG_TEMP_REQUESTED);
  }
}

void handleFeeding()
{
  uint32_t currentMillis = millis();

  if (CHECK_FLAG(FLAG_FEEDING))
  {
    if (!CHECK_FLAG(FLAG_VALVE_OPEN))
    {
      feederServo.write(VALVE_OPEN_POSITION);
      SET_FLAG(FLAG_VALVE_OPEN);
      feedingStartTime = currentMillis;
      printPROGMEMln(STR_VALVE_OPEN);
    }
    else if (currentMillis - feedingStartTime >= FEEDING_DURATION)
    {
      feederServo.write(VALVE_CLOSED_POSITION);
      CLEAR_FLAG(FLAG_VALVE_OPEN);
      CLEAR_FLAG(FLAG_FEEDING);
      printPROGMEMln(STR_VALVE_CLOSE);

      dailyFeedingCount++;

      if (currentFeedingType == 1)
      {
        Serial.print(F("Manual feeding completed - daily count: "));
        Serial.println(dailyFeedingCount);
      }
      else if (currentFeedingType == 2)
      {
        DateTime now = rtc.now();
        uint8_t currentHour = now.hour();
        uint8_t currentMinute = now.minute();

        for (uint8_t i = 0; i < NUM_SCHEDULED_FEEDINGS; i++)
        {
          if (currentHour == FEEDING_SCHEDULE[i].hour &&
              currentMinute == FEEDING_SCHEDULE[i].minute)
          {
            scheduledFeedingDone[i] = true;
            Serial.print(F("Scheduled feeding "));
            Serial.print(i + 1);
            Serial.print(F(" ("));
            if (FEEDING_SCHEDULE[i].hour < 10)
              Serial.write('0');
            Serial.print(FEEDING_SCHEDULE[i].hour);
            Serial.write(':');
            if (FEEDING_SCHEDULE[i].minute < 10)
              Serial.write('0');
            Serial.print(FEEDING_SCHEDULE[i].minute);
            Serial.print(F(") completed - daily count: "));
            Serial.println(dailyFeedingCount);
            break;
          }
        }
      }

      printPROGMEMln(STR_FEEDING_COMPLETED);

      currentFeedingType = 0;
    }
  }
}

void startFeeding(uint8_t feedingType)
{
  if (CHECK_FLAG(FLAG_FEEDING))
  {
    if (feedingType == 1)
    {
      Serial.println(F("Feeding already in progress - ignoring button"));
    }
    else if (feedingType == 2)
    {
      Serial.println(F("Feeding already in progress - ignoring schedule"));
    }
    return;
  }

  SET_FLAG(FLAG_FEEDING);
  CLEAR_FLAG(FLAG_VALVE_OPEN);

  if (feedingType == 1)
  {
    currentFeedingType = 1;
    printPROGMEMln(STR_FEEDING_MANUAL);
  }
  else if (feedingType == 2)
  {
    printPROGMEMln(STR_FEEDING_SCHEDULED);
  }
}

void updateFanControl()
{
  if (currentTemp_scaled >= 280 && !CHECK_FLAG(FLAG_FAN_RUNNING))
  {
    setFanOn();
    SET_FLAG(FLAG_FAN_RUNNING);
    printPROGMEMln(STR_FAN_ON);
  }
  else if (currentTemp_scaled <= 260 && CHECK_FLAG(FLAG_FAN_RUNNING))
  {
    setFanOff();
    CLEAR_FLAG(FLAG_FAN_RUNNING);
    printPROGMEMln(STR_FAN_OFF);
  }
}

void updateAlertLED()
{
  uint32_t currentMillis = millis();

  uint8_t newAlertBlinks = 0;
  uint8_t phIssue = (currentPH_scaled < 600 || currentPH_scaled > 800) ? 1 : 0;
  uint8_t tempIssue = (currentTemp_scaled > 300) ? 1 : 0;

  if (phIssue && tempIssue)
  {
    newAlertBlinks = 3;
  }
  else if (tempIssue)
  {
    newAlertBlinks = 2;
  }
  else if (phIssue)
  {
    newAlertBlinks = 1;
  }

  if (newAlertBlinks != alertBlinks)
  {
    alertBlinks = newAlertBlinks;
    setAlertLedOff();
    alertLedIsOn = 0;
    currentBlinkCount = 0;
    lastBlinkToggleMillis = currentMillis;
    currentAlertCycleStart = currentMillis;
  }

  if (alertBlinks == 0)
  {
    setAlertLedOff();
    return;
  }

  if (alertLedIsOn)
  {
    if (currentMillis - lastBlinkToggleMillis >= 100)
    {
      setAlertLedOff();
      alertLedIsOn = 0;
      lastBlinkToggleMillis = currentMillis;
    }
  }
  else
  {
    if (currentBlinkCount < alertBlinks)
    {
      if (currentMillis - lastBlinkToggleMillis >= 100)
      {
        setAlertLedOn();
        alertLedIsOn = 1;
        currentBlinkCount++;
        lastBlinkToggleMillis = currentMillis;
      }
    }
    else
    {
      if (currentMillis - currentAlertCycleStart >= 1000)
      {
        currentBlinkCount = 0;
        currentAlertCycleStart = currentMillis;
      }
    }
  }
}

void updateDisplay()
{
  uint32_t currentMillis = millis();

  if (currentMillis - lastDisplayUpdate >= displayUpdateInterval)
  {
    setupLCDLabels();

    DateTime now = rtc.now();

    lcd.setCursor(4, 0);
    lcd.print(F("      "));
    lcd.setCursor(4, 0);
    lcd.print((float)currentPH_scaled / 100.0, 2);

    lcd.setCursor(11, 0);
    if (now.hour() < 10)
      lcd.write('0');
    lcd.print(now.hour());
    lcd.write(':');
    if (now.minute() < 10)
      lcd.write('0');
    lcd.print(now.minute());

    lcd.setCursor(6, 1);
    lcd.print(F("     "));
    lcd.setCursor(6, 1);
    lcd.print((float)currentTemp_scaled / 10.0, 1);
    lcd.write('C');

    lcd.setCursor(12, 1);
    lcd.print(F("    "));
    lcd.setCursor(12, 1);

    if (dailyFeedingCount > 0)
    {
      lcd.print(F("F:"));
      lcd.print(dailyFeedingCount);
    }

    lastDisplayUpdate = currentMillis;
  }
}

void updateSerialOutput()
{
  uint32_t currentMillis = millis();

  if (currentMillis - lastSerialOutput >= serialOutputInterval)
  {
    DateTime now = rtc.now();

    Serial.print(F("Time: "));
    if (now.hour() < 10)
      Serial.write('0');
    Serial.print(now.hour());
    Serial.write(':');
    if (now.minute() < 10)
      Serial.write('0');
    Serial.print(now.minute());
    Serial.write(':');
    if (now.second() < 10)
      Serial.write('0');
    Serial.print(now.second());

    Serial.print(F(" | pH: "));
    Serial.print((float)currentPH_scaled / 100.0, 2);
    Serial.print(F(" | Temp: "));
    Serial.print((float)currentTemp_scaled / 10.0, 1);
    Serial.print(F("C | Fan: "));
    Serial.print(CHECK_FLAG(FLAG_FAN_RUNNING) ? F("ON") : F("OFF"));
    Serial.print(F(" | Alert LED: "));
    Serial.print(alertBlinks);
    Serial.print(F(" blinks | Feeding: "));
    Serial.print(CHECK_FLAG(FLAG_FEEDING) ? F("ACTIVE") : F("READY"));
    Serial.print(F(" | Daily Count: "));
    Serial.print(dailyFeedingCount);
    Serial.print(F("/"));
    Serial.print(NUM_SCHEDULED_FEEDINGS);
    Serial.print(F(" | LCD: "));
    if (dailyFeedingCount > 0)
    {
      Serial.print(F("F:"));
      Serial.print(dailyFeedingCount);
    }
    else
    {
      Serial.print(F("Empty"));
    }
    Serial.println();

    lastSerialOutput = currentMillis;
  }
}

void loop()
{
  handleNonBlockingRTCDiagnostics();

  if (buttonPressedFlag)
  {
    buttonPressedFlag = 0;
    startFeeding(1);
  }

  updatePhReading();
  updateTemperatureReading();

  checkScheduledFeeding();

  handleFeeding();
  updateFanControl();
  updateAlertLED();
  updateDisplay();
  updateSerialOutput();
}