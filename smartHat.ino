#include <Wire.h>
#define USE_ARDUINO_INTERRUPTS true

#include <PulseSensorPlayground.h>  ////  BPM   ///////
PulseSensorPlayground pulseSensor;
int PulseSensorPin = 0;
int Threshold = 550;
int BPM;
bool heart = false;

//////////////////////////////////////  TEMP   ///////
int ThermistorPin = 2;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
int temp;
bool cold = false;
bool hot = false;

#include <Ultrasonic.h>  ///////////  ULTRASONIC   ////
Ultrasonic top(4, 5);
Ultrasonic rear(2, 3);
int topDistance;   // TOP
int rearDistance;  // REAR
int distanceLimit = 30;

#include <U8glib.h>  ////////////////  SCREEN   ///////
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

#include <MQ2.h>  ////////////////////  GAS   /////////
MQ2 mq2(A1);
float smoke;
float co;
float lpg;
int gas = 0;

#define dustPin A3          // Black wire////  DUST  //////////
#define dustLedPin 11       // White wire
#define ANALOG_VOLTAGE 3.3  // analog top of range
float dust_density;
float output_voltage;
bool dust = false;

// PINS
int ledPin = 8;
int buzzerPin = 7;
int warningPin = 6;
int vibTopPin = 10;
int vibRearPin = 9;

// GENERAL
unsigned long starttime;
unsigned long start = 0;
int timer = 1001;
bool state = false;


// Object detection function
void objDetect() {

  // Measure Distance
  topDistance = top.read();
  //  Serial.print("Top: " + String(topDistance) + "\t");

  rearDistance = rear.read();
  //  Serial.println("Rear: " + String(rearDistance) + "\t");

  // Warning
  if (topDistance <= distanceLimit || topDistance == 0 || rearDistance <= distanceLimit || rearDistance == 0) {
    tone(buzzerPin, 500);

    if (topDistance <= distanceLimit || topDistance == 0) {
      digitalWrite(vibTopPin, HIGH);
    } else {
      digitalWrite(vibTopPin, LOW);
    }

    if (rearDistance <= distanceLimit || rearDistance == 0) {
      digitalWrite(vibRearPin, HIGH);
    } else {
      digitalWrite(vibRearPin, LOW);
    }
  }

  else {
    noTone(buzzerPin);
    digitalWrite(vibRearPin, LOW);
    digitalWrite(vibTopPin, LOW);
  }
}


// temperature function
void tempHumid() {
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  temp = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  temp = temp - 273.15;
  temp = (((temp * 9.0) / 5.0 + 32.0) - 32) * 5 / 9;
  //    Serial.print("Temperature: "+String(temp));

  // Hot or cold?
  if (temp > 30) {
    hot = true;
  } else {
    hot = false;
    if (temp < 13) {
      cold = true;
    } else {
      cold = false;
    }
  }
}


// BPM function
void heartRate() {
  BPM = pulseSensor.getBeatsPerMinute();
  //  Serial.println("BPM: " + String(BPM));

  // Too high?
  if (BPM > 170) {
    heart++;
  } else {
    heart = 0;
  }
}


// Gas function
void gasDnsity() {
  float* values = mq2.read(true);
  co = mq2.MQGetGasPercentage(mq2.MQRead() / mq2.Ro, mq2.GAS_CO);
  lpg = mq2.MQGetGasPercentage(mq2.MQRead() / mq2.Ro, mq2.GAS_LPG);
  smoke = mq2.MQGetGasPercentage(mq2.MQRead() / mq2.Ro, mq2.GAS_SMOKE);

  // Too much gas?
  if (smoke > 5000 || co > 60 || lpg > 1000) {
    gas++;
  } else {
    gas = 0;
  }
}

// Dust function
void dustConcn() {
  digitalWrite(dustLedPin, LOW);
  delayMicroseconds(280);
  output_voltage = analogRead(dustPin);
  delay(1);
  digitalWrite(dustLedPin, HIGH);  // turn the LED off

  output_voltage = (output_voltage / 1023) * ANALOG_VOLTAGE;
  dust_density = (0.18 * output_voltage) - 0.1;

  // Too dusty?
  if (dust_density < 0) {
    dust_density = 0.00;
  }
  if (dust_density > 0.15) {
    dust = true;
  } else {
    dust = false;
  }
}


// Home screen
void home() {
  u8g.drawLine(37, 5, 37, 27);
  u8g.drawLine(79, 5, 79, 27);
  u8g.drawLine(0, 32, 128, 32);

  u8g.setFont(u8g_font_helvR10);
  u8g.setPrintPos(0, 20);
  u8g.print(String(temp) + " C");

  u8g.setPrintPos(46, 12);
  u8g.print(BPM);
  u8g.setPrintPos(44, 27);
  u8g.print("BPM");

  u8g.setPrintPos(85, 12);
  u8g.print(String(dust_density));
  u8g.setPrintPos(85, 27);
  u8g.print("mg/m3");

  u8g.setPrintPos(2, 49);
  u8g.print("S: " + String(smoke));
  u8g.setPrintPos(62, 49);
  u8g.print("CO: " + String(co));
  u8g.setPrintPos(30, 64);
  u8g.print("LPG: " + String(lpg));
}


void setup() {
  Serial.begin(9600);

  mq2.begin();
  pulseSensor.begin();
  pulseSensor.analogInput(PulseSensorPin);
  pulseSensor.setThreshold(Threshold);
  pinMode(12, INPUT);
  pinMode(vibTopPin, OUTPUT);
  pinMode(vibRearPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(dustLedPin, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}


void loop() {

  // Check sensor values every 30 seconds
  if (timer > 1000) {
    objDetect();
    tempHumid();
    objDetect();
    gasDnsity();
    objDetect();
    heartRate();
    objDetect();
    dustConcn();
    objDetect();

    timer = 0;
    u8g.firstPage();
    do {
      home();
    } while (u8g.nextPage());

    starttime = millis();
    while (millis() - starttime < 2000) {
      objDetect();
    }

    // Gas warning //////////////////////////////////
    if (gas) {

      digitalWrite(ledPin, HIGH);
      tone(buzzerPin, 500);

      u8g.firstPage();
      do {
        u8g.setPrintPos(20, 12);
        u8g.print("Warning: GAS!");
        u8g.setPrintPos(0, 29);
        u8g.print("Smoke: " + String(smoke));
        u8g.setPrintPos(0, 45);
        u8g.print("LPG: " + String(lpg));
        u8g.setPrintPos(0, 61);
        u8g.print("CO2: " + String(co));

      } while (u8g.nextPage());
      noTone(buzzerPin);
      starttime = millis();
      while (millis() - starttime < 5000) {
        objDetect();
      }

      digitalWrite(ledPin, LOW);
    }

    // Heat warning //////////////////////////////////
    if (hot) {
      digitalWrite(ledPin, HIGH);
      tone(buzzerPin, 500, 500);

      u8g.firstPage();
      do {
        u8g.setFont(u8g_font_helvR10);
        u8g.setPrintPos(0, 12);
        u8g.print("Warning: TOO HOT!");
        u8g.setPrintPos(45, 40);
        u8g.print(String(temp) + " C");
      } while (u8g.nextPage());

      noTone(buzzerPin);
      starttime = millis();
      while (millis() - starttime < 5000) {
        objDetect();
      }
      digitalWrite(ledPin, LOW);
    }

    // Cold warning //////////////////////////////////
    else if (cold) {
      digitalWrite(ledPin, HIGH);

      tone(buzzerPin, 500);

      u8g.firstPage();
      do {
        u8g.setFont(u8g_font_helvR10);
        u8g.setPrintPos(30, 12);
        u8g.print("Warning:");
        u8g.setPrintPos(16, 31);
        u8g.print("TOO COLD!");
        u8g.setPrintPos(45, 55);
        u8g.print(String(temp) + " C");
      } while (u8g.nextPage());
      noTone(buzzerPin);
      starttime = millis();
      while (millis() - starttime < 5000) {
        objDetect();
      }
      digitalWrite(ledPin, LOW);
    }

    // BPM warning //////////////////////////////////
    if (heart) {
      digitalWrite(ledPin, HIGH);
      tone(buzzerPin, 500);

      u8g.firstPage();
      do {
        u8g.setPrintPos(30, 12);
        u8g.print("Warning:");
        u8g.setPrintPos(2, 31);
        u8g.print("HIGH HEART RATE!");
        u8g.setPrintPos(25, 52);
        u8g.print(String(BPM));
        u8g.setPrintPos(56, 52);
        u8g.print("BPM");
      } while (u8g.nextPage());

      noTone(buzzerPin);
      starttime = millis();
      while (millis() - starttime < 5000) {
        objDetect();
      }
      digitalWrite(ledPin, LOW);
    }

    // Dust warning //////////////////////////////////
    if (dust) {
      digitalWrite(ledPin, HIGH);
      tone(buzzerPin, 500);

      u8g.firstPage();
      do {
        u8g.setPrintPos(30, 12);
        u8g.print("Warning:");
        u8g.setPrintPos(2, 31);
        u8g.print("TOO MUCH DUST!");
        u8g.setPrintPos(25, 52);
        u8g.print(String(dust_density));
        u8g.setPrintPos(56, 52);
        u8g.print("mg/m3");
      } while (u8g.nextPage());

      noTone(buzzerPin);
      starttime = millis();
      while (millis() - starttime < 5000) {
        objDetect();
      }
      digitalWrite(ledPin, LOW);
    }

    u8g.firstPage();
    do {
      home();
    } while (u8g.nextPage());
  }

  // Run during 30 second break
  objDetect();

  // Check button
  int buttonState = digitalRead(12);
  if (buttonState == HIGH && millis() - start > 700) {
    state = !state;
    //      Serial.println("Button State: " + String(state));
    start = millis();
  }
  if (state) {
    digitalWrite(13, HIGH);
    tone(warningPin, 200);
    //        Serial.print("Button State: on");
  } else {
    digitalWrite(13, LOW);
    noTone(warningPin);
    //        Serial.print("Button State: off");
  }
  timer++;
}
