#include <MPU6050.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "RF24.h"
#include "printf.h"

RF24 radio(9, 10); //CE, CSN pins
 
byte address[6] = "1Node"; //Any 5 character string for address

//Function to configure the radio
void configureRadio() {
  radio.begin(); //starts radio
  radio.setPALevel(RF24_PA_HIGH); //Power level of radio (Can also be RF24_PA_MED, RF24_PA_HIGH, RF24_PA_MAX)
  radio.openWritingPipe(address); //Transmits on this address
  radio.stopListening(); //Make sure radio is in tx not rx
  radio.setAutoAck(0,false);
}


bool flex_output = false;
bool imu_output = true;
bool rst_button_output = true;

MPU6050 mpu;

const int FLEX_PIN = A4; // Pin connected to voltage divider output
const int RST_PIN = 4;

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 47000.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
const float STRAIGHT_RESISTANCE = 15000.0; // resistance when straight
const float BEND_RESISTANCE = 17000.0; // resistance at 90 deg

// Timers
const float FLEX_INTERVAL = 50;
float flex_timer = millis();

unsigned long IMU_INTERVAL = 20;
float imu_timer = millis();

const float RST_BUTTON_INTERVAL = 50;
float rst_button_timer = millis();

unsigned long MOTOR_INTERVAL = 50;
float motor_timer = millis();

const int RADIO_INTERVAL = 300;
float radio_timer = millis();

// Sensor values
float pitch = 0;
float roll = 0;
float yaw = 0;

int bend_angle = 0;

int base_thrust = 0;

void setup() 
{
  Serial.begin(115200);
  pinMode(FLEX_PIN, INPUT);
  pinMode(RST_PIN, INPUT);
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
//  mpu.calibrateGyro(/3);
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
//  mpu.setThreshold(1);/
  printf_begin();
  configureRadio();
}

void read_flex() {
  int flexADC = analogRead(FLEX_PIN);
  Serial.println(flexADC);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  bend_angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
                   
  if (flex_output) {
    Serial.println("Resistance: " + String(flexR) + " ohms");
    Serial.println("Bend: " + String(bend_angle) + " degrees");
    Serial.println();
  }
  if (bend_angle < 330) {
    base_thrust = 0;
  } else {
    base_thrust = (bend_angle/330) * 1.1;
  }
}

void read_IMU() {
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
  pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  if (-10 < pitch && pitch < 10) {
    pitch = 0;
  } else {
    if (pitch > 10) {
      pitch -= 10;
    } else {
      pitch += 10;
    }
  }
  if (-10 < roll && roll < 10) {
    roll = 0;
  } else {
    if (roll > 10) {
      roll -= 10;
    } else {
      roll += 10;
    }
  }
  // Output
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);
  
  Serial.println();
  
  imu_timer = millis();
}

void handle_rst() {
  if (digitalRead(RST_PIN) == 0) {
    if (rst_button_output) {
      Serial.println("RST");
    }
    pitch = 0;
    roll = 0;
    yaw = 0;
  }
  rst_button_timer = millis();
}

void sendData() {
  String str = String(pitch) + " " + String(roll) + " " + String(base_thrust) + " ";
  int str_len = str.length() + 1; 
  char text[str_len];
  Serial.println(text);
  str.toCharArray(text, str_len);
  Serial.println(text);
  if (!radio.write(&text, sizeof(text))) { //Returns a bool depending on whether it errors or not
    Serial.println(F("Failed, radio.write returned false"));
  }
}

void loop() 
{

  if (millis() - imu_timer > IMU_INTERVAL) {
    read_IMU();
  }

  if (millis() - rst_button_timer > RST_BUTTON_INTERVAL) {
    handle_rst();
  }

  if (millis() - flex_timer > FLEX_INTERVAL) {
    read_flex();
    flex_timer = millis();
  }

  if (millis() - radio_timer > RADIO_INTERVAL) {
    sendData();
    radio_timer = millis();
  }
  
}
