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
bool motor_output = true;

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
float offset_pitch = 0;
float offset_roll = 0;

float raw_pitch = 0;
float raw_roll = 0;

float pitch = 0;
float roll = 0;
float yaw = 0;

int bend_angle = 0;
int bend_angle_offset = 0;

bool is_calibrated = false;

int wheel_speed = 0;
int wheel_differential = 0;
int left_wheel_speed = 0;
int right_wheel_speed = 0;
int temp_left_wheel_speed = 0;
int temp_right_wheel_speed = 0;

int deadzone = 5;

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
  mpu.calibrateGyro();
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(1);
  printf_begin();
  configureRadio();
}

void read_flex() {
  int flexADC = analogRead(FLEX_PIN);
//  Serial.println(flexADC);
  float flexV = flexADC * VCC / 1023.0;
  float flexR = R_DIV * (VCC / flexV - 1.0);
  bend_angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,
                   0, 90.0);
                   
  if (flex_output) {
    Serial.println("Resistance: " + String(flexR) + " ohms");
    Serial.println("Bend: " + String(bend_angle) + " degrees");
    Serial.println();
  }
  
  // Map bend_angle to wheel_speed
  bend_angle = bend_angle - bend_angle_offset;
  if (bend_angle < 10) {
    wheel_speed = 0;
  } else {
    wheel_speed = map(bend_angle, 10, 60, 0, 130); 
  }

}

void read_IMU() {
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
  raw_pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  raw_roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  pitch = raw_pitch - offset_pitch;
  roll = raw_roll - offset_roll;
  if (-deadzone < pitch && pitch < deadzone) {
    pitch = 0;
  } else {
    if (pitch > deadzone) {
      pitch -= deadzone;
    } else {
      pitch += deadzone;
    }
  }
  if (-deadzone < roll && roll < deadzone) {
    roll = 0;
  } else {
    if (roll > deadzone) {
      roll -= deadzone;
    } else {
      roll += deadzone;
    }
  }

  wheel_differential = 0.5 * roll;

  imu_timer = millis();
}

void handle_rst() {
  if (digitalRead(RST_PIN) == 0) {
    if (rst_button_output) {
      Serial.println("RST");
    }
    offset_pitch = raw_pitch;
    offset_roll = raw_roll;
    bend_angle_offset = bend_angle;
    is_calibrated = true;
  }
  rst_button_timer = millis();
}

void calculate_motor_speeds() {
  if (wheel_speed != 0) {
    temp_left_wheel_speed = wheel_speed - wheel_differential;
    temp_right_wheel_speed = wheel_speed + wheel_differential;
  } else {
    temp_left_wheel_speed = 0;
    temp_right_wheel_speed = 0;
  }
  
  if (temp_left_wheel_speed < 0) {
    temp_left_wheel_speed = 0;
  }
  if (temp_right_wheel_speed < 0) {
    temp_right_wheel_speed = 0;
  }
  if (temp_left_wheel_speed > 125) {
    temp_left_wheel_speed = 125;
  }
  if (temp_right_wheel_speed > 125) {
    temp_right_wheel_speed = 125;
  }

  if (motor_output) {
   Serial.println("Left: ");
   Serial.print(left_wheel_speed);
   Serial.print("  Right: ");
   Serial.print(right_wheel_speed);
   Serial.println();
  }
  

  left_wheel_speed = temp_left_wheel_speed;
  right_wheel_speed = temp_right_wheel_speed;

}

void sendData() {
  if (!is_calibrated) {
    return;
  }
  String str = String(right_wheel_speed) + " " + String(left_wheel_speed) + " "; 
  int str_len = str.length() + 1; 
  char text[str_len];
  str.toCharArray(text, str_len);
  Serial.println("I'm here");
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

  if (millis() - motor_timer > MOTOR_INTERVAL) {
    calculate_motor_speeds();
    motor_timer = millis();
  }

  if (millis() - radio_timer > RADIO_INTERVAL) {
    sendData();
    radio_timer = millis();
  }
  
}
