#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
int SDA_PIN = 23;
int SCL_PIN = 22;
const int timeDelay = 100;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("MPU6050 not found. Check wiring.");
    delay(1000);
  }
  Serial.println("MPU6050 ready!");
}

void loop() {
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  // Output
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);
  
  Serial.println();
  

  delay(timeDelay);
}

