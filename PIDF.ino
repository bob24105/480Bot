#include "Wire.h" // includes for MPU
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

// Motor Structure
struct motor {
    // Member variables
    int ID;
    int IN1;
    int IN2;
    int ENA;
    int ENC_A;
    int ENC_B;

};


#define SATURATE(x,lo,hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))


// Assign motor ID and pins
motor motor1{1, 18, 19, 4, 21, 34};
motor motor2{2, 26, 27, 25, 33, 32};


// Macros
  // This function returns the hz over a loop
#define PrintHzRateDebug()                                                                 \
    do                                                                                     \
    {                                                                                      \
        static unsigned long last_time = micros();                                         \
        static unsigned int loop_count = 0;                                                \
                                                                                           \
        loop_count++;                                                                      \
        if (loop_count >= 1000)                                                            \
        {                                                                                  \
            unsigned long current_time = micros();                                         \
            unsigned long elapsed      = current_time - last_time;                         \
            last_time                  = current_time;                                     \
            loop_count                 = 0;                                                \
                                                                                           \
            if (elapsed > 0)                                                               \
            {                                                                              \
                /*elapsed is microseconds for 1000 loops -> Hz = 1000 / (elapsed/1e6) */   \
                float hz = (1000.0f * 1e6f) / (float)elapsed;                              \
                Serial.println(hz);                                                        \
            }                                                                              \
        }                                                                                  \
    } while (0)

  // This fucntion allows a funcntion to be ran at a specific sampling time
#define DO_EVERY(seconds, block)                      \
    do                                                \
    {                                                 \
        static unsigned long _lastRunTime = 0;        \
        unsigned long _now                = millis(); \
        if (_now - _lastRunTime >= (seconds * 1e3))   \
        {                                             \
            _lastRunTime = _now;                      \
            block;                                    \
        }                                             \
    } while (0)


// MPU Variables
int SDA_PIN = 23;
int SCL_PIN = 22;


// Variables for setting motor speed
static double speed;

/* Trying to filter the speed
// static double filtSpeed;
// static double prevSpeed = 0;
// static double prevFiltSpeed = 0;
// static double lowC = 1;  //  Hz
*/

// Variables for calculating RPM
static double RPM;
static double countDif;
const double timeDelay = 1; // [ms]
const double clockSpeed = timeDelay / 1000; // [s]
const double N = 1328; // counts per revolution on motor 1

// PID Variables
static double Hz = 100.0; // [Hz]
static double Ts = 1.0/Hz; // [s] Got this from PrintHzRateDebug()
static double alpha = 0.75;  // Complementary filter cutoff
static double wc = Hz / 20.0;  // cutoff frequency for DF


// For single PID
static double prevAngleError = 0;
static double angleErrorDot;
static double prevAngleErrorDot = 0;
static double filtAngleErrorDot;
static double prevFiltAngleErrorDot = 0;
static double angleErrorInt;
static double prevAngleErrorInt = 0;



// Angle PID Vars
static double pitchAngle;
static double angleError;
static double angleSetPoint = 86.55;  // [deg]
static double angleKp = 55.0;  // Proportional for angle PID. (55 best)
static double angleKd = 0.3;  // Derivative for angle PID (~1% of P) (0.3 best)
static double angleKi = 6.3;  // Integral for angle PID (~20% of P) (6 best)

static double error;
static double prevError = 0;
static double errorDot;
static double prevErrorDot = 0;
static double filtErrorDot;
static double prevFiltErrorDot = 0;
static double errorInt;
static double prevErrorInt = 0;
static double thetaOut;


// Vel PID Vars
static double pitchVel;
static double velKp = 1.0;  // Proportional for velocity PID
static double velKd = 0.0;  // Derivative for velocity PID
static double velKi = 0.0;  // Integral for velocity PID

static double velError;
static double prevVelError = 0;
static double velErrorDot;
static double prevVelErrorDot = 0;
static double filtVelErrorDot;
static double velErrorInt;
static double prevVelErrorInt = 0;
static double prevVelFiltErrorDot = 0;


// Stuff for encoder func
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
unsigned long _lastIncReadTime2 = micros(); 
unsigned long _lastDecReadTime2 = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 10;
volatile double currCount1 = 0;
volatile double currCount2 = 0;



void setup() {

  // Set up pins for MPU Reading
  Wire.begin(SDA_PIN, SCL_PIN);
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  // mpu.calcOffsets(); // gyro and accelero offsets  (commented out when happy with setup)

  // Set encoder pins and attach interrupts
  pinMode(motor1.ENC_A, INPUT_PULLUP);
  pinMode(motor1.ENC_B, INPUT_PULLUP);
  pinMode(motor2.ENC_A, INPUT_PULLUP);
  pinMode(motor2.ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(motor1.ENC_A), read_encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor1.ENC_B), read_encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.ENC_A), read_encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.ENC_B), read_encoder2, CHANGE);

  //Initialize Motor Pins
  pinMode(motor1.ENA, OUTPUT);
  pinMode(motor1.IN1, OUTPUT);
  pinMode(motor1.IN2, OUTPUT);

  pinMode(motor2.ENA, OUTPUT);
  pinMode(motor2.IN1, OUTPUT);
  pinMode(motor2.IN2, OUTPUT);

  // Start the serial monitor to show output
  Serial.begin(115200);
  
}


// Main loop that runs the code to control motors thorugh PIDs
void loop() {
  DO_EVERY(Ts, mainLoop()); // Runs main loop at 1000 Hz
  // PrintHzRateDebug();
}


// Main loop function to perform PID control
void mainLoop(){

  mpu.update();
  pitchAngle = mpu.getAngleX();
  pitchVel = mpu.getGyroX();

  angleError = pitchAngle - angleSetPoint;

  // For single PID
  angleErrorDot = (angleError - prevAngleError) / Ts;
  filtAngleErrorDot = (prevFiltAngleErrorDot + wc * angleErrorDot + wc * prevAngleErrorDot) * Ts; // filtering the derivative
  angleErrorInt = angleError * Ts + prevAngleErrorInt;
  speed = angleError * angleKp + filtAngleErrorDot * angleKd + angleErrorInt * angleKi;

  /*

  // Complementary filter to improve angle stability (combined error)
  error = alpha * (error + pitchVel * Ts) + (1 - alpha) * angleError;


  // Through Angle PID
  errorDot = (error - prevError) / Ts;
  filtErrorDot = (prevFiltErrorDot + wc * errorDot + wc * prevErrorDot) * Ts; // filtering the derivative
  errorInt = error * Ts + prevErrorInt;
  thetaOut = error * angleKp + filtErrorDot * angleKd + errorInt * angleKi;


  // Into Velocity PID
  velError = thetaOut - (error - prevError)/Ts;
  velErrorDot = (velError - prevVelError) / Ts;
  filtVelErrorDot = (prevVelFiltErrorDot + wc * velErrorDot + wc * prevVelErrorDot) * Ts;  // filtering the derivative
  velErrorInt = velError * Ts + prevVelErrorInt;
  speed = velError * velKp + filtVelErrorDot * velKd + velErrorInt * velKi;


  // filtSpeed = (lowC*Ts*speed + lowC*Ts*prevSpeed + (2-lowC*Ts)*prevFiltSpeed) / (2 + lowC*Ts);  // filtering speed output for low frequencies
  // filtSpeed = SATURATE(filtSpeed, -255, 255);

  */
  speed = SATURATE(speed, -255, 255);


  // Serial.println(pitchAngle);


  motorSpeed(motor1, speed);
  motorSpeed(motor2, speed);

  /*
  prevError = error;
  prevFiltErrorDot = filtErrorDot;
  prevErrorInt = errorInt;

  prevVelError = velError;
  prevVelFiltErrorDot = filtVelErrorDot;
  prevVelErrorInt = velErrorInt;

  */

  prevAngleError = angleError;
  prevFiltAngleErrorDot = filtAngleErrorDot;
  prevAngleErrorDot = angleErrorDot;
  prevAngleErrorInt = angleErrorInt;


  // prevSpeed = speed;
  // prevFiltSpeed = filtSpeed;
}


void motorSpeed(const motor& m, int speed) {

  // Pass speed -255 to 255
  if (m.ID == 1){
    if (speed > 0){
      digitalWrite(m.IN1, HIGH); // direction
      digitalWrite(m.IN2, LOW);
      analogWrite(m.ENA,  speed); // speed
    }
    else{
      digitalWrite(m.IN1, LOW);
      digitalWrite(m.IN2, HIGH);
      analogWrite(m.ENA,  - speed);
    }
  }
  else{
    if (speed > 0){
      digitalWrite(m.IN1, LOW);
      digitalWrite(m.IN2, HIGH);
      analogWrite(m.ENA,  speed);
    }
    else{
      digitalWrite(m.IN1, HIGH);
      digitalWrite(m.IN2, LOW);
      analogWrite(m.ENA,  - speed);
    }
  }
}


double vel(const motor& m, double currCount) {
  // Returns RPM
  static double prevCount1 = 0;
  static double prevCount2 = 0;

  double countDif;
  double RPM;

  if (m.ID == 1) {
    countDif = currCount - prevCount1;
    prevCount1 = currCount;
  } else if (m.ID == 2) {
    countDif = currCount - prevCount2;
    prevCount2 = currCount;
  } else {
    return 0; // Unknown motor ID
  }

  RPM = countDif / N / clockSpeed * 60;
  return RPM;
}




// Encoder ISR for motor1
void read_encoder1() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(motor1.ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(motor1.ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    currCount1 += changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    currCount1 += changevalue;              // Update counter
    encval = 0;
  }
} 


// Encoder ISR for motor2
void read_encoder2() {
  static uint8_t old_AB2 = 3;
  static int8_t encval2 = 0;
  static const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

  old_AB2 <<= 2;
  if (digitalRead(motor2.ENC_A)) old_AB2 |= 0x02;
  if (digitalRead(motor2.ENC_B)) old_AB2 |= 0x01;

  encval2 += enc_states[(old_AB2 & 0x0F)];

  if (encval2 > 3) {
    int change = 1;
    if ((micros() - _lastIncReadTime2) < _pauseLength) {
      change *= _fastIncrement;
    }
    _lastIncReadTime2 = micros();
    currCount2 += change;
    encval2 = 0;
  } else if (encval2 < -3) {
    int change = -1;
    if ((micros() - _lastDecReadTime2) < _pauseLength) {
      change *= _fastIncrement;
    }
    _lastDecReadTime2 = micros();
    currCount2 += change;
    encval2 = 0;
  }
}

