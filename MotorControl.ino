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


// Assign motor ID and pins
motor motor1{1, 18, 19, 4, 21, 34};
motor motor2{2, 26, 27, 25, 33, 32};


// Variables for calculating RPM
static double RPM;
static double countDif;
const double timeDelay = 100;
const double clockSpeed = timeDelay / 1000; // [s]
const double N = 1328; // counts per revolution on motor 1


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




void loop() {
  static double prevCount = 0;


  motorSpeed(motor1, );
  motorSpeed(motor2, 0);
  Serial.println(vel(motor2, currCount2));
  delay(timeDelay);

}





void motorSpeed(const motor& m, int speed) {
  // Pass speed -255 to 255 

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
