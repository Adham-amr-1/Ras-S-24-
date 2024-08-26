/* MACROS FOR MANUAL MODE*/
#define MOVE_FORWARD   'F'
#define MOVE_BACKWARD  'B' 
#define MOVE_LEFT      'L' 
#define MOVE_RIGHT     'R'
#define STOP           'S' 
#define GATES_UP       'U'
#define SIGMA_SHOOT    'X'
#define GATES_DOWN     'D' 
#define AUTOMATIC_MODE 'A'
#define MANUAL_MODE    'a'
#define DONE_SHOOTING  'Y'
/* MOTORs Speed */
#define MAX_SPEED              255  // Maximum speed for motors
#define ROTATE_SPEED           120
#define LINE_FOLLOWER_SPEED    100
//#define BASE_SPEED             180 // Base speed for straight movement
/* NEEDED Parameters for PID Algorithm */
/* Parameters for PID Algorithm */
#define KP 0.8   // Proportional constant
#define KD 0.2   // Derivative constant
#define KI 0.001  // Integral constant
//-----------------------------------------------------------------
/* MACROS FOR PINS */
/* MOTOR PINS */
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 7
#define ENA 5
#define ENB 6
//-----------------------------------------------------------------
/* IR Sensors Pins ( Random Numbers ) */
//#define FAR_RIGHT_IR_SENSOR 5       // single module  
//#define FAR_LEFT_IR_SENSOR  6      //  single module  
#define RIGHT_IR_SENSOR     A1     //   triple module
#define CENTER_IR_SENSOR    A2    //    triple module
#define LEFT_IR_SENSOR     A0   //     triple module
// To get IR Reading
//unsigned char FAR_RIGHT_IR , FAR_LEFT_IR;
unsigned char CENTER_IR, RIGHT_IR, LEFT_IR; 
// Idicating that the following is the IR Sensor Reading
#define BLACK HIGH 
#define WHITE LOW
//-----------------------------------------------------------------
// Mechanism Motors
// Gates
#define RIGHT_CABLE_GATE_MOTOR 12   // IN 1 ( Taraf mn El Motors )
#define LEFT_CABLE_GATE_MOTOR  10  //  IN 3 ( Taraf mn El Motors )
#define GATE_ENABLE 11
// Sigma Shoot 
//#define RIGHT_SOLENOID_MOTOR   9   // ( Taraf mn El Motors )
//#define LEFT_SOLENOID_MOTOR   8  //  ( Taraf mn El Motors )
#define SOLENOID_MOTORS     8 // ( Taraf mn El Motors )
//-----------------------------------------------------------------
// Checking Chars
// Check if i'm in automatic mode or not
bool AutoMode = false;
// Direction ( Car Movement )
char Direction;
// check the case to change from manual to auto and vice versa
char cases;
//-----------------------------------------------------------------
/* PID Algorithm Variables*/
// PID variables
float previousError = 0;
float integral = 0;
float error;
int correction;
// Change Motors speed
int LEFT_MOTOR_SPEED ;
int RIGHT_MOTOR_SPEED ;
// Calculate PID terms
float P;
float D;
float I;
/* IR Analog Reading */
int LEFT_IR_ANALOG;
int RIGHT_IR_ANALOG ;
//-----------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  //-----------------------------------------------------------------
  /* IR Sensors Initialization */
  //pinMode(FAR_RIGHT_IR_SENSOR, INPUT);
  //pinMode(FAR_LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  pinMode(CENTER_IR_SENSOR, INPUT); 
  pinMode(LEFT_IR_SENSOR, INPUT);
  //-----------------------------------------------------------------
  /* MOTOR INITIALIZATION */
  // Initialize motor driver pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stope(); 
  //-----------------------------------------------------------------
  // Mechanism Motors
  pinMode(SOLENOID_MOTORS, OUTPUT);
  //pinMode(RIGHT_SOLENOID_MOTOR, OUTPUT);
  //pinMode(LEFT_SOLENOID_MOTOR, OUTPUT);
  pinMode(RIGHT_CABLE_GATE_MOTOR, OUTPUT);
  pinMode(LEFT_CABLE_GATE_MOTOR, OUTPUT);
  pinMode(GATE_ENABLE,OUTPUT);
  //-----------------------------------------------------------------
}
//-----------------------------------------------------------------
 void loop() {
//--------------------------------------------------------------------------------------
/* =================================== Manual Mode =================================== */
//--------------------------------------------------------------------------------------
   if(Serial.available()){
        Direction = Serial.read();
        switch(Direction){
          // Move Forward
          case MOVE_FORWARD :
          {
            forward(MAX_SPEED);
            break;
          }
          // Move Back
          case MOVE_BACKWARD :
          {
            Back(MAX_SPEED);
            break;
          }
          // Move Left
          case MOVE_LEFT:
          {
            Left(MAX_SPEED);
            break;
          }
          // Move Right
          case MOVE_RIGHT :
          {
            Right(MAX_SPEED);
            break;
          }
          // At Release the buttons car stop
          case STOP :
          {
            stope();
            stopGates();
            break;
          }
          // Turn To Auto Mode ( Line Follower )
          case AUTOMATIC_MODE :
          {
            AutoMode = true;
            break;
          }
          // Mechanisms
          case GATES_UP :
          {
            gatesUp();
            break;
          }
          case GATES_DOWN :
          {
            gatesDown();
            break;
          }
          //Shoot Mechanism
          case SIGMA_SHOOT:{
            sigmaShoot();
            break;
          } 
          case DONE_SHOOTING :{
            Done_Shooting();
            break;
          }
      }
     }
//--------------------------------------------------------------------------------------
/* ================================= Automatic Mode ================================= */
//--------------------------------------------------------------------------------------
    while (AutoMode){
      
      /*FAR_RIGHT_IR = digitalRead(FAR_RIGHT_IR_SENSOR);
      FAR_LEFT_IR  = digitalRead(LEFT_RIGHT_IR_SENSOR);*/
      RIGHT_IR     = digitalRead(RIGHT_IR_SENSOR);
      LEFT_IR      = digitalRead(LEFT_IR_SENSOR);
      CENTER_IR    = digitalRead(CENTER_IR_SENSOR);
      //-----------------------------------------------------------------
      if((CENTER_IR == BLACK) && (RIGHT_IR == WHITE) && (LEFT_IR == WHITE))
      {
        forward(LINE_FOLLOWER_SPEED);
      }
      else if((CENTER_IR == WHITE) && (RIGHT_IR == BLACK) && (LEFT_IR == WHITE))
      {
        while(CENTER_IR == WHITE)
        {
          Right(ROTATE_SPEED);
          CENTER_IR    = digitalRead(CENTER_IR_SENSOR);
        }
      }
      else if((CENTER_IR == WHITE) && (RIGHT_IR == WHITE) && (LEFT_IR == BLACK))
      {
        while(CENTER_IR == WHITE)
        {
          Left(ROTATE_SPEED);
          CENTER_IR    = digitalRead(CENTER_IR_SENSOR);
        }
      }
      else if((CENTER_IR == BLACK) && (RIGHT_IR == BLACK) && (LEFT_IR == WHITE))
      {
        do{
          Right(ROTATE_SPEED);
          CENTER_IR    = digitalRead(CENTER_IR_SENSOR);
        }while(CENTER_IR == WHITE);
      }
      else if((CENTER_IR == BLACK) && (RIGHT_IR == WHITE) && (LEFT_IR == BLACK))
      {
        do{
          Left(ROTATE_SPEED);
          CENTER_IR    = digitalRead(CENTER_IR_SENSOR);
        }while(CENTER_IR == WHITE);
      }
      else{
        stope();
      }
      // -----------------------------------------------------------------
      /* PID Algorithm */
      /*LEFT_IR_ANALOG  = analogRead(LEFT_IR_SENSOR);
      RIGHT_IR_ANALOG = analogRead(RIGHT_IR_SENSOR);
      //-----------------------------------------------------------------
      // Calculate error
      error = LEFT_IR_ANALOG - RIGHT_IR_ANALOG;
      //-----------------------------------------------------------------
      // Calculate PID terms
      P = KP * error;
      D = KD * (error - previousError);
      integral += error;
      I = KI * integral;
      //-----------------------------------------------------------------
      // Calculate PID control output
      correction = P + I + D;
      //-----------------------------------------------------------------
      // Calculate motor speeds
      LEFT_MOTOR_SPEED  = BASE_SPEED + correction;
      RIGHT_MOTOR_SPEED = BASE_SPEED - correction;
      //-----------------------------------------------------------------
      // Constrain motor speeds to avoid overflow
      LEFT_MOTOR_SPEED = constrain(LEFT_MOTOR_SPEED, 0, MAX_SPEED);
      RIGHT_MOTOR_SPEED = constrain(RIGHT_MOTOR_SPEED, 0, MAX_SPEED);
      //-----------------------------------------------------------------
      // Apply speeds to motors
      digitalWrite(IN1,HIGH); 
      digitalWrite(IN2,LOW); 
      digitalWrite(IN3,HIGH); 
      digitalWrite(IN4,LOW); 
      /* Check Which EN Is Right and which is Left*/
      /*analogWrite(ENA,LEFT_MOTOR_SPEED); 
      analogWrite(ENB,RIGHT_MOTOR_SPEED);
      //-----------------------------------------------------------------
      // Update previous error
      previousError = error;
      //-----------------------------------------------------------------
      // stability
      delay(10);*/
      //-----------------------------------------------------------------
      // Checking if there's input that change it from auto to manual
      cases = Serial.read();
      if(cases == MANUAL_MODE){
        AutoMode = false;
        stope();
        break;
      }   
      //-----------------------------------------------------------------
    }
  }
void forward(unsigned char speed)
{
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW); 
  analogWrite(ENA,speed); 
  analogWrite(ENB,speed);
}
//-----------------------------------------------------------------
void Back(unsigned char speed)
{
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH);
  analogWrite(ENA,speed); 
  analogWrite(ENB,speed);
}
//-----------------------------------------------------------------
void Left(unsigned char speed)
{
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH); 
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW);
  analogWrite(ENA,speed); 
  analogWrite(ENB,speed);
}
//-----------------------------------------------------------------
void Right(unsigned char speed)
{
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH);
  analogWrite(ENA,speed); 
  analogWrite(ENB,speed);
}
//-----------------------------------------------------------------
void stope()
{
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,LOW); 
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,LOW);
  analogWrite(ENA,0); 
  analogWrite(ENB,0);
}
//-----------------------------------------------------------------
void sigmaShoot()
{
  digitalWrite(SOLENOID_MOTORS,HIGH); 
  //digitalWrite(RIGHT_SOLENOID_MOTOR,HIGH); 
  //digitalWrite(LEFT_SOLENOID_MOTOR,HIGH);
}
//-----------------------------------------------------------------
void Done_Shooting(){
  
  digitalWrite(SOLENOID_MOTORS,LOW); 
  //digitalWrite(RIGHT_SOLENOID_MOTOR,LOW); 
  //digitalWrite(LEFT_SOLENOID_MOTOR,LOW);
}
//-----------------------------------------------------------------
void gatesUp()
{
  digitalWrite(RIGHT_CABLE_GATE_MOTOR,HIGH); 
  digitalWrite(LEFT_CABLE_GATE_MOTOR,LOW); 
  analogWrite(GATE_ENABLE,255);
}
//-----------------------------------------------------------------
void gatesDown()
{
    digitalWrite(RIGHT_CABLE_GATE_MOTOR,LOW); 
    digitalWrite(LEFT_CABLE_GATE_MOTOR,HIGH); 
    analogWrite(GATE_ENABLE,255);
}
void stopGates(){
    digitalWrite(RIGHT_CABLE_GATE_MOTOR,LOW); 
    digitalWrite(LEFT_CABLE_GATE_MOTOR,HIGH); 
    analogWrite(GATE_ENABLE,0);
}
