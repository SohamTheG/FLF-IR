#include <QTRSensors.h>

#define Kp 2.5// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 1.3
 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 255 // max speed of the robot
#define leftMaxSpeed 255 // max speed of the robot
#define rightBaseSpeed 40 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed  40 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low

#define rightMotor1 6
#define rightMotor2 7
#define rightMotorPWM 10
#define leftMotor1 4
#define leftMotor2 5
#define leftMotorPWM 9

QTRSensors qtrrc;

unsigned int sensorValues[8];


void calib(){
  int i;
  // stops the motor
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(13,HIGH);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 0);
  
  delay(5000);
  for (int i = 0; i < 400; i++){ // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

    /*comment this part out for automatic calibration
      if ( i  < 100 || i >= 300 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
      {  turn_right();}
      else
      {turn_left();} */
    Serial.println("calibarating..");
    qtrrc.calibrate();
    }
  delay(20);
  digitalWrite(13,LOW);
  delay(2000); // wait for 2s to position the bot before entering the main loop

}

void setup()
{
  Serial.begin(9600);
  qtrrc.setTypeRC();
  qtrrc.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,12,3}, 8);
  qtrrc.setEmitterPin(11);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);                                                                                                       
  pinMode(leftMotorPWM, OUTPUT);

  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  //analogWrite(leftMotorPWM, 100);

  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  //analogWrite(rightMotorPWM, 100);

  //comment out for serial printing
/*
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  */
  
  attachInterrupt (digitalPinToInterrupt (2), calib, FALLING);
}

int lastError = 0;

/*void reverse(){
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 50);

  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, 50);
}*/

void loop()
{
    
  
  unsigned int sensors[8];
  int position = qtrrc.readLineBlack(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  //Serial.println(position);
  if(position == 0 && sensors[0] <= 300 ){
    digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM,70);

  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM,70);

  delay(50);

  }
  else{
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);


  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);

  int error = ((position - 3500)/100);
  
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  Serial.println(motorSpeed);  
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)
  { rightMotorSpeed = 0;} // keep the motor speed positive
  if (leftMotorSpeed < 0) 
  {leftMotorSpeed = 0;} // keep the motor speed positive

  // Writing the motor speed value as output to hardware motor
  analogWrite(rightMotorPWM,rightMotorSpeed);
  Serial.println("------------");
  Serial.println( rightMotorSpeed );
  analogWrite(leftMotorPWM,leftMotorSpeed);
  Serial.println(leftMotorSpeed);
  Serial.println("------------");
  delay(20);
  }
  
}

