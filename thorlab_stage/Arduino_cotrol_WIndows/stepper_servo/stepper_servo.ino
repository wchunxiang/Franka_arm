#include <AccelStepper.h>
#include <Servo.h>
String sdata="";

// Define a stepper and the pins it will use
//AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

// Define the AccelStepper interface type:
#define MotorInterfaceType 4

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(MotorInterfaceType, 8, 9, 10, 11);
Servo myservo;


int steps_per_rot = 400;
int stepper_vel = steps_per_rot/2;

double speed_write = 8;
double speed_zero = 90;
bool b_stop;
int value_step;
int pos_step;

const int MaxChars = 4;
char strValue[MaxChars+1];
int index = 0;
//others do not change
int length = 00;
String sub_str = "";

const int RESET_PIN = 2;

void(* resetFunc) (void) = 0;

void setup()
{ 
  Serial.begin(9600);
  stepper.setMaxSpeed(steps_per_rot);
  stepper.setAcceleration(10*steps_per_rot);
  myservo.attach(3);
  int pos_current = move_pos(1, stepper_vel);
  pos_current = move_pos(0, stepper_vel);
  
}

void loop()
{
//  move_servo(1000);
//  delay(1000);

}


void serialEvent()
{
   while(Serial.available()) 
   {
//      char ch = Serial.read();
//      sdata+= (char)ch;
      sdata = Serial.readStringUntil(',');
      Serial.print(sdata);
//      if (ch==',')
      {
        //sdata.trim();

        //process command
        char ch00 = sdata.charAt(0);
            {
             length = sdata.length();
             sub_str = sdata.substring(1,length);
      
             double value = sub_str.toDouble();
             
             
             sdata = "";
          
             if (ch00 == 'v'){
              speed_write = int(value);
              }
             if (ch00 == 'p'){
              int pos = int(value/360*steps_per_rot);
              int pos_current = move_pos(pos, stepper_vel);
              }
              
             if (ch00 == 'r'){
              move_servo(int(value),speed_write);
              }
             if (ch00 == 's'){
              stop_servo(int(value));
              }
              
              
              }
          
        sdata = "";
        
        }
       
   }
}


int move_pos(int pos, int vel)
{
  stepper.setMaxSpeed(vel);
  stepper.moveTo(pos);
  while(stepper.distanceToGo() != 0)
  {stepper.run();
  }
  
  return stepper.currentPosition();
  
  }

void move_servo(int speed_time, int speed_)
{
    if (speed_time >0){
      myservo.write(speed_zero+speed_);
      //delay(speed_time);
      }
    else{
      //myservo.write(speed_zero);
       myservo.write(speed_zero-speed_);
       //delay(-1*(speed_time));
      }
     //myservo.write(speed_zero);
  }

void stop_servo(int speed_time)
{

     myservo.write(speed_zero);
  }


//void serialEvent()
//{
//   while(Serial.available()) 
//   {
//      char ch = Serial.read();
//      Serial.write(ch);
//      if(index < MaxChars && isDigit(ch)) { 
//            strValue[index++] = ch; 
//      } else { 
//            strValue[index] = 0; 
//            int pos = atoi(strValue); 
//            pos_step = (int)((double)(pos)/360*steps_per_rot);
////            if(newAngle >= 0 && newAngle <= 360){
////                target_angle = newAngle; 
////            }
////            else{
////              target_angle = 180
////              }
//            
//            index = 0;
//      }  
//   }
//}

//
  