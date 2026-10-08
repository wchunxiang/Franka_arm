#include <AccelStepper.h>

String sdata="";
int pos_step = 0;
int pos_before = 0;

bool b_stop = false;


// Define a stepper and the pins it will use
//AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

// Define the AccelStepper interface type:
#define MotorInterfaceType 4

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(MotorInterfaceType, 8, 9, 10, 11);

int steps_per_rot = 400;


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
//  pinMode(RESET_PIN, OUTPUT);
//  digitalWrite(RESET_PIN, HIGH);
//  delay(3);
//  digitalWrite(RESET_PIN, LOW);
//  delay(1);
  
  Serial.begin(9600);
  stepper.setMaxSpeed(steps_per_rot);
  stepper.setAcceleration(10*steps_per_rot);
}

void loop()
{
//  if (pos_step==pos_before)
//  {int x = 0;}
//  else{
//  stepper.moveTo(pos_step);
//  if (stepper.distanceToGo() != 0)
//  {stepper.run();}
//  else
//  {pos_before = pos_step;}
//  }

  //if (b_continuous==false)
  {
  stepper.moveTo(pos_step);
  if (stepper.distanceToGo() != 0)
  {stepper.run();}
  }
//  else
//  {}

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
void serialEvent()
{
   while(Serial.available()) 
   {
      char ch = Serial.read();
      sdata+= (char)ch;

      if (ch==','){
        sdata.trim();

        //process command
        char ch00 = sdata.charAt(0);
        //if (stepper.distanceToGo() == 0)
            {
             length = sdata.length();
             sub_str = sdata.substring(1,length);
      
             double value = sub_str.toDouble();
             int value_step = (int)(value/360*steps_per_rot);
             
             sdata = "";
             
             if (ch00 == 'r'){
              b_stop = false;
              resetFunc();}

              
             if (ch00 == 'a'){
              b_stop = false;
              stepper.setAcceleration(value_step);}
              
             if (ch00 == 's'){
              b_stop = false;
              stepper.setMaxSpeed(value_step);}
              
              if (ch00 == 'p'){
                b_stop = false;
                 pos_step = value_step;
      //         Serial.println(pos_step);
      //         Serial.println('\r');
      //         Serial.println(value);
              }
              }
          
        sdata = "";
        
        }
       
   }
}
