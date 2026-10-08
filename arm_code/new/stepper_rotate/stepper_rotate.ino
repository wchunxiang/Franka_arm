
#include <AccelStepper.h>


String sdata="";

AccelStepper stepper(AccelStepper::DRIVER, 9, 8);


int steps_per_rot = 800;
int pos = steps_per_rot;

int stepper_vel = steps_per_rot*0.1;


bool b_run = 0;
bool b_direction = 1;
int value_step;

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
  stepper.setMaxSpeed(10*steps_per_rot);
  stepper.setAcceleration(10*steps_per_rot);

}

void loop()
{
if (b_direction)
{
  
  if (b_run)
  {  stepper.setSpeed(stepper_vel);
  stepper.runSpeed();}
  else
  {stepper.stop();}
  }
else
{
  if (b_run)
  {  stepper.setSpeed(-1*stepper_vel);
  stepper.runSpeed();}
   else
  {stepper.stop();}
  }

  
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
              stepper_vel = stepper_vel+int(value);
              
              }
             if (ch00 == 'p'){
              b_run = 1;
//              int pos = int(value/360*steps_per_rot);
//              int pos_current = move_pos(pos, stepper_vel);
              }
              
             if (ch00 == 'r'){
              b_run = 1;
              if (int(value)>0)
              {b_direction =1;}
              else
              {b_direction =0;}
              
              }
             if (ch00 == 's'){
              b_run = 0;
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
