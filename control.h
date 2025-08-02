

void getCurrentStatus(void);
void lowLevelControl(void);

#define END_CPR     100
#define Gear_Ratio  30
#define T           0.01  // 10 msec



float wheelCurrPos[2]  = {0.,0.};  // current wheel position 0 - left  1 - right
float wheelCurrVel[2]  = {0.,0.};  // current wheel velocity 0  left   1 right
float wheelDesPos[2]  = {0.,0.};  // desired wheel position 
float wheelDesVel[2]  = {0.,0.};  // desired wheel velocity 
float wheelPrevPos[2] = {0.,0.};  // previous wheel position
float wheelPrevDesPos[2] = {0.,0.};  // previous desired wheel position


void getCurrentStatus(void)
{
  int i;
 
  /*determine current wheel position in rad*/

  wheelCurrPos[0] = encoder_val0*2*PI/(4.0*END_CPR*Gear_Ratio);
  wheelCurrPos[1] = encoder_val1*2*PI/(4.0*END_CPR*Gear_Ratio);


  for(i=0;i<2;i++)
  { 
     wheelCurrVel[i]  = (wheelCurrPos[i] - wheelPrevPos[i])/T;
     wheelPrevPos[i]  = wheelCurrPos[i];
  }

  /*determine des_wheel position*/

  for (i=0;i<2;i++)
  {
  wheelDesPos[i] = wheelPrevDesPos[i] + wheelDesVel[i]*T;
  wheelPrevDesPos[i] = wheelDesPos[i];
  }
 
}

void lowLevelControl(void)
{
  float voltage[2] ={0.,0.};
  float Kp = 50.;
  float Kd = 5.;
  int   duty = 0;
  float Vcc  = 7.2; /* Battery Voltage*/


  /*PD Control*/

  
  voltage[0] = Kp*(wheelDesPos[0]-wheelCurrPos[0]) + Kd*(wheelDesVel[0] - wheelCurrVel[0]);
  duty      = (int) voltage[0]/Vcc * 255;
  
  setMotorSpeed(0,duty);

  voltage[1] = Kp*(wheelDesPos[1]- wheelCurrPos[1]) + Kd*(wheelDesVel[1] - wheelCurrVel[1]);
  duty       = (int) voltage[1]/Vcc * 255;
  
  setMotorSpeed(1,duty);
  
}



