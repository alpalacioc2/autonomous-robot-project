


void motorInit(void);
void directionInit(void);
void pwmInit(void);
void setMotorSpeed(int channel, int duty);
void setDirection(int channel, int dir);

void motorInit(void)
            {
              pwmInit();
              directionInit();
            }


void directionInit(void){
                //pinMode(50,OUTPUT); //motor 1 I1 - PB3
                //pinMode(51,OUTPUT); //motor 1 I2 - PB2
                //pinMode(52,OUTPUT); //motor 0 I3 - PB1
                //pinMode(53,OUTPUT); //motor 0 I4 - PB0
                DDRB |=0b1111;
                
            } /*end of direction_init*/

void pwmInit(void){//pwm set up

                //pinMode(45, OUTPUT); //motor 0
                //pinMode(46, OUTPUT); //motor 1

                DDRL |= 0b00011000;
                TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(WGM52) | _BV(WGM50);
                TCCR5B = _BV(CS51) | _BV(CS50);  //set prescaler to 128
                OCR5A  =  0;
                OCR5B  =  0;
            } /*end of pwm_init*/

//void setMotorSpeed(int channel, int duty)
 //           {
 //               //pwmSetDuty(channel,duty);


 //           }

void setMotorSpeed(int channel, int duty)
            { 
               
                if (duty > 255){  duty = 255; }
                else if(duty < -255) {duty = -255; }
            
                  
                if (duty>0) { setDirection(channel,1);  }
                else if(duty<0) { setDirection(channel,0);   //reverse
                    duty = -duty;
                    }
                    
                if (channel==0)  //motor 0 (left motor)
                    OCR5B = duty; //pin45
                    
                else if (channel ==1)//motor 1( Rightmotor)
                    OCR5A = duty; //pin46
            }
            /*end of pwn_set_duty*/

void setDirection(int channel, int dir)//set motor direction
            {
                if ((channel == 0) && (dir == 0))//reverse 
                   {
                    //digitalWrite(52,LOW);  //I1
                    //digitalWrite(53,HIGH); //I2
                    PORTB &=0b1101;
                    PORTB |=0b0001;
                    }
                else if ((channel == 0) && (dir == 1))
                   {
                    //digitalWrite(52,HIGH); //I1
                    //digitalWrite(53,LOW);  //I2
                    
                    PORTB |=0b0010;
                    PORTB &=0b1110;

                   }
                    
                
                else if((channel == 1) && (dir==0)) //motor 1(Right)
                 {
                  
                    //digitalWrite(50,LOW);  //I3
                    //digitalWrite(51,HIGH); //I4
                    PORTB &= 0b0111;
                    PORTB |= 0b0100;
                 }
                  
                else if ((channel ==1) && (dir == 1)) //forward
                   {
                    //digitalWrite(50,HIGH); //I3
                    //digitalWrite(51,LOW);  //I4
                    PORTB |=0b1000;
                    PORTB &=0b1011;

                   }
                 
            } /*end set_direction*/
            