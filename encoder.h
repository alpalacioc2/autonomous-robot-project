


void encoder_init(void);
ISR(INT4_vect);
ISR(INT5_vect);
ISR(PCINT2_vect);


long int encoder_val0 =0; 
long int encoder_val1 =0;

int enc_state[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
int enc_index0 = 0;
int enc_index1 = 0; 

void encoder_init(void)
{
  /*Configure Encoder 0 - Wheel 0*/
  DDRE &=0b11001111;
  enc_index0 = (PINE>>4)&0b0011;
  /*Set falling and rising edged trigger the interrupt*/
  EICRB |= 0b00000101;
  /*Enable INT4 and INT5*/
  EIMSK = EIMSK | 0b00110000;
  
  /*Configure Encoder 1 - Wheel 1*/
  DDRK &=0b00111111;
  enc_index1 = (PINK>>6)&0b0011;
  PCICR |= 0b100;
  PCMSK2 |=0b11000000;

}

  ISR(INT4_vect)
  {
    enc_index0 <<=2;
    enc_index0 |= (PINE>>4)&0b0011; 
    enc_index0 &=0x0f;
    encoder_val0 +=enc_state[enc_index0];
   
    
  }
    ISR(INT5_vect)
  {
    enc_index0 <<=2;
    enc_index0 |= (PINE>>4)&0b0011; 
    enc_index0 &=0x0f;
    encoder_val0 +=enc_state[enc_index0];

  }

  ISR(PCINT2_vect)
  {
    enc_index1 <<=2;
    enc_index1 |= (PINK>>6)&0b0011; 
    enc_index1 &=0x0f;
    encoder_val1 +=enc_state[enc_index1];
  }
