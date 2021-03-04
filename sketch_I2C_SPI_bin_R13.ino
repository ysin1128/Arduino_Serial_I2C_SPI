#include <SPI.h>

byte REV_Code[] = {0x00, 0x13}; // [0]: free, [1] Rev code of sketch
byte CMD;  // 0xAA = Read, 0xA5 = Write, 0x55 = GPIO, 0x5A = SPI, 0xC3 = Pattern, 0x69 = Clock, 0x96 = Config

int led=13;
int intTO = 100;

int i;
int j;

byte PAT[256];
int PAT_WD;
int PAT_LEN;
int PAT_EN;
int PAT_CNT;
int PAT_RPT;
int PAT_CYC;
int PAT_UNIT = 0; 

int cnt_to;
byte ADR;
byte inDAT[512]; //no delimiter, byte data only
//byte outDAT[15];
int cntDAT;
byte byteDAT;
int LNG; // Data length for Read
byte VAL;
byte RC;
int DLY; 
byte SPI_BO;
byte SPI_CD;
byte SPI_DM;
int intAVAL[2];
byte byteAVAL[4];
byte Item;
byte Param;
uint16_t pageTMP;

// for I2C functions
const int CONST_TIMEOUT = 100;
const byte SC_SUCCESS = 0xFF;
const byte SC_TIMEOUT = 0x80;
const byte SC_NACK = 0x00;

const byte SC_SUCCESS_GPIO = 0xFD;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  pinMode(SDA, INPUT);  //Internal Pull-up is disabled
  pinMode(SCL, INPUT);  //Internal Pull-up is disabled
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(9, OUTPUT);

  TCCR1A = TCCR1A & B00111100;
  TCCR1A = TCCR1A | B00000010;
  TCCR1B = TCCR1B & B11100000;
  TCCR1B = TCCR1B | B00011001;

  PAT_WD = 1000;
  PAT_LEN = 0;
  PAT_EN = 0;
  PAT_CNT = 0;
  PAT_RPT = 0;
  PAT_CYC = 0;

  for(i=0;i<256;i++){
    PAT[i] = 0;
  }

  TWSR &= ~_BV(TWPS1)&~_BV(TWPS0);  //I2C SCL setting
  TWBR = 72;                        //SCL=100kHz @16MHz

//for Debug.
//SPI_begin has to be disabled.
//digitalWrite(led, HIGH);
//delay(100);
//digitalWrite(led, LOW);
//delay(500);
}

void loop() {  
  cntDAT = 0;
   
  while(Serial.available()){    
    delay(5);
    
    inDAT[cntDAT] = Serial.read();
    cntDAT = cntDAT + 1;

    if(cntDAT == 512){
      break;
    }
  }

  if(cntDAT == 1){
    REV_Code[0] = inDAT[0];
    Serial.write(REV_Code, 2);
  }

  if(cntDAT > 2){    
    CMD = inDAT[0];

    if(CMD == 0xAA){   
      LNG = inDAT[1];
      ADR = inDAT[2];

/*             
      RC = i2c_read(ADR, inDAT + 3, cntDAT -3, outDAT, LNG);
      Serial.write(RC);

      if(RC == 0xFF){
        for(i=0;i<LNG;i++){
          Serial.write(outDAT[i]);
        }
      }
*/

      RC = i2c_read_ready(ADR, inDAT + 3, cntDAT - 3);
      Serial.write(RC);

      if(RC == SC_SUCCESS){
        for(i=0;i<LNG;i++){
          if(i == (LNG - 1)){
            byteDAT = i2c_get_data(false);            
          }
          else{
            byteDAT = i2c_get_data(true);
          }
          Serial.write(byteDAT);
        }                 
        i2c_stop();
      }

    }
    else if(CMD == 0xA5){
      ADR = inDAT[2];
      RC = i2c_write(ADR, inDAT + 3, cntDAT - 3);
      Serial.write(RC);            
    }
    else if(CMD == 0x55 && cntDAT > 3) {
      Serial.write(SC_SUCCESS_GPIO);

      PAT_EN = 0;
      PAT_CNT = 0;
      PAT_RPT = 0;

      DLY = inDAT[1] * 256 + inDAT[2];
      PAT_WD = DLY;
      PAT_LEN = cntDAT - 3;

      for(i=3;i<cntDAT;i++){
        byteDAT = inDAT[i];
        PAT[i-3] = byteDAT;

        CTRL_D72(byteDAT);

        delay(DLY);

        byteDAT = (PIND >> 2);

        Serial.write(byteDAT);

        for(j=0;j<2;j++){
          intAVAL[j] = analogRead(1-j);
          byteAVAL[2*j] = intAVAL[j] / 256;
          byteAVAL[(2*j)+1] = intAVAL[j] % 256;
          Serial.write(byteAVAL[2*j]);
          Serial.write(byteAVAL[(2*j)+1]);
        }
      }
    }
    else if(CMD == 0x5A){
      SPI_BO = inDAT[1] & 0x20;
      SPI_BO = SPI_BO >> 5;

      SPI_CD = inDAT[1] & 0x1C;
      SPI_CD = SPI_CD >> 2;
      SPI_DM = inDAT[1] & 0x03;

      SET_SPI_PARAM(SPI_BO, SPI_CD, SPI_DM);

      Serial.write(SC_SUCCESS);

      digitalWrite(SS, LOW);

      for(i=2;i<cntDAT;i++){
        byteDAT = SPI.transfer(inDAT[i]);
        Serial.write(byteDAT);
      }

      digitalWrite(SS, HIGH);
      
    }
    else if(CMD == 0xC3){
      if(inDAT[1] > 0){
        PAT_EN = 1;
        PAT_CYC = 0;
        PAT_RPT = inDAT[2];
        Serial.write(0xFA);
      }
      else{
        PAT_EN = 0;
        Serial.write(0xF0);
      }
    }
    else if(CMD == 0x69){
      if(inDAT[1] == 0 && inDAT[2] == 0){
        en_clk(false);
        Serial.write(0xFB);
      }
      else{
        ICR1H = inDAT[1];
        ICR1L = inDAT[2];

        if(cntDAT < 5){
          pageTMP = ((inDAT[1] << 8) & 0xFF00) | (inDAT[2] & 0x00FF);
          pageTMP = (pageTMP >> 1);
          inDAT[3] = (pageTMP >> 8) & 0xFF;
          inDAT[4] = pageTMP & 0xFF;
        }

        OCR1AH = inDAT[3];
        OCR1AL = inDAT[4];        

        delay(10);

        en_clk(true);

        Serial.write(0xFC);
      }
    }
    else if(CMD == 0x96){
      Item = inDAT[1];
      Param = inDAT[2];

      switch(Item){
        case 1: // Item = 1: analogReference
          switch(Param){
            case 0:
              analogReference(DEFAULT);
              break;
            case 1:
              analogReference(EXTERNAL);
              break;
            case 2:
              analogReference(INTERNAL);
              break;
          }
          Serial.write(SC_SUCCESS);
          break;
        case 2: // Item = 2: SCK/SCL Pull-up
          switch(Param){
            case 0:
              pinMode(SDA, INPUT);
              pinMode(SCL, INPUT);
              break;
            case 1:
              pinMode(SDA, INPUT_PULLUP);
              pinMode(SCL, INPUT_PULLUP);
              break;
          }
          Serial.write(SC_SUCCESS);
          break;
        case 3: // Item = 3: SCL Freq
          TWBR = Param;
          Serial.write(SC_SUCCESS);
          break;
        case 4: // Item = 4: unit of delay time for pattern gen
          PAT_UNIT = Param;
          break;        
      }
    }
  }

  if(PAT_EN == 1){
    if(PAT_CNT >= PAT_LEN){
      PAT_CNT = 0;
      PAT_CYC = PAT_CYC + 1;

      if((PAT_CYC >= PAT_RPT) && (PAT_RPT != 0)){
        PAT_EN = 0;
      }
    }

    byteDAT = PAT[PAT_CNT];
    CTRL_D72(byteDAT);
    PAT_CNT = PAT_CNT + 1;

    if(PAT_UNIT == 0){
      delay(PAT_WD);
    }
    else{
      delayMicroseconds(PAT_WD);
    }
  }
  else{
    delay(100);
  }
}

byte i2c_write(byte slave_adr, byte *data, int data_length){
  byte status_code;

  status_code = i2c_main(slave_adr, data, data_length);
  i2c_stop();

  return status_code;
}

byte i2c_read(byte slave_adr, byte *reg_adr, int reg_adr_length, byte *read_data, int read_data_length){
  int i_i2c;
  byte status_code;

  status_code = i2c_main(slave_adr, reg_adr, reg_adr_length);

  if(status_code != SC_SUCCESS){
    return status_code;
  }
  
  status_code = i2c_start();

  if(status_code != SC_SUCCESS){
    return status_code;
  }

  status_code = i2c_send_data((slave_adr<<1) + 1);
    
  if(status_code != SC_SUCCESS){
    return status_code;
  }

  for(i_i2c=0;i_i2c<read_data_length;i_i2c++){
    if(i_i2c == (read_data_length - 1)){
      read_data[i_i2c] = i2c_get_data(false);          
    }
    else{
      read_data[i_i2c] = i2c_get_data(true);    
    }
  }

  i2c_stop();
  return SC_SUCCESS;
}

byte i2c_read_ready(byte slave_adr, byte *reg_adr, int reg_adr_length){
  int i_i2c;
  byte status_code;

  status_code = i2c_main(slave_adr, reg_adr, reg_adr_length);

  if(status_code != SC_SUCCESS){
    return status_code;
  }

  status_code = i2c_start();

  if(status_code != SC_SUCCESS){
    return status_code;
  }

  return i2c_send_data((slave_adr<<1) + 1);
}

byte i2c_main(byte slave_adr, byte *data, int data_length){
  int i_i2c;
  byte status_code;

  status_code = i2c_start();

  if(status_code != SC_SUCCESS){
    return status_code;
  }

  status_code = i2c_send_data(slave_adr<<1);
    
  if(status_code != SC_SUCCESS){
    return status_code;
  }

  for(i_i2c=0;i_i2c<data_length;i_i2c++){
    status_code = i2c_send_data(data[i_i2c]);

    if(status_code != SC_SUCCESS){
      return status_code;
    }
  }

  return SC_SUCCESS;
}

byte i2c_start(){
  TWCR = _BV(TWINT)|_BV(TWSTA)|_BV(TWEN);
  return i2c_get_status_code(CONST_TIMEOUT);
}

void i2c_stop(){
  TWCR = _BV(TWINT)|_BV(TWSTO)|_BV(TWEN);
}

byte i2c_send_data(byte data){
  TWDR = data;
  TWCR = _BV(TWINT)|_BV(TWEN);

  return i2c_get_status_code(CONST_TIMEOUT);
}

byte i2c_get_data(bool repeat){
  byte status_code;

  if(repeat){
    TWCR = _BV(TWEA)|_BV(TWINT)|_BV(TWEN);    
  }
  else{
    TWCR = _BV(TWINT)|_BV(TWEN);        
  }

  if(i2c_get_status_code(CONST_TIMEOUT) == SC_SUCCESS){
    return TWDR;
  }
  else{
    return 0xFF;
  }
}

byte i2c_get_status_code(int timeout){
  byte status_code;
  int cnt_timeout = 0;

  while((TWCR & _BV(TWINT)) == 0){
    if(cnt_timeout == timeout){
      return SC_TIMEOUT;
    }

    cnt_timeout++;
    
    delayMicroseconds(100);  
  }

  status_code = i2c_status_code();

  if(status_code != SC_SUCCESS){
    i2c_stop();
  }

  return status_code;
}

byte i2c_status_code(){
  switch(TWSR & 0xF8){
    case 0x08:  //Stat condition
      return SC_SUCCESS;
    case 0x10:  //Repeted Start condition
      return SC_SUCCESS;
    case 0x18:  //Slave address + W ACK
      return SC_SUCCESS;
    case 0x20:  //Slave address + W NACK
      return SC_NACK;
    case 0x28:  //Data send ACK
      return SC_SUCCESS;
    case 0x30:  //Data send NACK
      return SC_NACK;
    case 0x40:  //Slave address + R ACK
      return SC_SUCCESS;
    case 0x48:  //Slave address + R NACK
      return SC_NACK;
    case 0x50:  //Data read ACK
      return SC_SUCCESS;
    case 0x58:  //Data read NACK
      return SC_SUCCESS;
    default:
      return SC_TIMEOUT;
  }
}

void CTRL_D72(byte byteDAT){
  DDRD = (DDRD & 0x3F) | (byteDAT & 0xC0);
  PORTD = (PORTD & 0x03) | ((byteDAT & 0x3F) << 2); 
}

boolean CK_BIT(byte byteDAT, int intDIG){
  byte byteTMP = 1;

  if(intDIG < 8){
    byteTMP = byteTMP << intDIG;
  }
  else{
    byteTMP = byteTMP << 7;
  }

  if((byteDAT & byteTMP) > 0){
    return true;
  }
  else{
    return false;
  }
}

void SET_SPI_PARAM(byte byteSPI_BO, byte byteSPI_CD, byte byteSPI_DM){
  if(byteSPI_BO == 0){
    SPI.setBitOrder(MSBFIRST);
  }
  else{
    SPI.setBitOrder(LSBFIRST);
  }

  switch(byteSPI_CD){
    case 0:
      SPI.setClockDivider(SPI_CLOCK_DIV2);
      break;
    case 1:
      SPI.setClockDivider(SPI_CLOCK_DIV4);
      break;
    case 2:
      SPI.setClockDivider(SPI_CLOCK_DIV8);
      break;
    case 3:
      SPI.setClockDivider(SPI_CLOCK_DIV16);
      break;
    case 4:
      SPI.setClockDivider(SPI_CLOCK_DIV32);
      break;
    case 5:
      SPI.setClockDivider(SPI_CLOCK_DIV64);
      break;        
    default:
      SPI.setClockDivider(SPI_CLOCK_DIV128);
      break;
  }

  switch(byteSPI_DM){
    case 0:
      SPI.setDataMode(SPI_MODE0);
      break;
    case 1:
      SPI.setDataMode(SPI_MODE1);
      break;
    case 2:
      SPI.setDataMode(SPI_MODE2);
      break;
    case 3:
      SPI.setDataMode(SPI_MODE3);
      break;
  }
  
}

void en_clk(boolean cEN){
  TCCR1A = TCCR1A & B00111111;

  if(cEN){
    TCCR1A = TCCR1A | B10000000;
  }
}
