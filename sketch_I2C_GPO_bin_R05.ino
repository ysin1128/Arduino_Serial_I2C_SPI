#include <Wire.h>
#include <SPI.h>

int led=13;

int i;
int j;
int k;

byte PAT[255];
int PAT_WD;
int PAT_LEN;
int PAT_EN;
int PAT_CNT;
int PAT_RPT;
int PAT_CYC; 

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  SPI.begin();
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
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

}

byte REV_Code[] = {0x00, 0x05}; // [0]: free, [1] Rev code of sketch

int CMD;  // 0xAA = Read, 0xA5 = Write, 0x55 = GPIO, 0x5A = SPI, 0xC3 = Pattern, 0x69 = Clock, 0x96 = Config
int cnt_to;
byte ADR; // 8-bit, LSB must be 0 both for Read/Write
byte inDAT[511]; //no delimiter, byte data only
byte byteDAT;
int LNG; // Data length for Read
byte VAL;
byte ACK;
int DLY; 
byte SPI_BO;
byte SPI_CD;
byte SPI_DM;
int intAVAL[1];
byte byteAVAL[3];
byte Item;
byte Param;


void loop() {  
  i = 0;
  CMD = 0;
   
  while(Serial.available()){
    delay(10);
    
    inDAT[i] = Serial.read();
    i = i + 1;

//    SPI_begin has to be disabled.
//    digitalWrite(led, HIGH);
//    delay(100);
//    digitalWrite(led, LOW);
//    delay(100);

    if(i == 512){
      break;
    }
  }

  if(i == 1){
    REV_Code[0] = inDAT[0];
    Serial.write(REV_Code, 2);
  }

  if(i > 2){    
    CMD = inDAT[0];

    if(CMD == 0xAA || CMD == 0xA5){      
      cnt_to = 0;

      while(1){
        if(digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH){
          break;
        }
        else{
          cnt_to = cnt_to + 1;
        }

        delay(100);

        if(cnt_to == 10){
          break;
        }
      }

      if(cnt_to == 10){
        Serial.write(0x80);
        return;
      }

      LNG = inDAT[1];
      ADR = inDAT[2];

      Wire.beginTransmission(ADR);
      
      for(j=3;j<i;j++){
       Wire.write(inDAT[j]);
      }

      if(CMD == 0xA5){
        ACK = Wire.endTransmission(true);
      }
      else{
        ACK = Wire.endTransmission(false);
        Wire.requestFrom(ADR,LNG,true);
      }        

      if(ACK == 0){
        Serial.write(0xFF);
      }
      else{
        Serial.write(0x00);
      }

      j=0;

      if((ACK == 0) && (CMD == 0xAA)) {
        for(j=0;j<LNG;j++){
          byteDAT = Wire.read();
          Serial.write(byteDAT);
        }        
      }
    }
    else if(CMD == 0x55 && i > 3) {
      PAT_EN = 0;
      PAT_CNT = 0;
      PAT_RPT = 0;
      
      DLY = inDAT[1] * 256 + inDAT[2];
      PAT_WD = DLY;
      PAT_LEN = i - 3;

      for(j=3;j<i;j++){
        byteDAT = inDAT[j];
        PAT[j-3] = byteDAT;

        for(k=0;k<5;k++){
          if(byteDAT%2 == 1){
            digitalWrite(2+k,HIGH);
          }
          else{
            digitalWrite(2+k,LOW);
          }
          byteDAT = byteDAT/2;
        }

        Serial.write(0xFF);
        
        delay(DLY);

        byteDAT = 0;

        if(digitalRead(7) == HIGH){
         byteDAT = byteDAT + 1; 
        }

        if(digitalRead(8) == HIGH){
         byteDAT = byteDAT + (1 * 2); 
        }

        Serial.write(byteDAT);

        for(k=0;k<2;k++){
          intAVAL[k] = analogRead(k);
          byteAVAL[2*k] = intAVAL[k] / 256;
          byteAVAL[2*k+1] = intAVAL[k] % 256;
          Serial.write(byteAVAL[2*k]);
          Serial.write(byteAVAL[2*k+1]);
        }
      }
    }
    else if(CMD == 0x5A){
      SPI_BO = inDAT[1] & 0x20;
      SPI_BO = SPI_BO >> 5;

      SPI_CD = inDAT[1] & 0x1C;
      SPI_CD = SPI_CD >> 2;

      SPI_DM = inDAT[1] & 0x03;

      if(SPI_BO == 0){
        SPI.setBitOrder(MSBFIRST);
      }
      else{
        SPI.setBitOrder(LSBFIRST);
      }

      switch(SPI_CD){
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

      switch(SPI_DM){
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

      Serial.write(0xFF);

      digitalWrite(SS, LOW);

      for(j=2;j<i;j++){
        byteDAT = SPI.transfer(inDAT[j]);
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
        TCCR1A = TCCR1A & B00111111;
        Serial.write(0xFB);
      }
      else{
        ICR1H = inDAT[1];
        ICR1L = inDAT[2];

        if(i < 5){
          inDAT[3] = inDAT[1] / 2;
          inDAT[4] = inDAT[2] / 2;
        }

        OCR1AH = inDAT[3];
        OCR1AL = inDAT[4];        

        delay(10);

        TCCR1A = TCCR1A & B00111111;
        TCCR1A = TCCR1A | B10000000;

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
          Serial.write(0xFF);
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
          Serial.write(0xFF);
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

    for(k=0;k<5;k++){
      if(byteDAT%2 == 1){
        digitalWrite(2+k,HIGH);
      }
      else{
        digitalWrite(2+k,LOW);
      }
      byteDAT = byteDAT/2;
    }

    PAT_CNT = PAT_CNT + 1;

    delay(PAT_WD);
  }
  else{
    delay(1000);
  }
}
