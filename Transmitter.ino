#include <SPI.h> 
#include <RH_RF95.h>
#include<Wire.h>

#define RFM95_CS 10 //NSS
#define RFM95_RST 9
#define RFM95_INT 2 

#define RF95_FREQ 434.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

char value;
const int MPU_addr1 = 0x68;
float xa, ya, za, roll, pitch;
int tu;
int td;

void setup() {
  Serial.begin(9600);
  
  pinMode(30,OUTPUT);
  pinMode(31,OUTPUT);
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  digitalWrite(30,HIGH);
  digitalWrite(31,HIGH);
  
  
  Wire.begin();                                      //begin the wire communication
  Wire.beginTransmission(MPU_addr1);                 //begin, send the slave adress (in this case 68)
  Wire.write(0x6B);                                  //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);
  
  pinMode(RFM95_RST, OUTPUT); 
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

;  rf95.setTxPower(18);  

}

void loop() {
  delay(200);
  tu=0;
  td=0;
  tu=digitalRead(3);
  td=digitalRead(4);
  Serial.println("hi");
  Serial.println(tu);
  Serial.println(td);
  float Roll_value = Roll_Detection();
  Serial.println(Roll_value);
  delay(10);
//  float Roll_value=0;
  float Pitch_value = Pitch_Detection();
  Serial.println(Pitch_value);
  delay(10);
    Serial.print("Send: ");
  value=0;
  if(tu==1)
  {
   // Serial.println("sent 1");
    value=1;
    Serial.println(value);
      char radiopacket[1] ={char(value)};
  rf95.send((uint8_t *)radiopacket, 1);
  }
 else if(td==1)
  {
    value=3;
    Serial.println(value);
      char radiopacket[1] ={char(value)};
  rf95.send((uint8_t *)radiopacket, 1);
  }
  else if(Pitch_value>=3.00)
  {
    value=2;
    Serial.println(value);
      char radiopacket[1] ={char(value)};
  rf95.send((uint8_t *)radiopacket, 1);
  }
  else if(Pitch_value<=-3.00)
  {
    value=8;
    Serial.println(value);
      char radiopacket[1] ={char(value)};
  rf95.send((uint8_t *)radiopacket, 1);
  }
  else if(Roll_value>=1.00)
  {
    value=4;
    Serial.println(value);
      char radiopacket[1] ={char(value)};
  rf95.send((uint8_t *)radiopacket, 1);
  }
  else if(Roll_value<=-1.00)
  {
    value=6;
    Serial.println(value);
      char radiopacket[1] ={char(value)};
  rf95.send((uint8_t *)radiopacket, 1);
  }
}

float Roll_Detection()
{
  
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr1, 6, true);
  xa = Wire.read() << 8 | Wire.read();
  ya = Wire.read() << 8 | Wire.read();
  za = Wire.read() << 8 | Wire.read();

  roll = atan2(ya , za) * 180.0 / PI;
  //Serial.println("###############################################");
 // Serial.print("roll = ");
 //Serial.println(roll);
  float ROLL=map(roll,0,55,0,2);
 // Serial.println(ROLL);
  return(ROLL);
}

float Pitch_Detection()
{
  
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr1, 6, true);
  xa = Wire.read() << 8 | Wire.read();
  ya = Wire.read() << 8 | Wire.read();
  za = Wire.read() << 8 | Wire.read();
//Serial.println(xa);
 // roll = atan2(ya , za) * 180.0 / PI;
  pitch = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI;
  // Serial.print(", pitch = ");
  //Serial.println("pitch");
  float PITCH=map(pitch,0,55,0,3);
 //Serial.println(PITCH);
 // Serial.println("###############################################");
 // float PITCH=map(pitch,*/
  return(PITCH);
}
