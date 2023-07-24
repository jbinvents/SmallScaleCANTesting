#include <Arduino.h>
#include <SPI.h>          //Library for using SPI Communication 
#include <mcp2515.h>      //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)
 
//#define DHTPIN 8
//#define DHTTYPE DHT11
 
struct can_frame canMsg;
MCP2515 mcp2515();
//MCP2515 mcp = MCP2515();

//DHT dht(DHTPIN, DHTTYPE);     //initilize object dht for class DHT with DHT pin with STM32 and DHT type as DHT11

int Pot = 14;
int PotVal; 
 
void setup()
{
  
  
  while (!Serial);
  Serial.begin(9600);
  Serial.println("setup very begining");
  pinMode(13, OUTPUT);
  pinMode(Pot, INPUT);
   
  SPI.begin();               //Begins SPI communication
  
  //dht.begin();               //Begins to read temperature & humidity sesnor value
  Serial.println("setup");
  delay(500);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();
 

}
 
 
void loop()
{

digitalWrite(13, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                      // wait for a second
  digitalWrite(13, LOW);   // turn the LED off by making the voltage LOW
  delay(500);                      // wait for a second

PotVal = analogRead(Pot);
  Serial.println(PotVal);
  delay(50);

  //int P = dht.readHumidity();       //Gets Humidity value
  //int t = dht.readTemperature();    //Gets Temperature value
 /*
  canMsg.can_id  = 0x036;           //CAN id as 0x036
  canMsg.can_dlc = 8;               //CAN data length as 8
  canMsg.data[0] = PotVal;               //Update humidity value in [0]
  canMsg.data[1] = 0x00;               //Update temperature value in [1]
  canMsg.data[2] = 0x00;            //Rest all with 0
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
 
  mcp2515.sendMessage(&canMsg);     //Sends the CAN message
  delay(500);
  */
}