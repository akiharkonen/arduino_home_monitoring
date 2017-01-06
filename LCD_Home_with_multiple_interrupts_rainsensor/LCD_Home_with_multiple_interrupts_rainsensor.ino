// Home measurement & monitoring system v.0001 by -Aki-
// Reads sensor data and displays it on a small lcd-screen
// alerts when actions are needed 

#include "U8glib.h"
#include "DHT.h"
#include "PinChangeInt.h" 
#define TRIGGERPIN A3 // setting the motion sensor pin for sensor #1
#define TRIGGERPIN1 A4 // setting the motion sensor pin for sensor #2
#define TRIGGERPIN2 A5 // setting the motion sensor pin for sensor #1

volatile uint16_t interruptCount=0; // The count will go back to 0 after hitting 65535.
volatile unsigned long start, finish, elapsed, over; 
volatile int h, m, s;
String Trigger;

int RainPin = A1; // rain detection module
int RainDigitalIn = 4; // rain detection module 
int RainVal; // rain detection module
boolean IsRaining = false; // rain detection module
String Rainmessagestring; // rain detection module

U8GLIB_ST7920_128X64 u8g(13, 11, 12, U8G_PIN_NONE);
#define DHTPIN 2     // Temperature & humidity sensor #0 Digital pin no (change to your own selection)
#define DHTTYPE DHT11   // DHT 11 other possible sensor types: DHT21 & 22
#define DHTPIN1 3     // Temperature & humidity sensor #1 Digital pin no (change to your own selection) 
#define DHTTYPE DHT11   
#define u8g_logo_width 64 // defining startup logo width
#define u8g_logo_height 64 // startup logo height
// the relative humidity value recommended maximum is 45 % for central heating season (winter)
// and 80 % for cold "non living" spaces. As the sensors are a bit "off" the max values are set a bit higher than
// recommended values. Please test yöur own sensors before setting a max value.

int maxh=85; //set the max humidity for sensor #0 = warn value for sensor in cellar
int maxh1=50; //set the max humidity for sensor #1 = warn value for hall

// next a logo converted to monochromatic hexadecimal byte array
// using http://manytools.org/hacker-tools/image-to-byte-array/go :

static unsigned char u8g_logo_bits[] U8G_PROGMEM = {		
0x00, 0x06, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0xfe, 0x04, 0x00, 0x00,
   0x80, 0x00, 0x20, 0x40, 0x92, 0x04, 0x00, 0x00, 0x80, 0x00, 0x00, 0x40,
   0x10, 0x14, 0xe7, 0xdd, 0x8e, 0x1c, 0xbf, 0x4d, 0x10, 0xa4, 0x48, 0x49,
   0x91, 0xa2, 0x24, 0x45, 0x10, 0xa4, 0x48, 0x08, 0x9f, 0xa2, 0x24, 0x42,
   0x10, 0xa4, 0x48, 0x30, 0x81, 0xa2, 0x24, 0x05, 0x38, 0x6e, 0xe7, 0xb0,
   0xce, 0x1d, 0xf5, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x12, 0x00, 0x04, 0x00,
   0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
   0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x28, 0x20, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x45, 0x85, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x29,
   0x94, 0x0a, 0x00, 0x00, 0x00, 0x00, 0xac, 0x54, 0x11, 0x04, 0x00, 0x00,
   0x00, 0x00, 0xaa, 0xd2, 0xa7, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x55, 0xa9,
   0x10, 0x5d, 0x00, 0x00, 0x00, 0x80, 0xd5, 0xab, 0x54, 0x50, 0x00, 0x00,
   0x00, 0x80, 0xf5, 0xde, 0x9f, 0x52, 0x00, 0x00, 0x00, 0xc0, 0xfd, 0xed,
   0x7b, 0x92, 0x00, 0x00, 0x00, 0xe0, 0xb6, 0xff, 0x5f, 0x69, 0x01, 0x00,
   0x00, 0xe0, 0x6d, 0xfd, 0xbf, 0xea, 0x01, 0x00, 0x00, 0x70, 0xdb, 0x57,
   0x7d, 0xfd, 0x07, 0x00, 0x00, 0xf0, 0xbb, 0x95, 0x52, 0xf8, 0x07, 0x00,
   0x00, 0x70, 0xef, 0xab, 0x4e, 0xfd, 0x0f, 0x00, 0x00, 0xf0, 0x7d, 0x4b,
   0x9b, 0xb0, 0x0e, 0x00, 0x00, 0xf0, 0xfe, 0xfe, 0x67, 0xeb, 0x0a, 0x00,
   0x00, 0xf0, 0xad, 0xfe, 0x4f, 0x82, 0x0f, 0x00, 0x00, 0xf8, 0xb7, 0x7a,
   0x50, 0x09, 0x07, 0x00, 0x00, 0xf0, 0x56, 0x4a, 0x55, 0x53, 0x0f, 0x00,
   0x00, 0xf8, 0x56, 0x01, 0x52, 0xd6, 0x06, 0x00, 0x00, 0xf8, 0xad, 0x04,
   0xd2, 0xfe, 0x05, 0x00, 0x00, 0xd8, 0xf6, 0x03, 0x28, 0xff, 0x00, 0x00,
   0x00, 0xf8, 0x96, 0x4a, 0xa0, 0xff, 0x05, 0x00, 0x00, 0xa2, 0x7b, 0x05,
   0x69, 0xfb, 0x02, 0x00, 0x00, 0xef, 0xff, 0x12, 0x60, 0xef, 0x0a, 0x00,
   0x00, 0xb7, 0x7e, 0x41, 0xd2, 0xff, 0x15, 0x00, 0x00, 0xeb, 0xfd, 0x0a,
   0xa0, 0xff, 0x03, 0x00, 0x80, 0xfc, 0x1b, 0x21, 0xfc, 0xff, 0x17, 0x00,
   0x80, 0xd1, 0x7b, 0x0a, 0xf8, 0x45, 0x0f, 0x00, 0x80, 0xee, 0x97, 0x22,
   0xfe, 0xaa, 0x1e, 0x00, 0x80, 0xdc, 0x5e, 0x0a, 0xbe, 0xfa, 0x1f, 0x00,
   0x80, 0xfa, 0xbb, 0xa4, 0x6f, 0xff, 0x1f, 0x00, 0x80, 0xd2, 0xb7, 0x11,
   0xff, 0xff, 0x3f, 0x00, 0x00, 0xed, 0x5d, 0xc5, 0xef, 0x01, 0x3d, 0x00,
   0x00, 0xdb, 0x7f, 0x95, 0x7f, 0xf5, 0x3d, 0x00, 0x00, 0xd6, 0xf7, 0xca,
   0x0f, 0xf8, 0x3b, 0x00, 0x00, 0xaa, 0xdf, 0x95, 0xae, 0xff, 0x3f, 0x00,
   0x00, 0xfc, 0x7f, 0xeb, 0xaf, 0xfe, 0x3f, 0x00, 0x00, 0xc0, 0x7f, 0x5f,
   0xdf, 0xff, 0x3f, 0x00, 0x00, 0xc0, 0xef, 0xf5, 0xf7, 0xff, 0x3f, 0x00,
   0x00, 0xc0, 0xff, 0xdf, 0xff, 0xff, 0x3f, 0x00, 0x00, 0xc0, 0xff, 0xfe,
   0xff, 0xff, 0x3f, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00,
   0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x80, 0xff, 0xff,
   0xff, 0xff, 0x3f, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x00,
   0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x80, 0xff, 0xff,
   0xff, 0xff, 0x0f, 0x00, 0x00, 0x80, 0xfe, 0xff, 0xff, 0xff, 0x07, 0x00,
   0x00, 0xc0, 0xfb, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0xff, 0xff,
   0xff, 0xff, 0x01, 0x00, 0x00, 0xc0, 0xfa, 0xff, 0xff, 0xff, 0x00, 0x00,
   0x00, 0x00, 0xeb, 0xff, 0xff, 0xff, 0x00, 0x00 };

DHT dht(DHTPIN, DHTTYPE); // Initialize the sensor #0
DHT dht1(DHTPIN1, DHTTYPE); // Initialize the sensor #1

void setup(void) {
u8g.setFont(u8g_font_unifont);
  pinMode(TRIGGERPIN, INPUT_PULLUP);  // Configure the pin as an input, and turn on the pullup resistor.
                                      // See http://arduino.cc/en/Tutorial/DigitalPins
  attachPinChangeInterrupt(TRIGGERPIN, interruptFunction, RISING);
  pinMode(TRIGGERPIN1, INPUT_PULLUP); 
  attachPinChangeInterrupt(TRIGGERPIN1, interruptFunction1, RISING);
  pinMode(TRIGGERPIN2, INPUT_PULLUP); 
  attachPinChangeInterrupt(TRIGGERPIN1, interruptFunction1, RISING);
  pinMode(4,INPUT); //setting the pin 4 to input
}

void draw(void) {

  int ts0=analogRead(0); // read the temp sensor #0 on analog port 0
  int ts1=analogRead(1); // read the temp sensor #1 on analog port 1
  const int WatersensorPin= 2; // Water sensor pin connected to analog port 2
  int Water_level; // Water level measurement value
  pinMode(WatersensorPin, INPUT); //the liquid level sensor will be an input to the arduino
  Water_level= analogRead(WatersensorPin); // read the analog port for the water level
}

void loop(void) {
// u8g.firstPage();  
// do {
//   draw();
// } while( u8g.nextPage() );
// delay(1000);
  u8g.firstPage();   
  do {
   draw_page_0();
  } 
  while( u8g.nextPage() );//end of first picture loop 
  delay(4000); 
 
  u8g.firstPage();   
  do {
   draw_page_1();
  } 
  while( u8g.nextPage() );//end of first picture loop 
  delay(4000);
  u8g.firstPage();   
  do {
   draw_page_2();
  // delay(1000);
  } 
  while( u8g.nextPage() );//end of second picture loop 
 delay(4000); 
  u8g.firstPage();   
  do {
   draw_page_3();
  } 
  while( u8g.nextPage() );//end of third picture loop 
  delay(4000); 
  
  u8g.firstPage();   
  do {
   draw_page_4();
  } 
  while( u8g.nextPage() );//end of fourth picture loop 
 delay(4000); 

  u8g.firstPage();   
  do {
   draw_page_5();
  } 
  while( u8g.nextPage() );//end of fifth picture loop 
  delay(4000); 
}

void draw_page_0(void)
{
 u8g.drawXBMP( 0, 0, u8g_logo_width, u8g_logo_height, u8g_logo_bits);
  u8g.setPrintPos(75, 10);
  u8g.print("Home");
  u8g.setPrintPos(50, 22);
  u8g.print("monitoring"); 
  u8g.setPrintPos(65, 34);
  u8g.print("system");
  u8g.setPrintPos(65, 46);
  u8g.print("v.0001");
  u8g.setPrintPos(60, 62);
  u8g.print("by -Aki-");
}
void draw_page_1(void)
{
  float h = dht.readHumidity(); // read the sensor #0 data
  float t = dht.readTemperature(); // add (=False) if you want fahrenheit
u8g.setPrintPos(0, 10); 
 u8g.print("Sensor in ");  
  u8g.print("cellar");
  u8g.setPrintPos(0, 30); 
  u8g.print("Temp: ");
  u8g.print(t); 
  u8g.setPrintPos(0, 42);
  u8g.print("Humidity: "); 
  u8g.print(h);
  u8g.setPrintPos(0, 59);
  if (h>maxh)
  {
  u8g.print ("Humidity HIGH");
  }
  else
  {
  u8g.print ("Humidity OK");
  }
}

void draw_page_2(void)
{
  float h1 = dht1.readHumidity(); // read the sensor #0 data
  float t1 = dht1.readTemperature(); // add (=False) if you want fahrenheit
u8g.setPrintPos(0, 10); 
 u8g.print("Sensor in ");  
  u8g.print("hall  ");
  u8g.setPrintPos(0, 30); 
  u8g.print("Temp: ");
  u8g.print(t1); 
  u8g.setPrintPos(0, 42);
  u8g.print("Humidity: "); 
  u8g.print(h1);
  u8g.setPrintPos(0, 59);
  if (h1>maxh1)
  {
  u8g.print ("Humidity HIGH");
  }
  else
  {
  u8g.print ("Humidity OK");
  }
}
void draw_page_3(void)
{
  const int WatersensorPin= 2; // Water sensor pin connected to analog port 2
  int Water_level_sensor_data; // Water level measurement value
  float Actual_Water_Level;
  pinMode(WatersensorPin, INPUT); //the liquid level sensor will be an input to the arduino
  Water_level_sensor_data= analogRead(WatersensorPin); // read the analog port for the water level sensor value
  Actual_Water_Level= Water_level_sensor_data;
u8g.setPrintPos(0, 10); 
 u8g.print("Water level");
  u8g.setPrintPos(0, 30); 
  u8g.print("at ");
  u8g.print(Actual_Water_Level);
  u8g.print(" ");
  u8g.setPrintPos(0, 59);
  if (Actual_Water_Level>0)
  {
  u8g.print ("Water level HIGH");
  }
  else
  {
  u8g.print ("Water level OK");
  }
}

void draw_page_4(void)
{
  finish = millis(); 

  if (interruptCount >0)
  {
  elapsed=finish-start;
  s=(elapsed/1000);
  m=(s/60);
  h=(s/3600);
  s=s-m*60;
  m=m-h*60;
  }
  else
  {
    elapsed = 0;
    h=00;
    m=00;
    s=00;
  }  
  u8g.setPrintPos(0, 10);
  u8g.print("Motion detected");
  u8g.setPrintPos(10, 22);
  u8g.print(interruptCount); 
  u8g.print(" -times");
  u8g.setPrintPos(0, 34);
  u8g.print("Last detection ");
  u8g.setPrintPos(10, 46);
  u8g.print (h);
  u8g.print (":");
  u8g.print (m);
  u8g.print (":");
  u8g.print (s);
  u8g.print (" ago in ");
  u8g.setPrintPos(0, 59);
  u8g.print (Trigger);
//  u8g.drawStr( 0, 15, Trigger);
//  u8g.print(analogRead(3));
}

void draw_page_5(void)
{
  RainVal = analogRead(RainPin);
  IsRaining = !(digitalRead(RainDigitalIn));
  
  if(IsRaining){
    Rainmessagestring = "Raining!";
  }
  else{
    Rainmessagestring = "NOT raining.";
  }
  u8g.setPrintPos(0, 10);
  u8g.print ("Front porch:");
  u8g.setPrintPos(10, 22);
  u8g.print (Rainmessagestring); 
  u8g.setPrintPos(10, 34);
  u8g.print ("Temp: ");
  u8g.print (" C");
  u8g.setPrintPos(0, 59);
  u8g.print ("Stadin kuumin...");
}
// pir motion sensor (hc-sr505)
void interruptFunction() {
  interruptCount++;
  start= millis();
  Trigger = "TV-room"; 
}
// pir motion sensor (hc-sr501)
void interruptFunction1() {
  interruptCount++;
  start= millis();
  Trigger = "Kitchen"; 
}
// pir motion sensor (hc-sr501)
void interruptFunction2() {
  interruptCount++;
  start= millis();
  Trigger = "Garage"; 
} 
