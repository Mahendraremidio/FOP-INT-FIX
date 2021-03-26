  //const int W_LED = 9;

const int Reset_LCD = 10;

const int TRI = 11;
const int C1 = 12;//camera_out_1  11  13 12
const int C2 = 13;//camera_out_2  12  12 13
const int C3 = 14;//camera_out_3   13  11 11

int C_S1 = 0;
int C_S2 = 0;
int C_S3 = 0;
int TRI_S = 0;

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



void setup() {
  //  TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz

  pinMode(C1, INPUT);
  pinMode(C2, INPUT);
  pinMode(C3, INPUT);
  pinMode(TRI, INPUT);
  
 if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) // Address 0x3D for 128x64
 { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Clear the buffer
  display.clearDisplay();

}

void loop() 
{
  C_S1 = digitalRead(C1);
  C_S2 = digitalRead(C2);
  C_S3 = digitalRead(C3);
  //TRI_S = digitalRead(TRI);

   
   
 
 if ((C_S1 == LOW)&&(C_S2 == LOW)&&(C_S3 == LOW)) {display.drawPixel(39, 8, WHITE); display.display();delay(500);display.drawPixel(39, 8, BLACK); display.display();delay(400); }
  else{ display.clearDisplay();} //0 
  
 if ((C_S1 == LOW)&&(C_S2 == LOW)&&(C_S3 == HIGH)) {display.drawPixel(89, 8, WHITE); display.display();delay(500);display.drawPixel(89, 8, BLACK); display.display();delay(400); }
 else{   display.clearDisplay();} //1
 
 if ((C_S1 == LOW)&&(C_S2 == HIGH)&&(C_S3 == LOW)) { display.drawPixel(28, 32, WHITE); display.display();delay(500);display.drawPixel(28, 32, BLACK); display.display();delay(400);}
 else{ display.clearDisplay();} //2, 36
 
 if ((C_S1 == LOW)&&(C_S2 == HIGH)&&(C_S3 == HIGH)) {display.drawPixel(48, 32, WHITE); display.display();delay(500);display.drawPixel(48, 32, BLACK); display.display();delay(400);}
 else{ display.clearDisplay();}//3
 
 if ((C_S1 == HIGH)&&(C_S2 == LOW)&&(C_S3 == LOW)) {display.drawPixel(80, 32, WHITE); display.display();delay(500);display.drawPixel(80, 32, BLACK); display.display();delay(400);}
 else{ display.clearDisplay();}//4
 
 if ((C_S1 == HIGH)&&(C_S2 == LOW)&&(C_S3 == HIGH)) {display.drawPixel(100, 32, WHITE); display.display();delay(500);display.drawPixel(100, 32, BLACK); display.display();delay(400);}
 else{ display.clearDisplay();}//5, 92
 
 if ((C_S1 == HIGH)&&(C_S2 == HIGH)&&(C_S3 == LOW)) {display.drawPixel(39, 56, WHITE); display.display();delay(500);display.drawPixel(39, 56, BLACK); display.display();delay(400); }
 else{ display.clearDisplay();}//6

  if ((C_S1 == HIGH)&&(C_S2 == HIGH)&&(C_S3 == HIGH)) {display.drawPixel(89, 56, WHITE); display.display();delay(500);display.drawPixel(89, 56, BLACK); display.display();delay(400); }
 else{ display.clearDisplay();}//7
 
 }


 
