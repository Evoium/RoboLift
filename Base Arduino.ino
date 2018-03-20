
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


int           n, i, cx = tft.width()  / 2 - 50, cy = tft.height() / 2 - 50;


char junk;
String inputString="";
int MotENA = 32;
int MotENB = 22;
int Mot1Left = 24;
int Mot1Right = 26;
int Mot2Left = 28;
int Mot2Right = 30;

int Speed = 60;

void setup()                    // run once, when the sketch starts
{
     Serial.begin(9600);
    uint32_t when = millis();
    //    while (!Serial) ;   //hangs a Leonardo until you connect a Serial
    if (!Serial) delay(5000);           //allow some time for Leonardo
    Serial.println("Serial took " + String((millis() - when)) + "ms to start");
    //    tft.reset();                 //hardware reset
    uint16_t ID = tft.readID(); //
    Serial.print("ID = 0x");
    Serial.println(ID, HEX);
    if (ID == 0xD3D3) ID = 0x9481; // write-only shield
//    ID = 0x9329;                             // force ID
    tft.begin(ID);
    tft.fillScreen(BLACK);


    
pinMode (MotENA, OUTPUT);
pinMode (MotENB, OUTPUT);
pinMode (Mot1Left, OUTPUT);
pinMode (Mot1Right, OUTPUT);
pinMode (Mot2Left, OUTPUT);
pinMode (Mot2Right, OUTPUT);
 
digitalWrite(MotENA, LOW);
digitalWrite(MotENA, LOW);
}

void loop()

{

  
  if(Serial.available()){
  while(Serial.available())
    {
      char inChar = (char)Serial.read(); //read the input
      inputString += inChar;        //make a string of the characters coming on serial
    }
    Serial.println(inputString);
    while (Serial.available() > 0)  
    { junk = Serial.read() ; }      // clear the serial buffer

    
    if(inputString == "w"){         //in case of 'a' turn the LED on
       
      digitalWrite(MotENA, HIGH);
      digitalWrite(MotENB, HIGH);
      digitalWrite(Mot1Right, HIGH);
      digitalWrite(Mot1Left, LOW);
      digitalWrite(Mot2Right, HIGH);
      digitalWrite(Mot2Left, LOW);

      tft.fillScreen(BLACK);
      tft.setRotation(2);
      n     = min(cx, cy);
      for (i = 0; i < n; i += 5) {
              tft.drawTriangle(
              cx - i+50    , cy+50, // peak
              cx + i+50, cy - i+50, // bottom left
              cx + i+50, cy + i+50, // bottom right
              tft.color565(0, 0, 255)); 
      inputString = "";
      digitalWrite(MotENA, LOW);
      digitalWrite(MotENB, LOW);
        }
    }
      
    }else if(inputString == "s"){   //incase of 'b' turn the LED off
      digitalWrite(MotENA, HIGH);
      digitalWrite(MotENB, HIGH);
      digitalWrite(Mot1Right, LOW);
      digitalWrite(Mot1Left, HIGH);
      digitalWrite(Mot2Right, LOW);
      digitalWrite(Mot2Left, HIGH);

      tft.fillScreen(BLACK);
      tft.setRotation(4);
      n     = min(cx, cy);
      for (i = 0; i < n; i += 5) {
              tft.drawTriangle(
              cx - i+50    , cy+50, // peak
              cx + i+50, cy - i+50, // bottom left
              cx + i+50, cy + i+50, // bottom right
              tft.color565(0, 0, 255)); 
      inputString = "";
      digitalWrite(MotENA, LOW);
      digitalWrite(MotENB, LOW);
        }
        }else if(inputString == "a"){   //incase of 'b' turn the LED off
      
      digitalWrite(MotENA, HIGH);
      digitalWrite(MotENB, LOW);
      digitalWrite(Mot1Right, HIGH);
      digitalWrite(Mot1Left, LOW);
      digitalWrite(Mot2Right, HIGH);
      digitalWrite(Mot2Left, LOW);

      tft.fillScreen(BLACK);

      tft.setRotation(4);
      n     = min(cx, cy);
      for (i = 0; i < n; i += 5) {
            tft.drawTriangle(
            cy  -20 , cx - i, // peak
            cy - i-20, cx + i, // bottom left
            cy + i-20, cx + i, // bottom right
            tft.color565(0, 0, 255));

       inputString = "";
      digitalWrite(MotENA, LOW);
      digitalWrite(MotENB, LOW);
  }
        }else if(inputString == "d"){   //incase of 'b' turn the LED off
          
      digitalWrite(MotENA, LOW);
      digitalWrite(MotENB, HIGH);
      digitalWrite(Mot1Right, HIGH);
      digitalWrite(Mot1Left, LOW);
      digitalWrite(Mot2Right, HIGH);
      digitalWrite(Mot2Left, LOW);

      tft.fillScreen(BLACK);
      tft.setRotation(2);
      n     = min(cx, cy);
      for (i = 0; i < n; i += 5) {
            tft.drawTriangle(
            cy    +40, cx - i, // peak
            cy - i+40, cx + i, // bottom left
            cy + i+40, cx + i, // bottom right
            tft.color565(0, 0, 255));
      inputString = "";
      digitalWrite(MotENA, LOW);
      digitalWrite(MotENB, LOW);
       




      }

      
    }
    else{
    inputString = "";
    }

      digitalWrite(MotENA, LOW);
      digitalWrite(MotENB, LOW);
  }
  


