#include <Wire.h>
#include <SPI.h>

#include <Adafruit_ADS1X15.h>   
#include <Adafruit_GFX.h>     
#include <Adafruit_Si7021.h>
#include <Adafruit_SSD1306.h>
#include <afstandssensor.h>


float t = 0, rh = 0, d = 0, mos1 = 0, mos2 = 0, mic = 0, ac = 0, prESP = 0, mos1ESP = 0, mos2ESP = 0, acESP = 0, micESP = 0;
const uint8_t IO_PR = 15, IO_AC = 33, IO_MIC = 14, IO_MOS1 = 16, IO_MOS2 = 7, IO_BUZZER = 25, IO_LED = 32, CHANNEL_BUZZER = 1, CHANNEL_LED = 2; 
unsigned long runtime = 0;
uint16_t interval = 1000;
float tscore=0, hscore=0, lscore=0, tbonus=0, hbonus=0, lbonus=0, l=0, Fscore=0;
String level;
Adafruit_SSD1306 display(128, 64, &Wire, -1);   //128x64 OLED Display - Using default I2C - No reset pin (-1)
Adafruit_ADS1015 ads;                           //4-Channel Analog to Digital Converter 10-bit resolution
AfstandsSensor hcsr04(13, 27);                  //HC-SR04 Ultrasonic Distance Sensor - Trigger pin connected to IO13 - Echo pin connected to IO27 
Adafruit_Si7021 si7021 = Adafruit_Si7021();     //Temperature & Humidity Sensor


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);                         //115200 is the baud rate. When reading from serial monitor, use the same baud rate
  pinMode(IO_AC, INPUT);                        //Defining all the IOs as either input or output
  pinMode(IO_PR, INPUT);
  pinMode(IO_MOS1, INPUT);
  pinMode(IO_MOS2, INPUT);
  pinMode(IO_MIC, INPUT);
  pinMode(IO_LED, OUTPUT);
  pinMode(IO_BUZZER, OUTPUT);
  ledcSetup(CHANNEL_BUZZER, 3200, 8);           //ledcSetup(Channel(0-15), Frequency(Hz), Resolution) - 8 being 8-bit (0-255)
  ledcSetup(CHANNEL_LED, 3200, 8);
  ledcAttachPin(IO_BUZZER, CHANNEL_BUZZER);     //ledcAttachPin(IO, Channel) - Connect channel from ledcSetup to IO
  ledcAttachPin(IO_LED, CHANNEL_LED);
  ledcWrite(CHANNEL_BUZZER, 0);                 //ledcWrite - Set duty cycle of channel to 0 - (0/255 = 0%)
  ledcWrite(CHANNEL_LED, 0);


  ledcWrite(CHANNEL_BUZZER, 128);               //ledcWrite - Set duty cycle of channel to 128 - (128/255 = 50%)
  delay(500);                                   //delay(ms)
  ledcWrite(CHANNEL_BUZZER, 0);
  ledcWrite(CHANNEL_LED, 128);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);    //Initialize OLED - Use internal power source - I2C address = 0x3C
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  si7021.begin();                               //Initialize temperature & humidity sensor
  ads.begin();                                  //Initialize ADC
}

void loop() {
  // put your main code here, to run repeatedly:
  while(millis() - runtime > interval){         //millis() returns milliseconds since start of program - interval is 1000ms - "if" would work the same way here
    runtime = millis();
    getReadings();
    printReadings();
    updateScreen();
  }
}

void updateScreen(){
    display.clearDisplay();                                       //Clears the display
    display.setCursor(0,0);                                       //Move to 0,0 on display (Top Left)
    display.print("T:"); display.print(t,1);                      //print("...") - prints out the characters between "" on the display
    display.print(" RH:"); display.print(rh,0);                   //print(variable, 1) - prints out the variable with 1 decimal
    display.print(" L:"); display.println(prESP, 1);                  //println - prints and then moves to the next line
    display.print("TS:"); display.print(tscore*100, 0);
    display.print(" HS:"); display.print(hscore*100, 0);
    display.print(" LS:"); display.println(lscore*100, 0);
    display.println();
    display.print("Final score:"); display.print(Fscore, 0);
    display.println();
    display.print(" Class: "); display.println(level);
    display.display();                                            //Show everything on the screen
  }

void getReadings(){
  t = si7021.readTemperature();                                   //Request update to temperature register and read said register from the SI7021 IC
  rh = si7021.readHumidity();                                     //Request update to humidity register and read said register from the SI7021 IC
  d = hcsr04.afstandCM(); 
 d=100;//Start pulse and measure how long it takes to return - Convert duration into distance - Use hcsr04.afstandCM(temperature) to adjust for temperature
  mos1 = ads.readADC_SingleEnded(0)*3.0;                          //Read from ADC channel 0 - Multiply by 3.0 to convert from ADC to mV - See singleended.ino example
  mos2 = ads.readADC_SingleEnded(1)*3.0;
  mic = ads.readADC_SingleEnded(2)*3.0;
  ac = ads.readADC_SingleEnded(3)*3.0;
  prESP = analogRead(IO_PR)/4095.0*3300.0;                        //Read from ESP32s ADC (12-bit (0-4095) - 3.3V max) - convert to mV
  acESP = analogRead(IO_AC)/4095.0*3300.0;
  prESP=1.392*pow(10,-6)*pow(prESP,3)-0.002228*pow(prESP,2)+2*prESP; 
  micESP = analogRead(IO_MIC)/4095.0*3300.0;
  mos1ESP = analogRead(IO_MOS1)/4095.0*3300.0;
  mos2ESP = analogRead(IO_MOS2)/4095.0*3300.0;
    //basic score for temperature
  if((t<=0)&&(t>40)){
    tscore=0;
  }else if((t>0)&&(t<=20)){
    tscore=0.05*t;
  }else if((t>20)&&(t<=40)){
    tscore=-0.05*t+2;
  }

  //basic score for humidity
   if((rh>=0)&&(rh<35)){
    hscore=rh/35;
  }else if((rh>=35)&&(rh<68)){
    hscore=1;
  }else if((rh>=68)&&(rh<=101)){
    hscore=-0.03125*rh+3.125;
  } 

  //basic score for light intensity
  l=prESP;
  if(l<=100){
    lscore=0;
  }else if((l>100)&&(l<=700)){
    lscore=-24.98*pow(l,-0.6144)+1.461-0.01467;
  }else if(l>700){
    lscore=1;
  }

  //bonus score
  if(tscore!=0){
    tbonus=pow(tscore,-0.66);
  }
  if(hscore!=0){
  hbonus=pow(hscore,-0.66);
  }
  if(lscore!=0){
  lbonus=pow(lscore,-0.66);
  }

  //Final score
  Fscore=(tscore*hscore*lscore*tbonus*hbonus*lbonus)*100;
  if((Fscore>=0) && (Fscore<=20)){
    level="Poor";
  }else if((Fscore>20) && (Fscore<=40)){
    level="So So";
  }else if((Fscore>40) && (Fscore<=60)){
    level="Medium";
  }else if((Fscore>60) && (Fscore<=80)){
    level="Not bad";
  }else if((Fscore>80) && (Fscore<=100)){
    level="Good";
  }
}
void printReadings(){
  Serial.print("Final score: "); Serial.print(Fscore);                        //Read serial prints from serial monitor - Use same baud rate as above
  Serial.print("\tMOS1: "); Serial.print(mos1);
  Serial.print("\tMOS2: "); Serial.print(mos2);
  Serial.print("\tMIC: "); Serial.print(mic);
  Serial.print("\tAC: "); Serial.print(ac);
  Serial.print("\tT: "); Serial.print(t);
  Serial.print("\tRH: "); Serial.println(rh);
  Serial.print("ESP ADC:\tMOS1: "); Serial.print(mos1ESP);
  Serial.print("\tMOS2: "); Serial.print(mos2ESP);
  Serial.print("\tMIC: "); Serial.print(micESP);
  Serial.print("\tAC: "); Serial.print(acESP);
  Serial.print("\tPR: "); Serial.println(prESP);
  Serial.println();
}
