/*
 * Randy Powell
 * Capitol Technology University 
 * Senior Design Project
 * Spring 2019
 * ABEONA: A Hiking Environment Tracker and Navigation Device
 */
//Sensor libraries
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <math.h>
#include <AM2320.h>
#include <EEPROM.h>
#define pi 3.14159265358979323846

//----- EERPOM MEMORY LOCATIONS AND SIZES -------
/* VARIABLE       LOCATION    SIZE    TYPE
 * --------       --------    ----    ----
 * pin1_lat       0           4       float
 * pin1_lon       4           4       float
 * pin2_lat       8           4       float
 * pin2_lon       12          4       float
 * trp1_st_time_h 16          1       char
 * trp1_st_time_m 17          1       char
 * trp2_st_time_h 18          1       char
 * trp2_st_time_m 19          1       char
 * trp1_tot_dist  20          4       float
 * trp2_tot_dist  24          4       float
 * trp1_st_alt    28          4       float
 * trp2_st_alt    32          4       float
 * trp_last_lat   36          4       float
 * trp_last_lon   40          4       float
 */


//Setup Temp/Hum Sensor
AM2320 th;

//Setup Display ---------------
//#define OLED_RESET 4
//Adafruit_SSD1306 display(OLED_RESET);
SSD1306AsciiWire oled;
#define OLED_ADDRESS 0x3C
#define OLED_RST -1

//setup GPS -------------------
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//Environment Variables----------------
float latt,lon;
float alt,pres;
float head;
char sats;
char time_h,time_m;
//int time_s;
float temp,humid;
unsigned long prev_mil = 0;
char mode=1;

//Trip data------------
unsigned long trp_prev_mil;
float trp_last_lat,trp_last_lon;
//float trp_last_alt;
//--- Trip 1 ---
float trp1_tot_dist;
float trp1_st_alt;
//float trp1_alt_gain;
char trp1_st_time_h;
char trp1_st_time_m;
//--- Trip 2 ---
float trp2_tot_dist;
float trp2_st_alt;
//float trp2_alt_gain;
char trp2_st_time_h;
char trp2_st_time_m;
//pin data------------------

float pin1_lat,pin1_lon;
float pin2_lat,pin2_lon;

//buttons-------------------
//const char bp1 = 8;
//const char bp2 = 9;
//const char bp3 = 10;
//char bs1 = 0;
//char bs2 = 0;
//char bs3 = 0;

unsigned long b1_prev_mil=0;
unsigned long b2_prev_mil=0;
unsigned long b3_prev_mil=0;

void setup(){
  Serial.begin(9600);
  delay(200);
  ss.begin(GPSBaud);
  


  #if OLED_RST >= 0
  oled.begin(&Adafruit128x64, OLED_ADDRESS, OLED_RST);
  #else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, OLED_ADDRESS);
  #endif
//-------- Set saved values ------------
  EEPROM.get(0, pin1_lat);
  EEPROM.get(4, pin1_lon);  
  EEPROM.get(8, pin2_lat);
  EEPROM.get(12, pin2_lon);
  EEPROM.get(16, trp1_st_time_h);
  EEPROM.get(17, trp1_st_time_m);
  EEPROM.get(18, trp2_st_time_h);
  EEPROM.get(19, trp2_st_time_m);
  EEPROM.get(20, trp1_tot_dist);
  EEPROM.get(24, trp2_tot_dist);
  EEPROM.get(28, trp1_st_alt);
  EEPROM.get(32, trp2_st_alt);
  EEPROM.get(36, trp_last_lat);
  EEPROM.get(40, trp_last_lon);
  

}

void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      
      latt=gps.location.lat();
      lon=gps.location.lng();
      time_h=gps.time.hour()-5;
      if(time_h<0){
        time_h=time_h+24;
        }
      time_m=gps.time.minute();
      //time_s=gps.time.second();
      alt=gps.altitude.feet();
      pres=alt_to_pres(alt);
      head=gps.course.deg();
      sats=gps.satellites.value();
    }
  }
  unsigned long cur_mil = millis();
  //Set up delay based on timestamp for these actions (only execute every 2 seconds)---------
  if(cur_mil-prev_mil>2000){
    prev_mil=cur_mil;
    //read temp and humidity  
    switch(th.Read()){
      case 2:
      case 1:
        temp=-99;
        humid=-99;
        break;
      case 0:
        temp=th.t;
        humid=th.h;
        break;
    }
    if(mode==2){
      mode_2();
    }else if(mode==3){
      mode_3();
    }else{
      mode_1();
    }
    //delay(15);
  }
  //---------- Handle Trip data every 60 seconds -----------
  if(cur_mil-trp_prev_mil>60000){
    //---Trip 1 ---
    //Find distance between current lat/lon and last trp lat/lon
    float tmp_dist = dist_between(latt,lon,trp_last_lat,trp_last_lon);
    //  Add it to total trip distance for both
    trp1_tot_dist = trp1_tot_dist + tmp_dist;
    trp2_tot_dist = trp2_tot_dist + tmp_dist;
  
    //  Set trp last lat/lon/alt to current
    trp_last_lat = latt;
    trp_last_lon = lon;

    
    EEPROM.put(20,trp1_tot_dist);
    EEPROM.put(24,trp2_tot_dist);
    EEPROM.put(36,trp_last_lat);
    EEPROM.put(40,trp_last_lon);
    }
  //---------- Handle button pressing -------------------


  // button 1 handler (pin 5)
  if(digitalRead(5) == HIGH){
    if(cur_mil-b1_prev_mil>2000){
      
        if(mode==2){
          //Set pin 1 location
          pin1_lat=latt;
          pin1_lon=lon;
          EEPROM.put(0,pin1_lat);
          EEPROM.put(4,pin1_lon);
          
        }else if(mode==3){
          //Start trip 1
          trp1_tot_dist=0;
//          trp1_alt_gain=0;
          trp1_st_time_h=time_h;
          trp1_st_time_m=time_m;
          trp1_st_alt=alt;

          EEPROM.put(16,trp1_st_time_h);
          EEPROM.put(17,trp1_st_time_m);
          EEPROM.put(28,trp1_st_alt);
          
        }
        
      b1_prev_mil=cur_mil;
    }
  }
  
  // button 2 handler (pin 6)
  if(digitalRead(6) == HIGH){
    if(cur_mil-b2_prev_mil>2000){
        if(mode==2){
          //Set pin 2 location
          pin2_lat=latt;
          pin2_lon=lon;
          EEPROM.put(8,pin2_lat);
          EEPROM.put(12,pin2_lon);
          
        }else{
          //Start trip 2
          trp2_tot_dist=0;
//          trp2_alt_gain=0;
          trp2_st_time_h=time_h;
          trp2_st_time_m=time_m;
          trp2_st_alt=alt;
          EEPROM.put(18,trp2_st_time_h);
          EEPROM.put(19,trp2_st_time_m);
          EEPROM.put(32,trp2_st_alt);
          
        }
      b2_prev_mil=cur_mil;
    }
  }
  
  // button 3 handler (pin 7)
  if(digitalRead(7) == HIGH){
    if(cur_mil-b3_prev_mil>2000){
        if(mode==1){
          mode=2;
        }else if(mode==2){
          mode=3;
        }else{
          mode=1;
        }
      b3_prev_mil=cur_mil;
    }
  }


  //-------------------------------------------------------  
}

//--------- DISPLAY MODE FUNCTIONS ------------------
//MODE 1: BASIC DATA PRINT
//  Print basic environment data metrics to the screen.
//  Button options:
//    -- N/A: No Function
//    -- N/A: No Function
//    -- MOD: Change Mode
void mode_1(){
  oled.clear();   // clears the screen and buffer

  oled.setFont(Adafruit5x7);
  oled.setCursor(0, 0);

    
    //display.print(F(":"));
    //display.println(time_s);
    oled.print(F("alt:"));
    oled.print(alt,1);
    oled.println(F("ft"));
    oled.print(F("prs:"));
    oled.print(pres);
    oled.println(F("bar"));
    oled.print(F("tmp:"));
    oled.print(temp);
    oled.println(F("C"));
    oled.print(F("hum:"));
    oled.print(humid);
    oled.println(F("%"));
    
    oled.print(F("hdg:"));
    oled.println(head);
    
    oled.print(latt,5);
    oled.print(F(","));
    oled.println(lon,5);
    oled.print(F("sats: "));
    oled.println((short)sats);
    oled.print(F("N/A--N/A--MOD--"));
    oled.print((short)time_h);
    oled.print(F(":"));
    oled.println((short)time_m);
//  oled.display();
  return;
  }
//MODE 2: COMPASS PINFINDING
//  Print distance and direction to two set pins.
//  Button options:
//    -- SP1: Set the location of pin 1
//    -- SP2: Set the location of pin 2
//    -- MOD: Change Mode
void mode_2(){
  oled.clear();   // clears the screen and buffer

  oled.setFont(Adafruit5x7);
  
//  oled.setCursor(0, 0);
  oled.print(F("Head: "));
  oled.println(head,4);
  
//  oled.setCursor(20, 8);
  oled.println(F("   PIN 1    PIN 2"));
  
  //Heading to pin 1, pin 2
//  oled.setCursor(0, 16);
  oled.print(F("H:"));
  oled.print(bearing(latt,lon,pin1_lat,pin1_lon),4);
  oled.print(F(" "));
//  oled.setCursor(72, 16);
  oled.println(bearing(latt,lon,pin2_lat,pin2_lon),4);
  
  //Distance to pin 1, pin 2
//    oled.setCursor(0, 24);
  oled.print(F("D:"));
  oled.print(dist_between(latt,lon,pin1_lat,pin1_lon),4);
  oled.print(F(" "));
//  oled.setCursor(72, 24);
  oled.println(dist_between(latt,lon,pin2_lat,pin2_lon),4);
  
  //Latitude of pin 1, pin 2
//    oled.setCursor(0, 32);
  oled.print(F("LT:"));
  oled.print(pin1_lat,3);
  oled.print(F(" "));
//  oled.setCursor(72, 32);
  oled.println(pin2_lat,3);
  
  //Longitude of pin 1, pin 2
//  oled.setCursor(0, 40);
  oled.print(F("LN:"));
  oled.print(pin1_lon,3);
  oled.print(F(" "));
//  oled.setCursor(72, 40);
  oled.println(pin2_lon,3);
  oled.println();
//  oled.setCursor(0, 56);
  oled.print(F("SP1--SP2--MOD--"));
  oled.print((short)time_h);
  oled.print(F(":"));
  oled.println((short)time_m);
  
//  oled.display();

  return;
  }
//MODE 3: TRIP DATA
//  Print trip information for two different trips (day and trip)
//  Button options:
//    -- TP1: Reset and start tracking trip 1
//    -- TP2: Reset and start tracking trip 2
//    -- MOD: Change Mode
void mode_3(){
//  ct=time_h+time_m/60
//  st=trp1_st_time_h+trp1_st_time_m/60;
float dt1=((time_h+time_m/60.0)-(trp1_st_time_h+trp1_st_time_m/60.0));
if(dt1<0){dt1=dt1+24;}
float dt2=((time_h+time_m/60.0)-(trp2_st_time_h+trp2_st_time_m/60.0));
if(dt2<0){dt2=dt2+24;}
  oled.clear();   // clears the screen and buffer

  oled.setFont(Adafruit5x7);
  

  oled.println(F("   TRIP 1   TRIP 2"));

  oled.print(F("TD:"));
  oled.print(trp1_tot_dist,3);
  oled.print(F("mi "));
  
//  oled.setCursor(72, 1);
  oled.print(trp2_tot_dist,3);
  oled.println(F("mi"));

//  oled.setCursor(0, 16);
  oled.print(F("TI:"));
  oled.print(dt1,2);
  oled.print(F("h "));
  
//  oled.setCursor(72, 16);
  oled.print(dt2,2);
  oled.println(F("h"));

//    oled.setCursor(0, 24);
  oled.print(F("SP:"));
  oled.print(trp1_tot_dist/dt1,2);
  oled.print(F("mph "));
  
//  oled.setCursor(72, 24);
  oled.print(trp2_tot_dist/dt2,2);
  oled.println(F("mph"));

//    oled.setCursor(0, 32);
  oled.print(F("AN:"));
  oled.print(alt-trp1_st_alt,0);
  oled.print(F("ft "));
  
//  oled.setCursor(72, 32);
  oled.print(alt-trp2_st_alt,0);
  oled.println(F("ft"));
  oled.println();
  oled.println();
    
//  oled.setCursor(0, 56);
  oled.print(F("ST1--ST2--MOD--"));
  oled.print((short)time_h);
  oled.print(F(":"));
  oled.println((short)time_m);
  
//  oled.display();

  return;
  }

// -------------------- HELPER FUNCTIONS ---------------------  
//calculate pressure in bars from altitude
float alt_to_pres(float alt){
    return 1.01325*pow((1-(2.25577*pow(10,-5)*alt*3.28084)),5.25588);
}

float dist_between(float lat1,float lon1,float lat2, float lon2){
  float a,c,d,d_o,d_y;
  float R=6371000;//earth radius in meters
  if ((lat1 == lat2) && (lon1 == lon2)) {
    return 0;
  }
  else {
//    o1=deg2rad(lat1);
//    o2=deg2rad(lat2);
    d_o=deg2rad(lat2-lat1);
    d_y=deg2rad(lon2-lon1);
    a=sin(d_o/2)*sin(d_o/2)+cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(d_y/2)*sin(d_y/2);
    c=2*atan2(sqrt(a),sqrt(1-a));
    d=R*c/1609.344;
    return d;
  }
}

float bearing(float lat1,float lon1,float lat2, float lon2){
  float y,x,brng;
  y=sin(deg2rad(lon2)-deg2rad(lon1));
  x=cos(deg2rad(lat1))*sin(deg2rad(lat2))-sin(deg2rad(lat1))*cos(deg2rad(lat2))*cos(deg2rad(lon2)-deg2rad(lon1));
  brng=rad2deg(atan2(y,x));
  return brng;
}

float rad2deg(float rad) {
  
  return (rad * 180 / pi);
}

float deg2rad(float deg) {
  return (deg * pi / 180);
}
