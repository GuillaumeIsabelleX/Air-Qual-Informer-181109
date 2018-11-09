//---------------------------------
//--------- JGHumIDer ---------
//----JGOptAIR__Unified_v1_180209
//---------- A Quick merge of Smoke/Humider and day to test.... (before the more complex merge)
/* Optimal Home Automation and Fun / OptAir
 *
 *  Optimal Humidity : Start Humidifier / Dryer as required
 *  Optimal Temperature : Start Cooler / Heater as required
 *  Optimal Air Quality : Extract/Input Fresh/Bad air as required
 *  Optimal Light Condition : Turn on Light when Night occures outside
 *  Prefered Behaviour for system when Night / Day
 *  Open/Close Windows
 *
 *  Other Related
 *    Use Sun Power to power up the system
 *

*/
/*
Next Works...
  * Tested the Default vent time
  * Added ways to Calibrate
    * Setup/Config Mode
      * Boot+Hold button = Setup Config Mode
      * CONF:
        * Ventilate Trigger Air Quality (ex.  120)
        * Ventilate Interval
        * ...
  *
  * Receive Venting SIgnal from an Ethernet Call
    * Mastered the Ethernet Libraries
    * -> Publish web service with values
  *

*/

String app_name = "OptAIR v2";
String app_info = "by JG 180304";
int app_delay = 1200; //delay show app start info


//CONF Air QUAL
int min_air_qual = 140; //over that quick vent

//TODO Add Potentiometer to configure min_air_qual... save config
int min_air_qual_conf = 5; //ext for config using Potentiometer
int max_air_qual_conf = 200; //ext for config using Potentiometer


int min_vent_time_seconds = 60; //number of seconds minimal time a ventilation last
int min_vent_time_seconds_count = 0; //used to count venting time to stop it when the min time is reached

//----------CONF----TEMP NIght & Day ----------------
int temperature_min_confort_stop_vent = 17; //Temperature where we stop venting (might want to set higher)

int max_temp_day = 28; //starts cooling
int max_heating_temp_day = 23; //stops heating
int min_heating_temp_day = 19; //starts heating

int max_temp_night = 25; //starts cooling
int max_heating_temp_night = 19; //stops heating
int min_heating_temp_night = 19; //starts heating

int max_temp = max_temp_day; //starts cooling
int max_heating_temp = max_heating_temp_day; //stops heating
int min_heating_temp = min_heating_temp_day; //starts heating


int humidity_min = 55; //bellow, it starts humidifying
int humidity_max = 70; //above, it stops humidifying
int humidity_min_dry = 90; //above, it starts drying


int lite_day_value = 180;
int lite_night_value = 250;




//Based On :

//---------------------------------------
// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// Depends on the following Arduino libraries:
// - Adafruit Unified Sensor Library: https://github.com/adafruit/Adafruit_Sensor
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>


#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd(0x27, 2,1,0,4,5,6,7,3, POSITIVE);  // Set the LCD I2C address
LiquidCrystal_I2C lcd(0x3F, 2,1,0,4,5,6,7,3, POSITIVE);  // Set the LCD I2C address

//SECOND SCREEN I BOUTH 
//LiquidCrystal_I2C lcd(0x3F, 2,1,0,4,5,6,7,3, POSITIVE);  // Set the LCD I2C address



#define DHTPIN            2         // Pin which is connected to the DHT sensor.

// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);



uint32_t delayMS;

int haderror = 0;


int airQualA0 = A0;

int liteA2 = A2;


int RELAY_HUMIDIFY = 8;
int RELAY_HEATING = 7;
int RELAY_VENTING = 12;
int RELAY_NIGHT_LIGHT = 13;
int IND_NIGHT_LIGHT = 6;

int RELAY_DRYING = 60;

int PRESENCE_PIN = 9;

//TODO Extend with Night Light
int RELAY_COOLING = 14;
int IND_COOLING = 41;

int IND_HEATING = 1;
int IND_DRYING = 40;

int IND_HUMIDIFY = 55;


//------------------------------------------------------------SimpleVent integrating...
int IND_POOR_AIR = 3;
int IND_VENTING = 15;
int IND_INTERVAL_VENTING = 5;
int count_vent_interval = 0;

int vent_amount_time_count = 0; //count when starting to vent for a certain amount of time


int vent_interval_wait = 45 ; //min to wait for normal venting
int vent_amount_time_sec = 50 ; //vent amount of time for normal
long interval_vent_amount_time = 300000; //ms for interval vent time

int poor_air_quick_vent_time = 15 ; //vent time in second when poor






int IND_PRESENCE = 4;


int RESET_STAT_BUTTON = 10;

void setup() {
  Serial.begin(9600);

  init_lcd(); //LCD
  init_relay(); //RELAY
  init_led();
  //

  //turn all on when starting
  turn_all_on();

  // Initialize device.
  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  delay(1900); //keep welcome msg a bit before clearing....
  clear_lcd();

  turn_all_off();

 //tmp presence
 pinMode(PRESENCE_PIN,INPUT);
 pinMode(IND_PRESENCE,OUTPUT);


// setup_venting_interval();


 test_extra_pin();

}
void test_extra_pin()
{
    //test Pin 15 / A1   --> WORKS, so analog have a Digital correspondant...
  digitalWrite(15,HIGH);
  delay (35);
  digitalWrite(15,LOW);

}

int pres__is_someonethere= 0; //presence detector, works with : PRESENCE_PIN
int pres__wait_until_reset_noone = 1800; //number of loop until it shift into no one mode after someone is there
int pres__count_until_reset_noone = 0;



int is_night = 0;

int temperature = 20; //default


void loop() {






   pres__detect_presence();
   //delay (25);
  // detect_presence(); //detected presence twice really quick





  // Delay between measurements.
  delay(delayMS);


  //reset sensor on error
   error_reset_sensor();




  //reads Smoke / Air quality Value
  int airQuality = analogRead(airQualA0);

  int had_vented = 0;




  //Reads Light Sensor
  int liteSensor = analogRead(liteA2);

  //TODO something with lite_day_value  &&  lite_night_value
  if (liteSensor < lite_day_value)
  {
    //day
    is_night = 0;

    lite_set_day();
  }
  else if (liteSensor > lite_night_value)
  {
    //night
    is_night = 1;
    lite_set_night();
  }


  if (haderror==1)
  {

      clear_lcd();

  }



  //--------------------------------------------TEMPERATURE-----
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);


  if (isnan(event.temperature)) {
    lcd.home ();                   // go home
    lcd.print("Error reading temperature!");

    clear_lcd();

   //default we turn it off if there is anything wrong with sensor...
    turn_all_off();


       haderror = 1;
  }
  else {

    lcd.home ();                   // go home
    temperature = Celcius(event.temperature);


    lcd.print("T:");
    lcd.print(temperature);

    if (temperature > max_heating_temp)
    {

      //if on, turn it off
      //  relay_off(); //RELAY OFF   <--- We would be Opening the Outside ventilation in the master project.
      heat_off();
    }

    if (temperature > 31)
    {
      lcd.print("<");
      heat_off();
    }
    else
    {
      if (temperature < min_heating_temp)
        {
          lcd.print(">");

         heat_on();


        }
     else {
      heat_off();  //default we turn it off
      lcd.print(" ");

      }
    }





//------------------------------------------------------------
//-------------------------------- AIR QUALITY LOGICS-------
//start ventilate for that amount of time : vent_amount_time_sec

    //  lcd.print("        "); //clearing the rest of the line
    lcd.print(";Q:");
    lcd.print(airQuality);

    if (airQuality > min_air_qual  &&  temperature > temperature_min_confort_stop_vent )
    {
     lcd.print(">");

     min_vent_time_seconds_count = min_vent_time_seconds; //we vent for that amount of time min and more until air is ok

     vent_on();
     had_vented = 1;
    }
    else { //if Air is ok

     // if (min_vent_time_seconds_count != 0)
     // {
         min_vent_time_seconds_count--; // we decrease min time until 0 and stop it
    //  }

      if (min_vent_time_seconds_count < 0) //and min vent time is up
      {

        if (vent_amount_time_sec < vent_amount_time_count || temperature < temperature_min_confort_stop_vent)
        {
         vent_off();
         //lcd.print(" ");
         vent_amount_time_count = 0; //Reset vent time

        }

      }
    }
         lcd.print("|");
         lcd.print(min_vent_time_seconds_count);

  }
  vent_amount_time_count++; //count loop amount to reach vent time

//--------------------- AIR QUAL Vent Interval-----





//-------------------------------- AIR QUALITY LOGICS-------
//------------------------------------------------------------
//--- Goal : Ventilate each specified timeframe

       unsigned long target_vent_seconds= vent_interval_wait * 60 ; //wait seconds before we do a vent


     Serial.print(count_vent_interval);
        Serial.print("/");
        Serial.print(target_vent_seconds);
        Serial.println(" ");

      if (had_vented ==0 && count_vent_interval > target_vent_seconds )
      {
        interval_vent();
        reset_count_vent_interval();
      }

       count_vent_interval++;





  //------------------------------------- PRESENCE--------
  if (pres__is_someonethere == 1) {
    //indicate someone...

    lcd.print(";P:Y ");
    digitalWrite(IND_PRESENCE,HIGH);
  }
  else
  {

    lcd.print(";P:N ");
    digitalWrite(IND_PRESENCE,LOW);
  }



   lcd.print("   "); // what is left in the lcd to  be cleared









  //---------------------------------------------------------------------------
    //---------------------------------------------------------HUMIDITY------
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
      lcd.print( "Error reading humidity!");

     //turn all off if anything with sensor
       turn_all_off();

       haderror = 1;
  }
  else {


    //get humidity
    int humidity = event.relative_humidity/10;

    lcd.setCursor ( 0, 1 );


    lcd.print("H:");
   // delay(100);
    //lcd.scrollDisplayLeft();
    lcd.print(humidity);


    if (humidity > humidity_max)
    {
      lcd.print("<");

      humidify_off();
    }
    else
    {
      if (humidity < humidity_min)
      {
        lcd.print(">");
       humidify_on();

      }
       else {  lcd.print(" ");  }
      }

  }

    //---------------------------------------------------------HUMIDITY.main------
  //---------------------------------------------------------------------------






  //---------------------------------------------------------------------------
  //---------------------------------------------------------liteSensor.main

  lcd.print(";L:");
  lcd.print(liteSensor);

   if (liteSensor > 350 )
   {

     lcd.print("-N");

     lcd.print("  ");  //fill what is left...
   }
   else {

     lcd.print("-D");


     lcd.print("  ");   //fill what is left...
   }

}






//---------------------------------------------------------
//----------AIR Interval Vent

void reset_count_vent_interval()
{
    count_vent_interval = 0; //reset_count_vent_interval()
}


void interval_vent()
{
      Serial.println("--------------------");

      Serial.println("interval_vent Starting ");

     digitalWrite(IND_INTERVAL_VENTING,HIGH);
     turn_interval_venting_indicator_on();

     vent_on();
       long  wait_seconds = interval_vent_amount_time ;

       Serial.print(wait_seconds);

        Serial.print(" venting time");

        Serial.println(" ");
     delay(wait_seconds); //interval venting time
     vent_off();
     turn_interval_venting_indicator_off();

}
void turn_interval_venting_indicator_on(){
     digitalWrite(IND_INTERVAL_VENTING,HIGH);
}

void turn_interval_venting_indicator_off(){
     digitalWrite(IND_INTERVAL_VENTING,LOW);
}

int very_quick_vent_time = 10000; //ms for the initial very quick vent


void setup_venting_interval()
{
    clear_lcd();

    lcd.home ();                   // go home

    lcd.print("Initial venting...");

  init_venting();
  //interval_vent();
  very_quick_vent();
}


void init_venting()
{
 pinMode(RELAY_VENTING,OUTPUT);
 pinMode(IND_INTERVAL_VENTING,OUTPUT);
 pinMode(IND_POOR_AIR,OUTPUT);

 pinMode(IND_VENTING,OUTPUT);
}

void very_quick_vent()
{

      Serial.println("very_quick_vent Starting ");
     vent_on();
     int wait_seconds = very_quick_vent_time ;
     delay(wait_seconds); //quick poor venting time
     vent_off();
}












//-------------------------------------------------------------------
//-----------------------------------------------------PRESENCE
void pres__detect_presence()
{
  byte state = digitalRead(PRESENCE_PIN);
  digitalWrite(4,state);
  if (state == 1 )
  {
    //someone in the area
    pres__count_until_reset_noone = 0; // we start counting a while and will consider there is someone
    pres__is_someonethere = 1 ; //we had someone
  }
  else
  {
    if (pres__count_until_reset_noone == pres__wait_until_reset_noone ) // reached a certain amout of time with no one
    {
      //nobody there
       pres__is_someonethere = 0 ; //no one moved for now
    }
    else
    {
     pres__count_until_reset_noone ++; //increment the count and assume someone was there not so long ago
    }
  }
}












//-----------------------------------------------------------
//-----------------------------------------------------------
//-------------------------------------------------INITS-----
//-----------------------------------------------------------


void init_led()
{
  //pinMode(13, OUTPUT);

  pinMode(IND_HUMIDIFY, OUTPUT);
  pinMode(IND_HEATING, OUTPUT);
  pinMode(IND_VENTING, OUTPUT);
  pinMode(IND_DRYING, OUTPUT);
  pinMode(IND_NIGHT_LIGHT, OUTPUT);

}


//---------------------------------------------RELAY
//relay
void init_relay()
{
  pinMode(RELAY_HUMIDIFY, OUTPUT);
  pinMode(RELAY_HEATING, OUTPUT);
  pinMode(RELAY_VENTING, OUTPUT);
 // pinMode(RELAY_DRYING, OUTPUT);
  pinMode(RELAY_NIGHT_LIGHT, OUTPUT);

  //RELAY_HEATING RELAY_HUMIDIFY
}




//-----------------------------------------------------------
//-------------------------------------------------INITS-----
//-----------------------------------------------------------
//-----------------------------------------------------------










//
void lite_set_day()
{
    max_temp = max_temp_day; //starts cooling
    max_heating_temp = max_heating_temp_day; //stops heating
    min_heating_temp = min_heating_temp_day; //starts heating
}

//
void lite_set_night()
{
    max_temp = max_temp_night; //starts cooling
    max_heating_temp = max_heating_temp_night; //stops heating
    min_heating_temp = min_heating_temp_night; //starts heating
}





int drying = 0; //hold drying status
int venting = 0; //hold venting status
int heating = 0; //hold heating status
int humidifying = 0; // holds humiding status
int nighting = 0 ; //holds night light status

void error_reset_sensor()
{
   if (haderror == 1)
  {
    //------------------------------------------RESETS Status Values-----
    humidifying = 0;
    venting = 0;
    heating = 0;
    //humidify_on(); //turn on after error...

    haderror = 0;
  }
}

void turn_all_off()
{
    heat_off(); //for security, we turn off heating if error
   humidify_off(); //...
   vent_off(); //...
   dry_off();
   night_light_off();
}

void turn_all_on()
{
    heat_on();
    humidify_on(); //...
   vent_on(); //...
   dry_on();
   night_light_on();
}



















//RELAY_DRYING

void dry_on()
{
   if (drying == 0) {
      //if off turn it on.
      digitalWrite(RELAY_DRYING,0);           // Turns ON Relays drying
      digitalWrite(IND_DRYING,HIGH);
      Serial.println("drying ON");
      drying = 1;
     }
}

void dry_off()
{
   if (drying == 1) {
      //if off turn it on.
      digitalWrite(RELAY_DRYING,1);           // Turns OFF Relays drying
      digitalWrite(IND_DRYING,LOW);
      Serial.println("drying OFF");
      drying = 0;
     }
}










//RELAY_DRYING

void night_light_on()
{
   if (nighting == 0) {
      //if off turn it on.
      digitalWrite(RELAY_NIGHT_LIGHT,0);           // Turns ON Relays
      digitalWrite(IND_NIGHT_LIGHT,HIGH);
      Serial.println("nighting ON");
      nighting = 1;
     }
}

void night_light_off()
{
   if (nighting == 1) {
      //if off turn it on.
      digitalWrite(RELAY_NIGHT_LIGHT,1);           // Turns OFF Relays
      digitalWrite(IND_NIGHT_LIGHT,LOW);
      Serial.println("nighting OFF");
      nighting = 0;
     }
}











void vent_on()
{
   if (venting == 0) {
      //if off turn it on.
      digitalWrite(RELAY_VENTING,0);           // Turns ON Relays RELAY_VENTING
      digitalWrite(IND_VENTING,HIGH);
      Serial.println("venting ON");
      venting = 1;
     }
}

void vent_off()
{
   if (venting == 1) {
      //if off turn it on.
      digitalWrite(RELAY_VENTING,1);           // Turns OFF Relays RELAY_VENTING
      digitalWrite(IND_VENTING,LOW);
      Serial.println("venting OFF");
      venting = 0;
     }
}











void heat_on()
{
   if (heating == 0) {
      //if off turn it on.
      digitalWrite(RELAY_HEATING,0);           // Turns ON Relays HEATING
      digitalWrite(IND_HEATING,HIGH);
      Serial.println("Heating ON");
      heating = 1;
     }
}
void heat_off()
{
   if (heating == 1) {
      //if off turn it on.
      digitalWrite(RELAY_HEATING,1);           // Turns OFF Relays HEATING
      digitalWrite(IND_HEATING,LOW);
      Serial.println("Heating OFF");
      heating = 0;
     }
}







void humidify_on()
{
   if (humidifying == 0) {
      //if off turn it on.
      digitalWrite(RELAY_HUMIDIFY,0);           // Turns ON Relays  humidify
      digitalWrite(IND_HUMIDIFY,HIGH);
      Serial.println("Humiding ON");
      humidifying = 1;
     }

}

void humidify_off()
{
  if (humidifying ==1) {
    digitalWrite(RELAY_HUMIDIFY,1);          // Turns Relay humidify_Off
    digitalWrite(IND_HUMIDIFY,LOW);
   Serial.println("Humiding OFF");
   humidifying = 0;
  }
}

















//---------------------------------------- LCD----------------
void init_lcd()
{

  lcd.begin(16,2);               // initialize the lcd

  lcd.home ();                   // go home
  lcd.print(app_name);
  lcd.setCursor ( 0, 1 );        // go to the next line
  lcd.print (app_info);
  delay ( app_delay );
}
void clear_lcd()
{
  lcd.begin(16,2);               // initialize the lcd
}
//----------------------------------------- LCD
//---------------------------------------------------------




















//--------------------------------------- Degree CONVERTION
//Celsius to Fahrenheit conversion
double Fahrenheit(double celsius)
{
  return 1.8 * celsius + 32;
}

// fast integer version with rounding
double Celcius(int f)
{
 //return f;//  return (f / 1.8 -32);
 // return ((f / 18 - 5)*10 - 32)/10;

  f = f /10;
//  return  (f - 32) * 0.5556  + 5;
  return  (f - 32) * 0.5556  ;
}
