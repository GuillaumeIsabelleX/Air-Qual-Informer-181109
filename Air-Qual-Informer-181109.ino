

String app_name = "AiRQual Analyzer";
String app_info = "by JG 191109";

//bellow ok, above not ok
int airQualThreahold = 140;
int airQualThreaholdBad = 240;

int app_delay = 2100; //delay show app start info

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Set the LCD I2C address

uint32_t delayMS;

int airQualA0 = A0;

int count_vent_interval = 0;

#define DHTPIN 2 // Pin which is connected to the DHT sensor.

// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11
#define DHTTYPE DHT22 // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

void setup()
{
  Serial.begin(9600);

  init_lcd(); //LCD

  // Initialize device.
  dht.begin();

  Serial.println("DHTxx Unified Sensor Example");

  // Print temperature sensor details.
  sensor_t sensor;

  dht.temperature().getSensor(&sensor);

  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" *C");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" *C");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println("%");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println("%");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println("%");
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  //delay(1900); //keep welcome msg a bit before clearing....
  clear_lcd();
}

int temperature = 20; //default
String spacer = "    ";

void loop()
{

  //--------------------------------------------TEMPERATURE-----
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);

  //reads Smoke / Air quality Value
  int airQuality1 = analogRead(airQualA0);

  lcd.home(); // go home
  temperature = Celcius(event.temperature);

  lcd.print("T:");
  lcd.print(temperature);

  //---------------------------------------------------------HUMIDITY------
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);

  //get humidity
  int humidity1 = event.relative_humidity / 10;

  delay(100);
  int airQuality2 = analogRead(airQualA0);
  int humidity2 = event.relative_humidity / 10;

  int airQuality = (airQuality1 + airQuality2) / 2;
  int humidity = (humidity1 + humidity2) / 2;
  lcd.print(spacer + "Q:");
  lcd.print(airQuality);

  //ENTER
  lcd.setCursor(0, 1);

  lcd.print("H:");
  // delay(100);
  //lcd.scrollDisplayLeft();
  lcd.print(humidity);

  //    lcd.home ();      z

  if (airQuality < airQualThreahold)
    lcd.print(spacer + "Ok :)   ");
  else if (airQuality > airQualThreahold && airQuality < airQualThreaholdBad)
    lcd.print(spacer + "Soso :| ");
  else if (airQuality > airQualThreaholdBad)
    lcd.print(spacer + "Bad :( ");

  delay(1500);
  //   clear_lcd();
}

//---------------------------------------- LCD----------------
void init_lcd()
{

  lcd.begin(16, 2); // initialize the lcd

  lcd.home(); // go home
  lcd.print(app_name);
  lcd.setCursor(0, 1); // go to the next line
  lcd.print(app_info);
  delay(app_delay);
}
void clear_lcd()
{
  lcd.begin(16, 2); // initialize the lcd
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

  f = f / 10;
  //  return  (f - 32) * 0.5556  + 5;
  return (f - 32) * 0.5556;
}