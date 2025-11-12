

/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TCA9554.h>
#include "Adafruit_SHTC3.h"
#include <DHT.h>
#include "XMQTTCliente.h"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C  // Common I2C address for OLED
#define SDA_CH1 21
#define SCL_CH1 45
#define SDA_CH2 42
#define SCL_CH2 41
#define DHTPIN 3

float temp;
float press;
const char* ssid = "eir59539728";
const char* password = "Radtel!TnY12671223";
const char* mqttServer = "192.168.1.11";
const uint16_t mqttPort = 1883;
const char* clientId = "ESP32Client_SensorTesting";
const char* willTopic = "testing/sensors";
const char* willMessage = "offline";

TwoWire I2C_CH1 = TwoWire(0);
TwoWire I2C_CH2 = TwoWire(1);
DHT dht(DHTPIN,DHT11);
XMQTTCliente mqtt(mqttServer, mqttPort, clientId, willTopic, willMessage,0,true);

// Pass the custom I2C bus to the constructor
Adafruit_BMP280 bmp_CH1(&I2C_CH1); // BMP280 Sensor  
TCA9554 gpioExpander(0x20,&I2C_CH2); //IO Expansion
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_CH1, -1); //OLED Display


Adafruit_Sensor *bmp_temp = bmp_CH1.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp_CH1.getPressureSensor();
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3(); // SHTC3 Sensor 

//MQTT Callback fucntion (Message reception), all time should be defined
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


void setup() {
  Serial.begin(115200); 
  
  unsigned status_CH1;
  unsigned status_CH2;
 

  I2C_CH1.begin(SDA_CH1,SCL_CH1); //Change the standart ESP32 I2C pins
  I2C_CH2.begin(SDA_CH2,SCL_CH2);
  gpioExpander.begin(OUTPUT);
  status_CH1 = bmp_CH1.begin(0x76); // BMP280 sensor
  dht.begin();
    
  //Reset all the Outputs
  for (uint pin =0; pin < 8; pin++){
   gpioExpander.write1(pin,LOW);   
  }
  //SHTC3 Sensor init
  Serial.println("SHTC3 test");
  if (! shtc3.begin(&I2C_CH1)) {
    Serial.println("Couldn't find SHTC3");
    while (1) delay(1);
  }
  Serial.println("Found SHTC3 sensor");  


  if (!status_CH1) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp_CH1.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);    
  }
  
  /* BMP280 Default settings from datasheet. */
  bmp_CH1.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

  //OLED Display initializing 
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Hello from OLED!");
  display.display();

  //MQTT Cmmunication 
  mqtt.begin(ssid, password);
  mqtt.setCallback(callback);
  mqtt.subscribe("test/topic");
  mqtt.publish("status/ESP32Client", "online");

}

void loop() {
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  temp = temp_event.temperature;
  press = pressure_event.pressure;
  float temperature_DHT11 = dht.readTemperature();  // Celsius
  float humidity_DHT11 = dht.readHumidity();        // %

  // Temperaure Alarm 
  if(temp <20) gpioExpander.write1(4,HIGH);
   else gpioExpander.write1(4,LOW);
  
  //Serial printing 
  Serial.print(F("Temperature = "));
  Serial.print(temp);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(press);
  Serial.println(" hPa");
  Serial.println();
  // SHTC3 Sensor 
  sensors_event_t humidity, temp_SHTC3;
  shtc3.getEvent(&humidity, &temp_SHTC3);// populate temp and humidity objects with fresh data
  Serial.print("Temperature SHTC3: "); Serial.print(temp_SHTC3.temperature); Serial.println(" degrees C");
  Serial.print("Humidity SHTC3: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  //DHT 11
  Serial.print("Temp DHT11: ");
  Serial.print(temperature_DHT11);
  Serial.print(" °C, Humi DHT11: ");
  Serial.print(humidity_DHT11);
  Serial.println(" %");
  
  //Show on OLED display 
  display.clearDisplay();
  display.setCursor(0, 0);
  
  display.print("Temp BMP: "); //BMP280
  display.print(temp);
  display.println(" C");
  
  display.print("Temp SHTC3: "); //SHTC3
  display.print(temp_SHTC3.temperature);
  display.println(" C");
  display.print("Temp DHT11: "); //DHT11
  display.print(temperature_DHT11);
  display.println(" C");

  display.print("Press: "); //BMP280
  display.print(press);
  display.println(" hPa");

  display.print("Hum SHT: "); //SHTC3
  display.print(humidity.relative_humidity);
  display.println(" rH");
  display.print("Hum DHT: "); //DHT11
  display.print(humidity_DHT11);
  display.println(" rH");

  display.display();
  // MQTT publshing
  
  mqtt.publish("mountoval/smallroom/temperatureSHTC3",String(temp_SHTC3.temperature).c_str());
  mqtt.publish("mountoval/smallroom/humiditySHTC3", String(temp_SHTC3.temperature).c_str());

  mqtt.publish("mountoval/smallroom/temperaureDHT11", String(temperature_DHT11).c_str());
  mqtt.publish("mountoval/smallroom/humidityDHT11", String(humidity_DHT11).c_str());

  mqtt.publish("mountoval/smallroom/temperatureBMP280", String(temp).c_str());
  mqtt.publish("mountoval/smallroom/pressionBMP280", String(press).c_str()); 

  mqtt.loop();
  delay(2000);
}
