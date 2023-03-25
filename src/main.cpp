/**
 * IotWebConf06MqttApp.ino -- IotWebConf is an ESP8266/ESP32
 *   non blocking WiFi/AP web configuration library for Arduino.
 *   https://github.com/prampec/IotWebConf
 *
 * Copyright (C) 2020 Balazs Kelemen <prampec+arduino@gmail.com>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

/**
 * Example: MQTT Demo Application
 * Description:
 *   All IotWebConf specific aspects of this example are described in
 *   previous examples, so please get familiar with IotWebConf before
 *   starting this example. So nothing new will be explained here,
 *   but a complete demo application will be built.
 *   It is also expected from the reader to have a basic knowledge over
 *   MQTT to understand this code.
 *
 *   This example starts an MQTT client with the configured
 *   connection settings.
 *   Will post the status changes of the D2 pin in channel "test/status".
 *   Receives messages appears in channel "test/action", and writes them to serial.
 *   This example also provides the firmware update option.
 *   (See previous examples for more details!)
 *
 * Software setup for this example:
 *   This example utilizes Joel Gaehwiler's MQTT library.
 *   https://github.com/256dpi/arduino-mqtt
 *
 * Hardware setup for this example:
 *   - An LED is attached to LED_BUILTIN pin with setup On=LOW.
 *   - [Optional] A push button is attached to pin D2, the other leg of the
 *     button should be attached to GND.
 */
#include <Arduino.h>
#include "pins.h"
// #include "driver/gpio.h"
#include <esp_sleep.h>

#include <Timezone.h>
#include <MQTT.h>

#include <IotWebConf.h>
#include <IotWebConfUsing.h> // This loads aliases for easier class names.

#include <ArduinoJson.h>

#include <TimeLib.h>

#include <Battery18650Stats.h>

// epd
#include <epd_driver.h>
#include <epd_highlevel.h>

// Icons and fornts
#include "opensans12b.h"
#include "opensans16b.h"
#include "opensans24b.h"

#include "temp_img.h"
#include "hum_img.h"
#include "pres_img.h"
#include "batt_img.h"
//#include "time_img.h"

#include "r_rect_img.h"

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60 * 30  /* Time ESP32 will go to sleep (in seconds) */
// #define TIME_TO_SLEEP 60 /* Time ESP32 will go to sleep (in seconds) */

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = "testThing";

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "smrtTHNG8266";

#define STRING_LEN 128

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "mqt1"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)
#define CONFIG_PIN BUTTON_1

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
// #define STATUS_PIN LED_BUILTIN

Battery18650Stats battery(BATT_PIN,1.79);

// -- Method declarations.
void handleRoot();
void mqttMessageReceived(MQTTClient *client, char topic[], char payload[], int payload_length);
bool connectMqtt();
bool connectMqttOptions();
// -- Callback methods.
void wifiConnected();
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper);

DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
MQTTClient mqttClient(768);

// NTP time
WiFiUDP ntpUDP;

// Eastern European Time (Helsinki, Finland)
TimeChangeRule EEST = {"EEST", Last, Sun, Mar, 3, 180};     // Eastern European Summer Time
TimeChangeRule EET = {"EET ", Last, Sun, Oct, 4, 120};       // Eastern European Standard Time
Timezone EE(EEST, EET);

char mqttServerValue[STRING_LEN];
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];
char ruuvitag1Value[STRING_LEN];
char ruuvitag2Value[STRING_LEN];
char ruuvitag3Value[STRING_LEN];

IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
// -- You can also use namespace formats e.g.: iotwebconf::ParameterGroup
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN);

IotWebConfParameterGroup ruuviGroup = IotWebConfParameterGroup("ruuvi", "Ruuvitag configuration");
IotWebConfTextParameter ruuvitag1Param = IotWebConfTextParameter("Ruuvitag #1", "ruuvitag1", ruuvitag1Value, STRING_LEN);
IotWebConfTextParameter ruuvitag2Param = IotWebConfTextParameter("Ruuvitag #2", "ruuvitag2", ruuvitag2Value, STRING_LEN);
IotWebConfTextParameter ruuvitag3Param = IotWebConfTextParameter("Ruuvitag #3", "ruuvitag3", ruuvitag3Value, STRING_LEN);

bool needMqttConnect = false;
bool needReset = false;
int pinState = HIGH;
unsigned long lastReport = 0;
unsigned long lastMqttConnectionAttempt = 0;

//const char *ruuvitags[] = {"ruuvitag/F9:81:78:B2:70:BE", "ruuvitag/ED:30:75:FE:CD:37", "ruuvitag/CD:8C:07:25:4B:54"};

String ruuvitags[3];
int ruuvitagIndex = 0;

const char *ruuvi_name;
const char *ruuvi_mac;
float ruuvi_temperature;
float ruuvi_humidity;
long ruuvi_pressure;
float ruuvi_batteryVoltage;
unsigned long ruuvi_timestamp;

bool enableSleep = false;

StaticJsonDocument<768> doc;

/*E-Paper*/
#define WAVEFORM EPD_BUILTIN_WAVEFORM

EpdiyHighlevelState hl;
// ambient temperature around device
int ambient_temperature = 21;

int cursor_x;
int cursor_y;

uint8_t *fb;
enum EpdDrawError err;

// CHOOSE HERE YOU IF YOU WANT PORTRAIT OR LANDSCAPE
// both orientations possible
// EpdRotation orientation = EPD_ROT_PORTRAIT;
EpdRotation orientation = EPD_ROT_LANDSCAPE;

const int tzHours = 2;      // aikavyöhykkeen tuntieroaika UTC:hen nähden
const int tzMinutes = 0;    // aikavyöhykkeen minuutin eroaika UTC:hen nähden
const int dstOffset = 3600; // kesäajan tunnin siirtymä
const int stdOffset = 0;    // talviajan tunnin siirtymä

/*End of E-Paper*/

void draw_sensors_top(const char *sensor_text, int cursor_x, int cursor_y)
{
  EpdFontProperties font_props = epd_font_properties_default();
  font_props.flags = EPD_DRAW_ALIGN_CENTER;

  epd_write_string(&OpenSans24B, sensor_text, &cursor_x, &cursor_y, fb, &font_props);
}

void draw_sensors_frame(int cursor_x, int cursor_y, int width, int height)
{
  EpdRect r_rect_img_area = {
      .x = cursor_x,
      .y = cursor_y,
      .width = width,
      .height = height,
  };
  epd_copy_to_framebuffer(r_rect_img_area, (uint8_t *)r_rect_img_data, fb);
}

void draw_sensors_label(int img_cursor_x, int img_cursor_y, int img_width, int img_height, const uint8_t *img_data, const char *label_text, int text_cursor_x, int text_cursor_y)
{
  EpdRect img_area = {
      .x = img_cursor_x,
      .y = img_cursor_y,
      .width = img_width,
      .height = img_height,
  };

  epd_copy_to_framebuffer(img_area, (uint8_t *)img_data, fb);

  EpdFontProperties font_props = epd_font_properties_default();
  font_props.flags = EPD_DRAW_ALIGN_LEFT;
  cursor_x = text_cursor_x;
  cursor_y = text_cursor_y;
  epd_write_string(&OpenSans16B, label_text, &cursor_x, &cursor_y, fb, &font_props);
}

void draw_sensors_value(const char *sensor_value, int cursor_x, int cursor_y)
{
  EpdFontProperties font_props = epd_font_properties_default();
  font_props.flags = EPD_DRAW_ALIGN_RIGHT;

  epd_write_string(&OpenSans16B, sensor_value, &cursor_x, &cursor_y, fb, &font_props);
}

void draw_sensors_datetime(const char *sensor_value, int cursor_x, int cursor_y)
{
  EpdFontProperties font_props = epd_font_properties_default();
  font_props.flags = EPD_DRAW_ALIGN_RIGHT;

  epd_write_string(&OpenSans12B, sensor_value, &cursor_x, &cursor_y, fb, &font_props);
}

void draw_bottom_battery(const char *battery_value, int cursor_x, int cursor_y)
{
  EpdFontProperties font_props = epd_font_properties_default();
  font_props.flags = EPD_DRAW_ALIGN_LEFT;

  epd_write_string(&OpenSans12B, battery_value, &cursor_x, &cursor_y, fb, &font_props);
}

void setup()
{
  delay(500);
  Serial.begin(115200);
  // Serial.begin(9600);
  Serial.println();
  Serial.println("Starting up...");
  //

  // First setup epd to use later
  epd_init(EPD_OPTIONS_DEFAULT);
  hl = epd_hl_init(WAVEFORM);
  epd_set_rotation(orientation);
  fb = epd_hl_get_framebuffer(&hl);
  epd_hl_set_all_white(&hl);

  Serial.print("Battery: ");
  char buff[32];
  Serial.println(battery.getBatteryChargeLevel());
  String batt_buff = "Battery: " + String(battery.getBatteryChargeLevel(true)) + "% " + String(battery.getBatteryVolts()) + " V";
  batt_buff.toCharArray(buff, batt_buff.length() + 1);
  draw_bottom_battery(buff, 20, 520);

  epd_poweroff();
  int x;
  for (x = 10; x <= 650; x = x + 320)
  {
    draw_sensors_frame(x, 100, 300, 350);
  }
  x = 0;
  for (x = 20; x <= 660; x = x + 320)
  {
    draw_sensors_label(x, 105, 60, 60, temp_img_data, " °C", x + 195, 148);
    draw_sensors_label(x, 175, 60, 60, hum_img_data, " %", x + 195, 218);
    draw_sensors_label(x, 245, 60, 60, pres_img_data, " hpa", x + 195, 288);
    draw_sensors_label(x, 315, 60, 60, batt_img_data, " V", x + 195, 358);
  }

  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  ruuviGroup.addItem(&ruuvitag1Param);
  ruuviGroup.addItem(&ruuvitag2Param);
  ruuviGroup.addItem(&ruuvitag3Param);

  // iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addParameterGroup(&mqttGroup);
  iotWebConf.addParameterGroup(&ruuviGroup);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);

  // -- Initializing the configuration.
  bool validConfig = iotWebConf.init();
  if (!validConfig)
  {
    mqttServerValue[0] = '\0';
    mqttUserNameValue[0] = '\0';
    mqttUserPasswordValue[0] = '\0';
    ruuvitag1Value[0] = '\0';
    ruuvitag2Value[0] = '\0';
    ruuvitag3Value[0] = '\0';
  }

  ruuvitags[0] = ruuvitag1Value;
  ruuvitags[1] = ruuvitag2Value;
  ruuvitags[2] = ruuvitag3Value;

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", []
            { iotWebConf.handleConfig(); });
  server.onNotFound([]()
                    { iotWebConf.handleNotFound(); });

  
  

  mqttClient.begin(mqttServerValue, net);
  mqttClient.onMessageAdvanced(mqttMessageReceived);

  

  Serial.println("Ready.");
}
 


/**
 * Input time in epoch format and return tm time format
 * by Renzo Mischianti <www.mischianti.org>
 */
static tm getDateTimeByParams(long time)
{
  struct tm *newtime;
  const time_t tim = time;
  newtime = localtime(&tim);
  return *newtime;
}

/**
 * Input tm time format and return String with format pattern
 * by Renzo Mischianti <www.mischianti.org>
 */
static String getDateTimeStringByParams(tm *newtime, char *pattern = (char *)"%d/%m/%Y %H:%M:%S")
{
  char buffer[30];
  strftime(buffer, 30, pattern, newtime);
  return buffer;
}

/**
 * Input time in epoch format format and return String with format pattern
 * by Renzo Mischianti <www.mischianti.org>
 */
static String getEpochStringByParams(long time, char *pattern = (char *)"%d/%m/%Y %H:%M:%S")
{
  //    struct tm *newtime;
  tm newtime;
  newtime = getDateTimeByParams(time);
  return getDateTimeStringByParams(&newtime, pattern);
}

void loop()
{
  // -- doLoop should be called as frequently as possible.
  iotWebConf.doLoop();
  mqttClient.loop();
  // Serial.println(tagNumber);

  if (needMqttConnect)
  {
    if (connectMqtt())
    {
      needMqttConnect = false;
    }
  }
  else if ((iotWebConf.getState() == iotwebconf::OnLine) && (!mqttClient.connected()))
  {
    Serial.println("MQTT reconnect");
    connectMqtt();
  }

  if (needReset)
  {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }

  // mqttClient.subscribe(ruuvitags[ruuvitagIndex]);
  ruuvi_pressure = doc["pressure"];

  if (ruuvi_pressure != 0)
  {
    String str;
    ruuvi_name = doc["name"];
    ruuvi_mac = doc["mac"];
    ruuvi_temperature = doc["temperature"];
    ruuvi_humidity = doc["humidity"];
    ruuvi_batteryVoltage = doc["batteryVoltage"];
    ruuvi_timestamp = doc["timestamp"];

    switch (ruuvitagIndex)
    {
    case 0:
      cursor_x = 160;
      break;

    case 1:
      cursor_x = 480;
      break;

    case 2:
      cursor_x = 800;
      break;
    }
    cursor_y = 60;
    draw_sensors_top(ruuvi_name, cursor_x, 60);

    char buff[32];
    dtostrf(ruuvi_temperature, 7, 2, buff);
    draw_sensors_value(buff, cursor_x + 55, 148);
    dtostrf(ruuvi_humidity, 7, 2, buff);
    draw_sensors_value(buff, cursor_x + 55, 218);
    dtostrf(ruuvi_pressure * 0.01, 7, 2, buff);
    draw_sensors_value(buff, cursor_x + 55, 288);
    dtostrf(ruuvi_batteryVoltage, 7, 2, buff);
    draw_sensors_value(buff, cursor_x + 55, 358);

    char timeformat[] = "%d/%m/%y %H:%M:%S";
    String time_buff = getEpochStringByParams(EE.toLocal(ruuvi_timestamp),timeformat);
    time_buff.toCharArray(buff,time_buff.length()+ 1);
    draw_sensors_datetime(buff, cursor_x + 100, 428);
    // if (isDST(t)) {
    //   adjustTime(dstOffset); // siirrä aikaa kesäajan mukaisesti
    // } else {
    //   adjustTime(stdOffset); // siirrä aikaa talviajan mukaisesti
    // }

    // draw_sensors_value(t, cursor_x + 55, 428);

    // String buff = getEpochStringByParams(EE.toLocal(ruuvi_timestamp),"%d/%m/%y %H:%M:%S");

    // Serial.println(ruuvi_name);
    // Serial.println(str + "temperature " + ruuvi_temperature + " C");
    // Serial.println(str + "humidity " + ruuvi_humidity + " %");
    // Serial.println(str + "pressure " + ruuvi_pressure + " hPa");
    // Serial.println(str + "battery " + ruuvi_batteryVoltage + " V");
    // Serial.println("-----------------");

    doc.clear();

    if (ruuvitagIndex < 2)
    {
      mqttClient.unsubscribe(ruuvitags[ruuvitagIndex]);
      ++ruuvitagIndex;
      mqttClient.subscribe(ruuvitags[ruuvitagIndex]);
    }
    else
    {
      enableSleep = true;
    }
  }
  // else
  // {
  //   mqttClient.subscribe(ruuvitags[ruuvitagIndex]);
  // }

  if (enableSleep)
  {
    WiFi.disconnect();
    epd_poweron();
    epd_clear();
    ambient_temperature = epd_ambient_temperature();
    EpdDrawError err = epd_hl_update_screen(&hl, MODE_GC16, ambient_temperature);
    epd_poweroff();
    Serial.print("Time passed: ");
    Serial.print(millis());
    Serial.println(" ms");
    epd_deinit();
    
    gpio_reset_pin(GPIO_NUM_0);
    gpio_reset_pin(GPIO_NUM_2);
    gpio_reset_pin(GPIO_NUM_4);
    gpio_reset_pin(GPIO_NUM_12);
    gpio_reset_pin(GPIO_NUM_13);
    gpio_reset_pin(GPIO_NUM_14);
    gpio_reset_pin(GPIO_NUM_15);
    gpio_reset_pin(GPIO_NUM_25);
    gpio_reset_pin(GPIO_NUM_26);
    gpio_reset_pin(GPIO_NUM_27);
    gpio_reset_pin(GPIO_NUM_32);
    gpio_reset_pin(GPIO_NUM_33);
    gpio_reset_pin(GPIO_NUM_34);
    gpio_reset_pin(GPIO_NUM_35);
    gpio_reset_pin(GPIO_NUM_36);
    gpio_reset_pin(GPIO_NUM_37);
    gpio_reset_pin(GPIO_NUM_38);
    gpio_reset_pin(GPIO_NUM_39);
    
    Serial.println("Time to sleep");
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
}

/**
 * Handle web requests to "/" path.
 */
void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>IotWebConf 06 MQTT App</title></head><body>MQTT App demo";
  s += "<ul>";
  s += "<li>MQTT server: ";
  s += mqttServerValue;
  s += "</li>";
  s += "</ul>";
  s += "Go to <a href='config'>configure page</a> to change values.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void wifiConnected()
{
  needMqttConnect = true;
}

void configSaved()
{
  Serial.println("Configuration was updated.");
  needReset = true;
}

bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper)
{
  Serial.println("Validating form.");
  bool valid = true;

  int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
  if (l < 3)
  {
    mqttServerParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  return valid;
}

// bool connectMqtt() {
//   unsigned long now = millis();
//   if (1000 > now - lastMqttConnectionAttempt)
//   {
//     // Do not repeat within 1 sec.
//     return false;
//   }
//   Serial.println("Connecting to MQTT server...");
//   if (!connectMqttOptions()) {
//     lastMqttConnectionAttempt = now;
//     return false;
//   }
//   Serial.println("Connected!");
//   //mqttClient.unsubscribe(ruuvitags[ruuvitagIndex]);
//    if (ruuvitagIndex < 2) {
//     mqttClient.subscribe(ruuvitags[ruuvitagIndex]);
//     Serial.println(ruuvitags[ruuvitagIndex]);
//   }

//   return true;
// }

// -- This is an alternative MQTT connection method.
bool connectMqtt()
{
  Serial.println("Connecting to MQTT server...");
  while (!connectMqttOptions())
  {
    iotWebConf.delay(500);
  }
  Serial.println("Connected!");
  Serial.println(ruuvitags[0]);
  Serial.println(ruuvitags[1]);
  Serial.println(ruuvitags[2]);
  //   if (ruuvitagIndex < 2) {
  //    mqttClient.subscribe(ruuvitags[ruuvitagIndex]);
  //    Serial.println(ruuvitags[ruuvitagIndex]);
  //    Serial.println(millis());
  //  }
  // Serial.println(ruuvitags[ruuvitagIndex]);
  // Serial.println(ruuvitagIndex);
  mqttClient.subscribe(ruuvitags[ruuvitagIndex]);
  return true;
}

bool connectMqttOptions()
{
  bool result;
  if (mqttUserPasswordValue[0] != '\0')
  {
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue, mqttUserPasswordValue);
  }
  else if (mqttUserNameValue[0] != '\0')
  {
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue);
  }
  else
  {
    result = mqttClient.connect(iotWebConf.getThingName());
  }
  return result;
}

void mqttMessageReceived(MQTTClient *client, char topic[], char payload[], int payload_length)
{
  deserializeJson(doc, payload, payload_length);
}
