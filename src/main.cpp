/*
 * Led Strip SMD 5050 controlled by ESP12
 * Created by Jose Rivera, Jun 2018.
 *
 * This work is licensed under a Creative Commons Attribution 4.0 International License.
 * http://creativecommons.org/licenses/by/4.0/
 */

/*
 * This sketch for the ESP12 to control a strip of leds SMD 5050
 * RGBW. It can also work without the white LED (W). The sketch considers a
 * circuit with a button that allows to change the mode of lights of the strip
 * of leds and a potentiometer to vary the brightness intensity or the speed of
 * the changes of lights.
 *
 * White light mode
 * When the driver starting to work, the led strip lights up with the white led,
 * with a brightness intensity determined by the value provided by the
 * potentiometer.
 *
 * Color light mode
 * While in the white light mode, pressing the button switches to the color mode,
 * that is, it turns off the white LED and shows the color defined by default
 * with the RGB LEDs, you can change the color of the LED strip by varying the
 * potentiometer value.
 *
 * Strobe mode
 * While in the color light mode, pressing the button switches to Strobe mode
 * using the same color as before, in the same way you can change the color with
 * the variation of the potentiometer.
 *
 * Flash mode
 * While in Strobe mode, pressing the button switches to Flash mode which
 * displays a predefined color sequence (mainly primary colors), you can change
 * the speed of the color change of the sequence, varying the speed of the
 * potentiometer.
 *
 * Fade mode
 * While in Flash mode, pressing the button switches to Fade mode which displays
 * a predefined sequence of colors, the transition between colors is more
 * gradual than in Flash mode, but its speed can be modified by varying the
 * value of the potentiometer.
 *
 * Off mode
 * If the button is held down for approximately one second, all the LEDs will
 * turn off. To turn on again you can press the button or modify the value of
 * the potentiometer.
 *
 * When not configuration, AP "Driver 5050" with pass "ledstrip" is started to
 * establish the Network configuration to connect to the Internet. Additionally,
 * it allows to configure the Host, port and topic of the MQTT server and
 * the Blynk Token.
 *
 * You can send instructions through the Blynk application using the following
 * virtual pins (virtual pin: Widget [description]):
 *    V0: zeRGBa [set color for RGB Led]
 *    V1: Slider [set intensity of the white Led 0-255]
 *    V2: Menu [to select the RGB Led mode 1-Normal, 2-Strobe, 3-Flash, 4-Fade]
 *    V3: Button (push) [set the next RGB LED mode]
 *    V4: Led [status of the white Led]
 *    V5: Led [status of the red LED]
 *    V6: Led [green Led status]
 *    V7: Led [blue Led status]
 *    V8: Button (switch) [turn on or off the white LED]
 *
 * The status of the Leds is sent and commands can be received through MQTT
 * usign:
 *
 *  Telemetry
 *    {topic}/tele/STATE {"white": {"state": "ON | OFF", intensity: 0-1024},
 *          "rgb": {"state": "ON | OFF", "mode": 0-4 , "color": 0-16777215}}
 *  Status
 *    {topic}/stat/STATE {"white": {"state": "ON | OFF", intensity: 0-1024},
 *          "rgb": {"state": "ON | OFF", "mode": 0-4 , "color": 0-16777215}}
 *
 *  Commands
 *    {topic}/cmnd/white [ON | OFF]
 *    {topic}/cmnd/white/intensity [0-1024]
 *    {topic}/cmnd/rgb [ON | OFF]
 *    {topic}/cmnd/rgb/mode [Normal | Strobe | Fade | Flash]
 *    {topic}/cmnd/rgb/color 0-16777215
 *
 * TODO: Rest API and Websockets
 */

#include <Arduino.h>
#include <FS.h>

#include "BtnHandler.h"
#include "LedStrip.h"
#include "LedStripRGB.h"

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>         //https://github.com/knolleary/pubsubclient
#include <BlynkSimpleEsp8266.h>   //http://www.blynk.cc

//uncomment this line if using a Common Anode LED
//#define COMMON_ANODE

char mqtt_server[40];
char mqtt_port[6];
char mqtt_topic[50];
char blynk_server[40];
char blynk_port[6];
char blynk_token[34];

//flag for saving data
bool shouldSaveConfig = false;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

long mqttLastMsg = 0;
long mqttLastConnect = 0;

// Send telemetry each 5 minutes // TODO: 300 000
#define MQTT_TELEMETRY_INTERVAL 300000
#define MQTT_RETRY_CONNECT_INTERVAL 30000

// It allows to avoid that small variations of voltage turn on the light
#define THRESHOLD_FOR_TURN_ON 100

const uint8_t RED_PIN = D2;
const uint8_t GREEN_PIN = D1;
const uint8_t BTN_MODE_PIN = D3;
const uint8_t BLUE_PIN = D7;
const uint8_t WHITE_PIN = D6;
const uint8_t POT_COLOR_PIN = A0;

const char CONFIG_FILE[] = "/config.json";
const char KEY_MQTT_SERVER[] = "mqtt_server";
const char KEY_MQTT_PORT[] = "mqtt_port";
const char KEY_MQTT_TOPIC[] = "mqtt_topic";
const char KEY_BLYNK_SERVER[] = "blynk_server";
const char KEY_BLYNK_PORT[] = "blynk_port";
const char KEY_BLYNK_TOKEN[] = "blynk_token";

// Set a default color for the color mode
const uint32_t DEFAULT_COLOR = COLOR_DARKPURPLE;
uint32_t last_color = COLOR_WHITE;

// Allows validation if there is a change in voltage
uint16_t last_pot_color_value = 1;

// Instance that allows to handle the RGB leds of the strip of leds
LedStripRGB led_strip_rgb({ RED_PIN, GREEN_PIN, BLUE_PIN });
// Instance that allows to handle the led of white light of the strip of leds
LedStrip led_strip_w(WHITE_PIN);

// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println(F("Should save config."));
  shouldSaveConfig = true;
}

void saveConfig() {
  Serial.println(F("Saving config... "));
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json[KEY_MQTT_SERVER] = mqtt_server;
  json[KEY_MQTT_PORT] = mqtt_port;
  json[KEY_MQTT_TOPIC] = mqtt_topic;
  json[KEY_BLYNK_SERVER] = blynk_server;
  json[KEY_BLYNK_PORT] = blynk_port;
  json[KEY_BLYNK_TOKEN] = blynk_token;

  File configFile = SPIFFS.open(CONFIG_FILE, "w");
  if (!configFile) {
    Serial.println(F("Failed to open config file for writing"));
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
  //end save
}

void mountFS() {
  if (SPIFFS.begin()) {
    Serial.println(F("Mounted file system"));
    if (SPIFFS.exists(CONFIG_FILE)) {
      //file exists, reading and loading
      Serial.println(F("Reading config file..."));
      File configFile = SPIFFS.open(CONFIG_FILE, "r");
      if (configFile) {
        Serial.println(F("Opened config file..."));
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println(F("\nparsed json..."));

          strcpy(mqtt_server, json[KEY_MQTT_SERVER]);
          strcpy(mqtt_port, json[KEY_MQTT_PORT]);
          strcpy(mqtt_topic, json[KEY_MQTT_TOPIC]);
          strcpy(blynk_server, json[KEY_BLYNK_SERVER]);
          strcpy(blynk_port, json[KEY_BLYNK_PORT]);
          strcpy(blynk_token, json[KEY_BLYNK_TOKEN]);

        } else {
          Serial.println(F("failed to load json config"));
        }
      }
    }
  } else {
    Serial.println(F("Failed to mount FS"));
  }
}

String getState()
{
  StaticJsonBuffer<512> jsonBuffer;
  JsonObject &root = jsonBuffer.createObject();
  // root["uptime"] = millis();
  JsonObject &white = root.createNestedObject("white");
  JsonObject &rgb = root.createNestedObject("rgb");

  if(led_strip_w.getState() == LedStripState::ON)
  {
    white["state"] = "ON";
    white["intensity"] = led_strip_w.getIntensity();
  } else {
    white["state"] = "OFF";
    white["intensity"] = 0;
  }

  if(led_strip_rgb.getState() == LedStripState::ON)
  {
    rgb["state"] = "ON";
    LedStripRgbMode mode = led_strip_rgb.getMode();
    switch (mode) {
      case LedStripRgbMode::NORMAL:
        rgb["mode"] = "NORMAL";
        break;
      case LedStripRgbMode::STROBE:
        rgb["mode"] = "STROBE";
        break;
      case LedStripRgbMode::FLASH:
        rgb["mode"] = "FLASH";
        break;
      case LedStripRgbMode::FADE:
        rgb["mode"] = "FADE";
        break;
    }
  } else {
    rgb["state"] = "OFF";
    rgb["mode"] = "";
  }
  RGBColor c = led_strip_rgb.getRGBColor();
  rgb["color"] = "#" + String(c.red, HEX) + String(c.green, HEX) + String(c.blue, HEX);

  String json;
  root.printTo(json);
  return json;
}

void mqttSendTele() {
  long now = millis();
  if (now - mqttLastMsg > MQTT_TELEMETRY_INTERVAL) {
    mqttLastMsg = now;

    String json = getState();

    char teleTopic[] = "/tele/STATE";
    char topic[sizeof(mqtt_topic) + sizeof(teleTopic) + 1];
    sprintf(topic, "%s%s", mqtt_topic, teleTopic);
    char payload[json.length() + 1];
    json.toCharArray(payload, json.length() + 1);
    Serial.printf("%s %s\r\n", topic, payload);
    mqttClient.publish(topic, payload);
  }
}

void mqttSendStat()
{
  String json = getState();

  char statTopic[] = "/stat/STATE";
  char topic[sizeof(mqtt_topic) + sizeof(statTopic) + 1];
  sprintf(topic, "%s%s", mqtt_topic, statTopic);
  char payload[json.length() + 1];
  json.toCharArray(payload, json.length() + 1);
  Serial.printf("%s %s\r\n", topic, payload);
  mqttClient.publish(topic, payload);
}

WidgetLED whiteLed(V4);
WidgetLED redLed(V5);
WidgetLED greenLed(V6);
WidgetLED blueLed(V7);

void updateWidgets(void)
{
  if(led_strip_w.getState() == LedStripState::ON)
  {
    whiteLed.setValue(led_strip_w.getIntensity());
    Blynk.virtualWrite(V8, 1);
  } else {
    whiteLed.off();
    Blynk.virtualWrite(V8, 0);
  }
  if(led_strip_rgb.getState() == LedStripState::ON)
  {
    RGBColor color = led_strip_rgb.getRGBColor();
    redLed.setValue(color.red);
    greenLed.setValue(color.green);
    blueLed.setValue(color.blue);
    Blynk.virtualWrite(V2, led_strip_rgb.getMode() + 1);
  } else {
    redLed.off();
    greenLed.off();
    blueLed.off();
    Blynk.virtualWrite(V2, 0);
  }
  mqttSendStat();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  Serial.print(topic);
  Serial.print(" ");

  char caPayload[length];

  for (unsigned int i = 0; i < length; i++) {
    caPayload[i] = payload[i];
    Serial.print((char)payload[i]);
  }
  Serial.println();

  String strTopic = String(topic);
  String strPayload = String(caPayload);

  strPayload.trim();
  strPayload.toLowerCase();

  if (strTopic.endsWith("/white"))
  {
    if (strPayload.startsWith("on"))
    {
      led_strip_w.turnOn();
    } else if(strPayload.startsWith("off"))
    {
      led_strip_w.turnOff();
    }
  } else if(strTopic.endsWith("/white/intensity"))
  {
    uint32_t intensity = strPayload.toInt();
    led_strip_w.setIntensity(intensity);
  } else if(strTopic.endsWith("/rgb"))
  {
    if (strPayload.startsWith("on"))
    {
      led_strip_rgb.turnOn();
    } else if(strPayload.startsWith("off"))
    {
      led_strip_rgb.turnOff();
    }
  } else if(strTopic.endsWith("/rgb/mode"))
  {
    if(strPayload.startsWith("normal"))
    {
      led_strip_rgb.setMode(LedStripRgbMode::NORMAL);
    } else if(strPayload.startsWith("strobe"))
    {
      led_strip_rgb.setMode(LedStripRgbMode::STROBE);
    } else if(strPayload.startsWith("flash"))
    {
      led_strip_rgb.setMode(LedStripRgbMode::FLASH);
    } else if(strPayload.startsWith("fade"))
    {
      led_strip_rgb.setMode(LedStripRgbMode::FADE);
    }
    led_strip_rgb.turnOn();
  } else if(strTopic.endsWith("/rgb/color"))
  {
    uint32_t color = strPayload.toInt();
    led_strip_rgb.setColor(color);
  }
  updateWidgets();
}

void mqttConnect() {
  long now = millis();
  if (now - mqttLastConnect > MQTT_RETRY_CONNECT_INTERVAL)
  {
    mqttLastConnect = millis();
    Serial.print(F("Attempting MQTT connection..."));
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(F("Connected"));
      // Once connected, publish an announcement...
      char teleTopic[] = "/tele/LWT";
      char topic[sizeof(mqtt_topic) + sizeof(teleTopic) + 1];
      sprintf(topic, "%s%s", mqtt_topic, teleTopic);
      Serial.printf("%s ONLINE\r\n", topic);
      mqttClient.publish(topic, "ONLINE");
      // ... and resubscribe
      char cmndTopic[] = "/cmnd/#";
      char subTopic[sizeof(mqtt_topic) + sizeof(cmndTopic) + 1];
      sprintf(subTopic, "%s%s", mqtt_topic, cmndTopic);
      Serial.print(F("Subscribe to "));
      Serial.println(subTopic);
      mqttClient.subscribe(subTopic);
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(mqttClient.state());
      Serial.print(F(" Try again in "));
      Serial.print(MQTT_RETRY_CONNECT_INTERVAL / 1000);
      Serial.println(F(" seconds"));
    }
  }
}

BLYNK_WRITE(V0) // zeRGBa assigned to V0
{
  int red = param[0].asInt();
  int green = param[1].asInt();
  int blue = param[2].asInt();

  red = (red & 0xFF) << 16;
  green = (green & 0xFF) << 8;
  blue = blue & 0xFF;

  uint32_t color = red + green + blue;
  led_strip_rgb.setColor(color);
  updateWidgets();
}

BLYNK_WRITE(V1) // Slider (0 - 255) to V1
{
  // Light intensity
  int intensity = param[0].asInt();
  led_strip_w.setIntensity(intensity);
  updateWidgets();
}

BLYNK_WRITE(V2) // Menu [Normal, Strobe, Flash, Fade]  to V2
{
  // Menu option selected
  switch (param[0].asInt())
  {
    case 1: { // Normal
      led_strip_rgb.setMode(LedStripRgbMode::NORMAL);
      led_strip_rgb.turnOn();
      break;
    }
    case 2: { // Strobe
      led_strip_rgb.setMode(LedStripRgbMode::STROBE);
      led_strip_rgb.turnOn();
      break;
    }
    case 3: { // Flash
      led_strip_rgb.setMode(LedStripRgbMode::FLASH);
      led_strip_rgb.turnOn();
      break;
    }
    case 4: { // Fade
      led_strip_rgb.setMode(LedStripRgbMode::FADE);
      led_strip_rgb.turnOn();
      break;
    }
  }
  updateWidgets();
}

BLYNK_WRITE(V8) // Switch button to V8
{
  if (param[0].asInt() == 0) {
    led_strip_w.turnOff();
  } else {
    led_strip_w.turnOn();
  }
  updateWidgets();
}

/*
 * When the mode button is pressed depending on the condition of the led strip,
 * different mode changes are made.
 *  - If the white and RGB LEDs are all off, then turn on the white LEDs.
 *  - If the RGB LEDs are off and the white LEDs are on, then turn off the white
 *    LEDs and turn on the LEDs of colors (with the last mode that has been set).
 *  - When the RGB LEDs are on and they are in Fade mode (last mode in the list),
 *    then turn off the RGB LEDs and turn on the white LEDs.
 *  - If the RGB LEDs are on and they are not in the Fade mode, then switch to
 *    the next mode in the list (NORMAL > STROBE > FLASH > FADE).
 */
void btnModeShortPressed(void)
{
  if(led_strip_w.getState() == LedStripState::OFF &&
    led_strip_rgb.getState() == LedStripState::OFF)
  {
    led_strip_w.turnOn();
  }
  else if(led_strip_w.getState() == LedStripState::ON &&
    led_strip_rgb.getState() == LedStripState::OFF)
  {
      led_strip_w.setIntensity(255);
      last_color = led_strip_rgb.getColor();
      led_strip_rgb.setColor(COLOR_WHITE);
      led_strip_rgb.setMode(LedStripRgbMode::NORMAL);
      led_strip_rgb.turnOn();
  }
  else if(led_strip_rgb.getMode() == LedStripRgbMode::NORMAL &&
    led_strip_rgb.getColor() == COLOR_WHITE)
  {
    led_strip_w.turnOff();
    led_strip_rgb.setColor(last_color);
    led_strip_rgb.turnOn();
  }
  else if(led_strip_rgb.getMode() == LedStripRgbMode::FADE)
  {
    led_strip_w.turnOn();
    led_strip_rgb.nextMode();
    led_strip_rgb.turnOff();
  }
  else
  {
    led_strip_rgb.nextMode();
  }
  updateWidgets();
}

BLYNK_WRITE(V3) // Push button to V3
{
  if(param.asInt())
  {
    btnModeShortPressed();
  }
}

/*
 * When the mode button is pressed for approximately one second, then all the
 * LEDs are turned off.
 */
void btnModeLongPressed(void)
{
  led_strip_w.turnOff();
  led_strip_rgb.turnOff();
  updateWidgets();
}

// Instance to handle button press events.
BtnHandler btn_mode(BTN_MODE_PIN, btnModeShortPressed, btnModeLongPressed);

// Function to calculate a color based on an input voltage.
uint32_t color_mixer(uint16_t input_value)
{
  int32_t red = 0;
  int32_t green = 0;
  int32_t blue = 0;
  if(input_value < 341)
  {
    input_value = (input_value * 3) / 4;
    red = 256 - input_value;
    green = input_value;
    blue = 1;
  }
  else if (input_value < 682)
  {
    input_value = ((input_value - 341) * 3) / 4;
    red = 1;
    green = 256 - input_value;
    blue = input_value;
  }
  else
  {
    input_value = ((input_value - 683) * 3) / 4;
    red = input_value;
    green = 1;
    blue = 256 - input_value;
  }

  red = (red & 0xFF) << 16;
  green = (green & 0xFF) << 8;
  blue = blue & 0xFF;
  return red + green + blue;
}

/*
 * Function to read the voltage on the analog pin and based on the operating
 * mode perform an action.
 *  - When white LEDs are on, change the brightness of them.
 *  - When the RGB leds are turned on in Normal or Strobe mode, then change
 *  the color with the help of the color_mixer function.
 *  - If the RGB LEDs are on in Flash or Fade mode, then the speed of the color
 *    sequence is changed.
 */
void readPotValue(void)
{
  uint16_t new_pot_value = analogRead(POT_COLOR_PIN);
  if((new_pot_value / 4) != last_pot_color_value)
  {
    last_pot_color_value = new_pot_value / 4;
    if(led_strip_rgb.getState() == LedStripState::ON)
    {
      LedStripRgbMode mode = led_strip_rgb.getMode();
      switch (mode) {
        case LedStripRgbMode::NORMAL:
          led_strip_rgb.setColor(color_mixer(new_pot_value));
          break;
        case LedStripRgbMode::STROBE:
          led_strip_rgb.setColor(color_mixer(new_pot_value));
          break;
        case LedStripRgbMode::FLASH:
          led_strip_rgb.setSpeed(new_pot_value);
          break;
        case LedStripRgbMode::FADE:
          led_strip_rgb.setSpeed(new_pot_value);
          break;
      }
    }
    else if(led_strip_w.getState() == LedStripState::ON)
    {
      led_strip_w.setIntensity(last_pot_color_value);
    }
    else
    {
      led_strip_w.setIntensity(last_pot_color_value);
      led_strip_w.turnOn();
      if(abs((new_pot_value / 4) - last_pot_color_value) > THRESHOLD_FOR_TURN_ON)
      {
        led_strip_w.setIntensity(last_pot_color_value);
        led_strip_w.turnOn();
      }
    }
    updateWidgets();
  }
}

/**
 * Function that allows to verify the correct operation of each one of the RGBW leds.
 */
void test_leds(void)
{
  led_strip_w.turnOn();
  led_strip_rgb.turnOff();
  led_strip_rgb.setMode(LedStripRgbMode::NORMAL);
  delay(500);
  led_strip_w.turnOff();
  led_strip_rgb.turnOn();
  delay(500);
  led_strip_rgb.setColor(COLOR_RED);
  led_strip_rgb.loop();
  delay(500);
  led_strip_rgb.setColor(COLOR_GREEN);
  led_strip_rgb.loop();
  delay(500);
  led_strip_rgb.setColor(COLOR_BLUE);
  led_strip_rgb.loop();
  delay(500);
}

void serialLoop() {
  if(Serial.available() > 0)
  {
    String command = Serial.readString();
    command.toLowerCase();
    Serial.println(command);
    if(command.startsWith("on"))
    {
      Serial.println(F("Turn on"));
      led_strip_w.turnOn();
      led_strip_rgb.turnOff();
    }
    else if(command.startsWith("off"))
    {
      Serial.println(F("Turn off"));
      btnModeLongPressed();
    }
    else if(command.startsWith("normal"))
    {
      Serial.println(F("Normal mode"));
      led_strip_rgb.setMode(LedStripRgbMode::NORMAL);
      led_strip_rgb.turnOn();
    }
    else if(command.startsWith("strobe"))
    {
      Serial.println(F("Strobe mode"));
      led_strip_rgb.setMode(LedStripRgbMode::STROBE);
      led_strip_rgb.turnOn();
    }
    else if(command.startsWith("flash"))
    {
      Serial.println(F("Flash mode"));
      led_strip_rgb.setMode(LedStripRgbMode::FLASH);
      led_strip_rgb.turnOn();
    }
    else if(command.startsWith("fade"))
    {
      Serial.println(F("Fade mode"));
      led_strip_rgb.setMode(LedStripRgbMode::FADE);
      led_strip_rgb.turnOn();
    }
    else if(command.startsWith("next"))
    {
      Serial.println(F("Next mode"));
      btnModeShortPressed();
    }
    else if(command.startsWith("color"))
    {
      command.remove(0, 7);
      uint32_t color = command.toInt();
      Serial.print(F("Set color "));
      Serial.println(color, HEX);
      led_strip_rgb.setColor(command.toInt());
      led_strip_rgb.turnOn();
    }
    else if(command.startsWith("mqttserver"))
    {
      command.remove(0, 10);
      command.trim();
      Serial.print(F("Set MQTT server "));
      Serial.println(command);
      command.toCharArray(mqtt_server, 40);
      saveConfig();
    }
    else if(command.startsWith("mqttport"))
    {
      command.remove(0, 10);
      command.trim();
      Serial.print(F("Set MQTT port "));
      Serial.println(command);
      command.toCharArray(mqtt_port, 6);
      saveConfig();
    }
    else if(command.startsWith("mqtttopic"))
    {
      command.remove(0, 11);
      command.trim();
      Serial.print(F("Set MQTT topic "));
      Serial.println(command);
      command.toCharArray(mqtt_topic, 50);
      saveConfig();
    }
    else if(command.startsWith("blynkserver"))
    {
      command.remove(0, 12);
      command.trim();
      Serial.print(F("Set Blynk Server "));
      Serial.println(command);
      command.toCharArray(blynk_server, 40);
      saveConfig();
    }
    else if(command.startsWith("blynkport"))
    {
      command.remove(0, 10);
      command.trim();
      Serial.print(F("Set Blynk Port "));
      Serial.println(command);
      command.toCharArray(blynk_port, 6);
      saveConfig();
    }
    else if(command.startsWith("token"))
    {
      command.remove(0, 7);
      command.trim();
      Serial.print(F("Set Blynk Token "));
      Serial.println(command);
      command.toCharArray(blynk_token, 34);
      saveConfig();
    }
    updateWidgets();
  }
}

/**
 * Set the pins for the LEDs and the button. For the ATTiny85 it is not
 * necessary to configure the analog input. Executes the function to verify the
 * operation of the RGBW LEDs and establishes the initial status of the LEDs
 * (white on, RGB off).
 */
void setup() {
  Serial.begin(115200);
  Serial.println();

  btn_mode.activateWith(LOW);
  btn_mode.setup();
  led_strip_w.setup();
  led_strip_rgb.setup();

  test_leds();

  led_strip_w.turnOn();
  led_strip_rgb.turnOff();
  led_strip_rgb.setColor(DEFAULT_COLOR);

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println(F("Mounting FS..."));
  mountFS();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_topic("topic", "MQTT Topic", mqtt_topic, 50);
  WiFiManagerParameter custom_blynk_server("blynk_server", "Blynk Server", blynk_server, 40);
  WiFiManagerParameter custom_blynk_port("blynk_port", "Blynk Port", blynk_port, 6);
  WiFiManagerParameter custom_blynk_token("token", "Blynk Token", blynk_token, 34);


  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_topic);
  wifiManager.addParameter(&custom_blynk_server);
  wifiManager.addParameter(&custom_blynk_port);
  wifiManager.addParameter(&custom_blynk_token);

  //reset saved settings
  //wifiManager.resetSettings();

  //set custom ip for portal
  //wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("Driver 5050", "ledstrip");
  //or use this for auto generated name ESP + ChipID
  //wifiManager.autoConnect();

  //if you get here you have connected to the WiFi
  Serial.println("Connected :)");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_topic, custom_mqtt_topic.getValue());
  strcpy(blynk_server, custom_blynk_server.getValue());
  strcpy(blynk_port, custom_blynk_port.getValue());
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    saveConfig();
  }

  Serial.println();

  mqttClient.setServer(mqtt_server, atoi(mqtt_port));
  mqttClient.setCallback(mqttCallback);

  Blynk.config(blynk_token, blynk_server, atoi(blynk_port));
  Blynk.connectWiFi(WiFi.SSID().c_str(), WiFi.psk().c_str());
  int counter = 0;
  do
  {
    Serial.print("Connecting to the Blynk Server, try number ");
    Serial.println(++counter);
    Blynk.connect();
  } while(!Blynk.connected() && counter < 4);
}

/**
 * In each iteration the voltage value in the analog input is read,
 * the button input and the RGB LEDs are updated (mainly by the Strobe, Flash
 * and Fade modes, which vary their color in time).
 */
void loop() {
  // readPotValue();
  serialLoop();
  btn_mode.loop();
  led_strip_rgb.loop();

  if (!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();
  mqttSendTele();

  Blynk.run();

  delay(50);
}
