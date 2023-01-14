#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <DNSServer.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <Wire.h>

#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "Adafruit_SGP30.h"

#include "Config.h"
#include "SerialCom.h"
#include "Types.h"

#define SPIFFS LittleFS
#define MQTT_CONNECTION_INTERVAL 60000l
#define MQTT_STATUS_PUBLISH_INTERVAL 30000l

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
Adafruit_SGP30 sgp;

WiFiClient wifiClient;
PubSubClient mqttClient;

AsyncWebServer server(80);

uint32_t lastMqttConnectionAttempt = 0;
uint32_t statusPublishPreviousMillis = 0;

char identifier[24];

#define FIRMWARE_PREFIX "esp8266-vindriktning-particle-sensor"
#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"

char MQTT_TOPIC_AVAILABILITY[128];
char MQTT_TOPIC_STATE[128];
char MQTT_TOPIC_COMMAND[128];

char MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PM25_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_AVG_TEMPERATURE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_AHT_HUMIDITY_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_SGP30_ECO2_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_SGP30_TVOC_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_BMP_TEMPERATURE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_BMP_SEA_PRESURE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_BMP_PRESURE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_BMP_ALTITUDE_SENSOR[128];
char char MQTT_TOPIC_AUTOCONF_BMP_BOILING_POINT_SENSOR[128];

bool shouldSaveConfig = false;

typedef struct Aht10SensorData
{
    float temperature;
    float humidity;
};

typedef struct Bmp280SensorData
{
    float temperature;
    float altitude;
    float pressure;
    float seaLevelPressure;
    float waterBoilingPoint
};

typedef struct Sgp30SensorData
{
    uint16_t tvoc;
    uint16_t eCo2;
};

Aht10SensorData     ahtSensorData;
Bmp280SensorData    bmpSensorData;
Sgp30SensorData     sgpSensorData;

void saveConfigCallback()
{
    shouldSaveConfig = true;
}

uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

void handleErrorOnSetup(String message)
{
    Serial.println(message);
    while (1) {}
}

void setup()
{
    Serial.begin(115200);
    SerialCom::setup();
    Wire.begin();

    if (!bmp.begin())
        handleErrorOnSetup("Could not find a valid BMP280 sensor, check wiring!");

    

    Serial.println("\n");
    Serial.println("Hello from esp8266-vindriktning-particle-sensor");
    Serial.printf("Core Version: %s\n", ESP.getCoreVersion().c_str());
    Serial.printf("Boot Version: %u\n", ESP.getBootVersion());
    Serial.printf("Boot Mode: %u\n", ESP.getBootMode());
    Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());

    snprintf(identifier, sizeof(identifier), "VINDRIKTNING-%X", ESP.getChipId());
    snprintf(MQTT_TOPIC_AVAILABILITY, 127, "%s/%s/status", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_STATE, 127, "%s/%s/state", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_COMMAND, 127, "%s/%s/command", FIRMWARE_PREFIX, identifier);

    snprintf(MQTT_TOPIC_AUTOCONF_PM25_SENSOR, 127, "homeassistant/sensor/%s/%s_pm25/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_WIFI_SENSOR, 127, "homeassistant/sensor/%s/%s_wifi/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_SCD4X_CO2_SENSOR, 127, "homeassistant/sensor/%s/%s_scd4x_co2/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_SCD4X_TEMPERATURE_SENSOR, 127, "homeassistant/sensor/%s/%s_scd4x_temperature/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_SCD4X_HUMIDITY_SENSOR, 127, "homeassistant/sensor/%s/%s_scd4x_humidity/config", FIRMWARE_PREFIX, identifier);

    DNSServer dns;
    AsyncWiFiManager wifiManager(&server, &dns);

    AsyncWiFiManagerParameter custom_mqtt_server("server", "MQTT server", "", sizeof(Config::mqtt_server));
    AsyncWiFiManagerParameter custom_mqtt_user("user", "MQTT username", "", sizeof(Config::username));
    AsyncWiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", "", sizeof(Config::password));

    WiFi.hostname(identifier);

    Config::load();

    wifiManager.setDebugOutput(false);
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);

    WiFi.hostname(identifier);
    wifiManager.autoConnect(identifier);
    mqttClient.setClient(wifiClient);

    strcpy(Config::mqtt_server, custom_mqtt_server.getValue());
    strcpy(Config::username, custom_mqtt_user.getValue());
    strcpy(Config::password, custom_mqtt_pass.getValue());

    if (shouldSaveConfig)
    {
        Config::save();
    }
    else
    {
        // For some reason, the read values get overwritten in this function
        // To combat this, we just reload the config
        // This is most likely a logic error which could be fixed otherwise
        Config::load();
    }

    setupOTA();
    mqttClient.setServer(Config::mqtt_server, 1883);
    mqttClient.setKeepAlive(10);
    mqttClient.setBufferSize(2048);
    mqttClient.setCallback(mqttCallback);

    Serial.printf("Hostname: %s\n", identifier);
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

    Serial.println("-- Current GPIO Configuration --");
    Serial.printf("PIN_UART_RX: %d\n", SerialCom::PIN_UART_RX);

    server.serveStatic("/bootstrap.min.css", LittleFS, "/bootstrap.min.css");
    server.serveStatic("/style.css", LittleFS, "/style.css");
    server.serveStatic("/bootstrap.bundle.min.js", LittleFS, "/bootstrap.bundle.min.js");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
        { request->send(LittleFS, "/index.html", String()); });
        
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        DynamicJsonDocument json(512);
        json["ssid"] = WiFi.SSID();
        json["rssi"] = WiFi.RSSI();
        json["ip"] = WiFi.localIP().toString();
        json["hostname"] = WiFi.getHostname();
        json["co2"] = scd4xCo2;
        json["humidity"] = scd4xHumidity;
        json["temperature"] = scd4xTemperature;
        json["scd4_error_message"] = scd4xErrorMessage;
        json["mqtt_is_conected"] = mqttClient.connected();
        serializeJson(json, *response);
        request->send(response); });

    server.onNotFound([](AsyncWebServerRequest *request)
        { request->send(404, "text/plain", "Page not found"); });

    server.begin();

    mqttReconnect();
}

void setupOTA()
{
    ArduinoOTA.onStart([]()
                       { Serial.println("Start"); });
    ArduinoOTA.onEnd([]()
                     { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        } });

    ArduinoOTA.setHostname(identifier);

    // This is less of a security measure and more a accidential flash prevention
    ArduinoOTA.setPassword(identifier);
    ArduinoOTA.begin();
}

bool setupScd4x()
{
    scd4x.begin(Wire);

    scd4xError = scd4x.stopPeriodicMeasurement();
    if (scd4xError)
    {
        errorToString(scd4xError, scd4xErrorMessage, 256);
        Serial.println(scd4xErrorMessage);
        return false;
    }

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    scd4xError = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (scd4xError)
    {
        errorToString(scd4xError, scd4xErrorMessage, 256);
        Serial.println(scd4xErrorMessage);
        return false;
    }
    Serial.print("SCD4X sensor serial number: ");
    printSerialNumber(serial0, serial1, serial2);

    Serial.print("SCD4X sensor self test started it will take about 10 secounds ");
    uint16_t sensorStatus;
    scd4xError = scd4x.performSelfTest(sensorStatus);
    if (scd4xError)
    {
        errorToString(scd4xError, scd4xErrorMessage, 256);
        Serial.println(scd4xErrorMessage);
        return false;
    }

    uint16_t scd4xAltitude = (uint16_t)round(bmpAltitude);
    scd4xError = scd4x.setSensorAltitude(scd4xAltitude);
    if (scd4xError)
    {
        errorToString(scd4xError, scd4xErrorMessage, 256);
        Serial.println(scd4xErrorMessage);
        return false;
    }

    scd4xError = scd4x.startLowPowerPeriodicMeasurement();
    if (scd4xError)
    {
        errorToString(scd4xError, scd4xErrorMessage, 256);
        Serial.println(scd4xErrorMessage);
        return false;
    }

    return true;
}

void loop()
{
    ArduinoOTA.handle();
    SerialCom::handleUart(state);
    mqttClient.loop();

    const uint32_t currentMillis = millis();
    if (currentMillis - statusPublishPreviousMillis >= MQTT_CONNECTION_INTERVAL)
    {
        if (state.valid && isScd4xReady())
        {
            statusPublishPreviousMillis = currentMillis;

            scd4xRead();
            printf("Publish state\n");
            publishState();
        }
    }

    if (!mqttClient.connected() && currentMillis - lastMqttConnectionAttempt >= MQTT_CONNECTION_INTERVAL)
    {
        lastMqttConnectionAttempt = currentMillis;
        printf("Reconnect mqtt\n");
        mqttReconnect();
    }
}

void mqttReconnect()
{
    for (uint8_t attempt = 0; attempt < 3; ++attempt)
    {
        if (mqttClient.connect(identifier, Config::username, Config::password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE))
        {
            mqttClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
            publishAutoConfig();

            // Make sure to subscribe after polling the status so that we never execute commands with the default data
            mqttClient.subscribe(MQTT_TOPIC_COMMAND);
            break;
        }
        delay(500);
    }
}

void readBmp()
{
    bmpTemperature = bmp.readTemperature();
    bmpPressure = bmp.readPressure();
    bmpSealevelPressure = bmp.readSealevelPressure();
    bmpAltitude = bmp.readAltitude();
}

bool isScd4xReady()
{
    bool isReady = false;
    scd4xError = scd4x.getDataReadyFlag(isReady);
    if (scd4xError)
    {
        errorToString(scd4xError, scd4xErrorMessage, 256);
        Serial.println(scd4xErrorMessage);
    }
    return isReady;
}

void scd4xRead()
{
    uint16_t tmpCo2 = 0;
    float tmpTemperature = 0.0f;
    float tmpHumidity = 0.0f;

    for (uint8_t i = 0; i < 3; i++)
    {
        scd4xError = scd4x.readMeasurement(tmpCo2, tmpTemperature, tmpHumidity);
        if (scd4xError)
        {
            errorToString(scd4xError, scd4xErrorMessage, 256);
            Serial.println(scd4xErrorMessage);
        }
        else if (tmpCo2 == 0)
        {
            Serial.println("Invalid sample detected, skipping.");
        }
        else
        {
            Serial.print("Co2:");
            Serial.print(tmpCo2);
            Serial.print("\t");
            Serial.print("Temperature:");
            Serial.print(tmpTemperature);
            Serial.print("\t");
            Serial.print("Humidity:");
            Serial.println(tmpHumidity);
            scd4xCo2 = tmpCo2;
            scd4xTemperature = tmpTemperature;
            scd4xHumidity = tmpHumidity;
            break;
        }
        delay(500);
    }
}

float getAvgTemperature()
{
    return (bmpTemperature + scd4xTemperature) / 2.0f;
}

void publishState()
{
    DynamicJsonDocument wifiJson(192);
    DynamicJsonDocument stateJson(796);
    DynamicJsonDocument scd4xJson(192);
    char payload[256];

    wifiJson["ssid"] = WiFi.SSID();
    wifiJson["ip"] = WiFi.localIP().toString();
    wifiJson["rssi"] = WiFi.RSSI();

    scd4xJson["co2"] = scd4xCo2;
    scd4xJson["temperature"] = scd4xTemperature;
    scd4xJson["humidity"] = scd4xHumidity;

    stateJson["pm25"] = state.avgPM25;
    stateJson["scd4x"] = scd4xJson.as<JsonObject>();
    stateJson["wifi"] = wifiJson.as<JsonObject>();

    serializeJson(stateJson, payload);
    mqttClient.publish(&MQTT_TOPIC_STATE[0], &payload[0], true);
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int length) {}

void publishAutoConfig()
{
    char mqttPayload[2048];
    DynamicJsonDocument device(256);
    DynamicJsonDocument autoconfPayload(1024);
    StaticJsonDocument<64> identifiersDoc;
    JsonArray identifiers = identifiersDoc.to<JsonArray>();

    identifiers.add(identifier);

    device["identifiers"] = identifiers;
    device["manufacturer"] = "Ikea";
    device["model"] = "VINDRIKTNING";
    device["name"] = identifier;
    device["sw_version"] = "2021.08.0";

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" WiFi");
    autoconfPayload["value_template"] = "{{value_json.wifi.rssi}}";
    autoconfPayload["unique_id"] = identifier + String("_wifi");
    autoconfPayload["unit_of_measurement"] = "dBm";
    autoconfPayload["json_attributes_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["json_attributes_template"] = "{\"ssid\": \"{{value_json.wifi.ssid}}\", \"ip\": \"{{value_json.wifi.ip}}\"}";
    autoconfPayload["icon"] = "mdi:wifi";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" PM 2.5");
    autoconfPayload["unit_of_measurement"] = "μg/m³";
    autoconfPayload["value_template"] = "{{value_json.pm25}}";
    autoconfPayload["unique_id"] = identifier + String("_pm25");
    autoconfPayload["icon"] = "mdi:air-filter";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PM25_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" CO₂");
    autoconfPayload["unit_of_measurement"] = "ppm";
    autoconfPayload["value_template"] = "{{value_json.scd4x.co2}}";
    autoconfPayload["unique_id"] = identifier + String("_scd4x_co2");
    autoconfPayload["icon"] = "mdi:molecule-co2";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_SCD4X_CO2_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Temperature");
    autoconfPayload["unit_of_measurement"] = "˚C";
    autoconfPayload["value_template"] = "{{(value_json.scd4x.temperature)|round(1)}}";
    autoconfPayload["unique_id"] = identifier + String("_scd4x_temperature");
    autoconfPayload["icon"] = "mdi:thermometer";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_SCD4X_TEMPERATURE_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Humidity");
    autoconfPayload["unit_of_measurement"] = "% RH";
    autoconfPayload["value_template"] = "{{(value_json.scd4x.humidity)|round(1)}}";
    autoconfPayload["unique_id"] = identifier + String("_scd4x_humidity");
    autoconfPayload["icon"] = "mdi:water-percent";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_SCD4X_HUMIDITY_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
}
