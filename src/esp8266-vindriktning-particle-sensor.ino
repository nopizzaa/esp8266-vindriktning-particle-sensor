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

#define SPIFFS                          LittleFS
#define MQTT_CONNECTION_INTERVAL        60000l
#define MQTT_STATUS_PUBLISH_INTERVAL    30000l

// #define BMP_AHT_BOARD_PRESENT

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
char MQTT_TOPIC_AUTOCONF_BMP_PRESURE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_BMP_ALTITUDE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_BMP_BOILING_POINT_SENSOR[128];

bool shouldSaveConfig = false;

particleSensorState_t pmSensorState;
aht10SensorState_t ahtSensorState;
bmp280SensorState_t bmpSensorState;
sgp30SensorState_t sgpSensorState;

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

float getAvgTemperature(float sensor1, float sensor2)
{
    return (sensor1 + sensor2) / 2;
}

void handleErrorOnSetup(String message)
{
    Serial.println(message);
    // ToDo: turn builtin led
    while (1) delay(10);
}

void setup()
{
    Serial.begin(115200);
    SerialCom::setup();
    Wire.begin();

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

    snprintf(MQTT_TOPIC_AUTOCONF_WIFI_SENSOR, 127, "homeassistant/sensor/%s/%s_wifi/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_PM25_SENSOR, 127, "homeassistant/sensor/%s/%s_pm25/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_AVG_TEMPERATURE_SENSOR, 127, "homeassistant/sensor/%s/%s_avg_temperature/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_AHT_HUMIDITY_SENSOR, 127, "homeassistant/sensor/%s/%s_aht_humidity/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_SGP30_ECO2_SENSOR, 127, "homeassistant/sensor/%s/%s_sgp30_eco2/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_SGP30_TVOC_SENSOR, 127, "homeassistant/sensor/%s/%s_sgp30_tvoc/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_BMP_PRESURE_SENSOR, 127, "homeassistant/sensor/%s/%s_bmp_presure/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_BMP_ALTITUDE_SENSOR, 127, "homeassistant/sensor/%s/%s_bmp_altitude/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_BMP_BOILING_POINT_SENSOR, 127, "homeassistant/sensor/%s/%s_bmp_boiling_point/config", FIRMWARE_PREFIX, identifier);

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

    #ifdef BMP_AHT_BOARD_PRESENT
        if (!aht.begin())    
            handleErrorOnSetup("Could not find AHT?. Check wiring!");
        Serial.println("AHT10 or AHT20 found");

        if (!bmp.begin())
        {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
            Serial.print("SensorID was: 0x");
            Serial.println(bmp.sensorID(),16);
            Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
            Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
            Serial.print("        ID of 0x60 represents a BME 280.\n");
            Serial.print("        ID of 0x61 represents a BME 680.\n");
            handleErrorOnSetup("Could not find BMP280. Check wiring!");
        }
        Serial.println("BMP280 found");
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,       /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X16,      /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,      /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,        /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_4000);  /* Standby time. */
    #endif

    if (!sgp.begin())
        handleErrorOnSetup("Could not find SPG30. Check wiring!");
    Serial.print("SGP30 sensor serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);

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
    { 
        request->send(LittleFS, "/index.html", String());
    });
        
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        AsyncResponseStream *response = request->beginResponseStream("application/json");

        DynamicJsonDocument wifiJson(192);
        DynamicJsonDocument stateJson(1024);
        DynamicJsonDocument sgp30Json(128);
        DynamicJsonDocument bmpJson(512);
        DynamicJsonDocument ahtJson(128);

        wifiJson["ssid"] = WiFi.SSID();
        wifiJson["ip"] = WiFi.localIP().toString();
        wifiJson["rssi"] = WiFi.RSSI();

        sgp30Json["eco2"] = sgpSensorState.eCo2;
        sgp30Json["tvoc"] = sgpSensorState.tvoc;

        ahtJson["temperature"] = ahtSensorState.temperature;
        ahtJson["humidity"] = ahtSensorState.humidity;

        bmpJson["temperature"] = bmpSensorState.temperature;
        bmpJson["altitude"] = bmpSensorState.altitude;
        bmpJson["pressure"] = bmpSensorState.pressure;
        bmpJson["water_boiling_point"] = bmpSensorState.waterBoilingPoint;

        stateJson["pm25"] = pmSensorState.avgPM25;
        stateJson["temperature"] = getAvgTemperature(ahtSensorState.temperature, bmpSensorState.temperature);

        stateJson["sgp30"] = sgp30Json.as<JsonObject>();
        stateJson["bmp"] = bmpJson.as<JsonObject>();
        stateJson["aht"] = ahtJson.as<JsonObject>();
        stateJson["wifi"] = wifiJson.as<JsonObject>();

        serializeJson(stateJson, *response);
        request->send(response);
    });

    server.onNotFound([](AsyncWebServerRequest *request)
    { 
        request->send(404, "text/plain", "Page not found");
    });

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

void getSensorData()
{
    #ifdef BMP_AHT_BOARD_PRESENT
        sensors_event_t ahtHumidity, ahtTemp;
        aht.getEvent(&ahtHumidity, &ahtTemp);
        ahtSensorState.humidity = ahtHumidity.relative_humidity;
        ahtSensorState.temperature = ahtTemp.temperature;

        bmpSensorState.temperature = bmp.readTemperature();
        bmpSensorState.pressure = bmp.readPressure();
        bmpSensorState.altitude = bmp.readAltitude();
        bmpSensorState.waterBoilingPoint = bmp.waterBoilingPoint(bmpSensorState.pressure / 100);

        const float temp = getAvgTemperature(ahtSensorState.temperature, bmpSensorState.temperature);
        sgp.setHumidity(getAbsoluteHumidity(temp, ahtSensorState.humidity));
    #endif

    if (! sgp.IAQmeasure()) {
        Serial.println("Measurement failed");
        return;
    }

    sgpSensorState.tvoc = sgp.TVOC;
    sgpSensorState.eCo2 = sgp.eCO2;
}

void loop()
{
    ArduinoOTA.handle();
    SerialCom::handleUart(pmSensorState);
    mqttClient.loop();

    const uint32_t currentMillis = millis();
    if (currentMillis - statusPublishPreviousMillis >= MQTT_CONNECTION_INTERVAL)
    {
        if (pmSensorState.valid)
        {
            statusPublishPreviousMillis = currentMillis;

            getSensorData();

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

void publishState()
{
    DynamicJsonDocument wifiJson(192);
    DynamicJsonDocument stateJson(1024);
    DynamicJsonDocument sgp30Json(128);
    DynamicJsonDocument bmpJson(512);
    DynamicJsonDocument ahtJson(128);

    wifiJson["ssid"] = WiFi.SSID();
    wifiJson["ip"] = WiFi.localIP().toString();
    wifiJson["rssi"] = WiFi.RSSI();

    sgp30Json["eco2"] = sgpSensorState.eCo2;
    sgp30Json["tvoc"] = sgpSensorState.tvoc;

    ahtJson["temperature"] = ahtSensorState.temperature;
    ahtJson["humidity"] = ahtSensorState.humidity;

    bmpJson["temperature"] = bmpSensorState.temperature;
    bmpJson["altitude"] = bmpSensorState.altitude;
    bmpJson["pressure"] = bmpSensorState.pressure;
    bmpJson["water_boiling_point"] = bmpSensorState.waterBoilingPoint;

    stateJson["pm25"] = pmSensorState.avgPM25;
    stateJson["temperature"] = getAvgTemperature(ahtSensorState.temperature, bmpSensorState.temperature);

    stateJson["sgp30"] = sgp30Json.as<JsonObject>();
    stateJson["bmp"] = bmpJson.as<JsonObject>();
    stateJson["aht"] = ahtJson.as<JsonObject>();
    stateJson["wifi"] = wifiJson.as<JsonObject>();

    char payload[512];
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
    device["manufacturer"] = "IKEA";
    device["model"] = "VINDRIKTNING";
    device["name"] = identifier;
    device["sw_version"] = "2023.01.19";

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
    autoconfPayload["name"] = identifier + String(" eCO₂");
    autoconfPayload["unit_of_measurement"] = "ppm";
    autoconfPayload["value_template"] = "{{value_json.sgp30.eco2}}";
    autoconfPayload["unique_id"] = identifier + String("_sgp30_eco2");
    autoconfPayload["icon"] = "mdi:molecule-co2";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_SGP30_ECO2_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" VOC");
    autoconfPayload["unit_of_measurement"] = "ppb";
    autoconfPayload["value_template"] = "{{value_json.sgp30.tvoc}}";
    autoconfPayload["unique_id"] = identifier + String("_sgp30_tvoc");
    autoconfPayload["icon"] = "mdi:molecule";

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_SGP30_TVOC_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    #ifdef BMP_AHT_BOARD_PRESENT

        autoconfPayload["device"] = device.as<JsonObject>();
        autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
        autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
        autoconfPayload["name"] = identifier + String(" Temperature");
        autoconfPayload["unit_of_measurement"] = "˚C";
        autoconfPayload["value_template"] = "{{(value_json.temperature)|round(2)}}";
        autoconfPayload["unique_id"] = identifier + String("_temperature");
        autoconfPayload["icon"] = "mdi:thermometer";

        serializeJson(autoconfPayload, mqttPayload);
        mqttClient.publish(&MQTT_TOPIC_AUTOCONF_AVG_TEMPERATURE_SENSOR[0], &mqttPayload[0], true);

        autoconfPayload.clear();

        autoconfPayload["device"] = device.as<JsonObject>();
        autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
        autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
        autoconfPayload["name"] = identifier + String(" Humidity");
        autoconfPayload["unit_of_measurement"] = "% RH";
        autoconfPayload["value_template"] = "{{(value_json.aht.humidity)|round(1)}}";
        autoconfPayload["unique_id"] = identifier + String("_aht_humidity");
        autoconfPayload["icon"] = "mdi:water-percent";

        serializeJson(autoconfPayload, mqttPayload);
        mqttClient.publish(&MQTT_TOPIC_AUTOCONF_AHT_HUMIDITY_SENSOR[0], &mqttPayload[0], true);

        autoconfPayload.clear();

        autoconfPayload["device"] = device.as<JsonObject>();
        autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
        autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
        autoconfPayload["name"] = identifier + String(" Pressure");
        autoconfPayload["unit_of_measurement"] = "hPa";
        autoconfPayload["value_template"] = "{{((value_json.bmp.pressure)|float/100)|round(1)}}";
        autoconfPayload["unique_id"] = identifier + String("_bmp_pressure");
        autoconfPayload["icon"] = "mdi:weather-windy-variant";

        serializeJson(autoconfPayload, mqttPayload);
        mqttClient.publish(&MQTT_TOPIC_AUTOCONF_BMP_PRESURE_SENSOR[0], &mqttPayload[0], true);

        autoconfPayload.clear();

        autoconfPayload["device"] = device.as<JsonObject>();
        autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
        autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
        autoconfPayload["name"] = identifier + String(" Altitude");
        autoconfPayload["unit_of_measurement"] = "m";
        autoconfPayload["value_template"] = "{{(value_json.bmp.altitude)|round(1)}}";
        autoconfPayload["unique_id"] = identifier + String("_bmp_altitude");
        autoconfPayload["icon"] = "mdi:image-filter-hdr";

        serializeJson(autoconfPayload, mqttPayload);
        mqttClient.publish(&MQTT_TOPIC_AUTOCONF_BMP_ALTITUDE_SENSOR[0], &mqttPayload[0], true);

        autoconfPayload.clear();

        autoconfPayload["device"] = device.as<JsonObject>();
        autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
        autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
        autoconfPayload["name"] = identifier + String(" Boiling point");
        autoconfPayload["unit_of_measurement"] = "°C";
        autoconfPayload["value_template"] = "{{(value_json.bmp.water_boiling_point)|round(1)}}";
        autoconfPayload["unique_id"] = identifier + String("_bmp_water_boiling_point");
        autoconfPayload["icon"] = "mdi:pot-steam-outline";

        serializeJson(autoconfPayload, mqttPayload);
        mqttClient.publish(&MQTT_TOPIC_AUTOCONF_BMP_BOILING_POINT_SENSOR[0], &mqttPayload[0], true);

        autoconfPayload.clear();
    #endif
}
