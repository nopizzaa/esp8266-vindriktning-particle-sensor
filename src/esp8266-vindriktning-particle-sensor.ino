#define FIRMWARE_PREFIX                         "esp8266-vindriktning-particle-sensor"
#define FIRMWARE_VERSION                        "2023.02.17"
#define AVAILABILITY_ONLINE                     "online"
#define AVAILABILITY_OFFLINE                    "offline"

#define WEBSERVER_PORT                          80
#define MQTT_PORT                               1883
#define SGP30_DELAY                             9e5l
#define I2C_CLOCK                               400000

#define MHZ19_RX_PIN    14                                          // Rx pin which the MHZ19 Tx pin is attached to
#define MHZ19_TX_PIN    12                                          // Tx pin which the MHZ19 Rx pin is attached to
#define MHZ19_BAUDRATE  9600                                        // Device to MH-Z19 Serial baudrate (should not be changed)

#define _TASK_PRIORITY
#define _TASK_WDT_IDS
#define _TASK_TIMECRITICAL

#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <TaskScheduler.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <Adafruit_BMP280.h>
#include "SparkFun_SGP30_Arduino_Library.h"
#include <Wire.h>
#include "MHZ19.h"

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>

#include "Config.h"
#include "SerialCom.h"
#include "Types.h"

const char ERROR_SGP30_BASELINE_MESURMENT[] PROGMEM = "Failed to get baseline readings (SGP30 - CRC ERROR?)";
const char ERROR_SGP30_DELAY[] PROGMEM = "Sensor SGP30 needs least 15 minute to stabilize";

particleSensorState_t state;
vocSensorState_t vocState;
tempHumSensorState_t tempHummidityState;
pressureSensorState_t pressureState;
co2SensorState_t co2SensorState;

WiFiClient wifiClient;
PubSubClient mqttClient;

AsyncWebServer webServer(WEBSERVER_PORT);

AsyncWiFiManagerParameter custom_mqtt_server("server", "mqtt server", Config::mqtt_server, sizeof(Config::mqtt_server));
AsyncWiFiManagerParameter custom_mqtt_user("user", "MQTT username", Config::username, sizeof(Config::username));
AsyncWiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", Config::password, sizeof(Config::password));

char identifier[24];

char MQTT_TOPIC_AVAILABILITY[128];
char MQTT_TOPIC_STATE[128];
char MQTT_TOPIC_COMMAND[128];

char MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PM25_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_ECO2_SENSOR[128];  
char MQTT_TOPIC_AUTOCONF_VOC_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_PRESURE_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_CO2_SENSOR[128];  

bool shouldSaveConfig = false;

AHT20 aht;
Adafruit_BMP280 bmp;
SGP30 sgp;

MHZ19 mhz19;
SoftwareSerial mhz19Serial(MHZ19_RX_PIN, MHZ19_TX_PIN);  

Scheduler runner;
Scheduler highPriority;

void tParticleSensorLoop(void);
void tVocSensorCallback(void);
void tReadSensorCallback(void);
void tMqttClientLoopCallback(void);
void tPublishStateCallback(void);

Task tVocSensor(1000, TASK_FOREVER, &tVocSensorCallback, &runner);
Task tReadSensorsSensor(20000, TASK_FOREVER, &tReadSensorCallback, &runner);
Task tPublishState(30000, TASK_FOREVER, &tPublishStateCallback, &runner);
Task tHandleParticleSensor(1, TASK_FOREVER, &tParticleSensorLoop, &highPriority);
Task tHandleMqttClient(10, TASK_FOREVER, &tMqttClientLoopCallback, &highPriority);

void saveConfigCallback() {
    shouldSaveConfig = true;
}

void setup() {
    Serial.begin(115200);
    SerialCom::setup();

    Serial.println(F("\n"));
    Serial.println(F("Hello from esp8266-vindriktning-particle-sensor"));
    Serial.printf("Core Version: %s\n", ESP.getCoreVersion().c_str());
    Serial.printf("Boot Version: %u\n", ESP.getBootVersion());
    Serial.printf("Boot Mode: %u\n", ESP.getBootMode());
    Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());

    delay(3000);

    Wire.begin();
    Wire.setClock(I2C_CLOCK);

    if (! aht.begin()) {
        Serial.println(F("AHT20 not detected. Please check wiring."));
    } else {
        Serial.println(F("AHT20 acknowledged."));
    }

    if (! bmp.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!")); 
    } else {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,       /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,       /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,      /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,        /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_4000);   /* Standby time. */
        Serial.println(F("BMP280 acknowledged."));
    }
    
    if (! sgp.begin()) {
        Serial.println(F("No SGP30 Detected. Check connections."));
    } else {
        sgp.getSerialID();
        sgp.getFeatureSetVersion();
        Serial.print(F("SerialID: 0x"));
        Serial.print((unsigned long)sgp.serialID, HEX);
        Serial.print(F("\tFeature Set Version: 0x"));
        Serial.println(sgp.featureSetVersion, HEX);

        SGP30ERR error;
        error = sgp.measureTest();
        if (error == SGP30_SELF_TEST_FAIL) {
            Serial.println(F("SGP30 Self-Test failed"));
        }
    }

    mhz19Serial.begin(MHZ19_BAUDRATE);                    // (Uno example) device to MH-Z19 serial start
    mhz19.begin(mhz19Serial);                             // *Serial(Stream) reference must be passed to library begin().
    mhz19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))
    mhz19.setFilter();

    snprintf(identifier, sizeof(identifier), "VINDRIKTNING-%X", ESP.getChipId());
    snprintf(MQTT_TOPIC_AVAILABILITY, 127, "%s/%s/status", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_STATE, 127, "%s/%s/state", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_COMMAND, 127, "%s/%s/command", FIRMWARE_PREFIX, identifier);

    snprintf(MQTT_TOPIC_AUTOCONF_PM25_SENSOR, 127, "homeassistant/sensor/%s/%s_pm25/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_WIFI_SENSOR, 127, "homeassistant/sensor/%s/%s_wifi/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_ECO2_SENSOR, 127, "homeassistant/sensor/%s/%s_eco2/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_VOC_SENSOR, 127, "homeassistant/sensor/%s/%s_tvoc/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR, 127, "homeassistant/sensor/%s/%s_temp/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR, 127, "homeassistant/sensor/%s/%s_humidity/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_PRESURE_SENSOR, 127, "homeassistant/sensor/%s/%s_presure/config", FIRMWARE_PREFIX, identifier);
    snprintf(MQTT_TOPIC_AUTOCONF_CO2_SENSOR, 127, "homeassistant/sensor/%s/%s_co2/config", FIRMWARE_PREFIX, identifier);

    DNSServer dns;
    AsyncWiFiManager wifiManager(&webServer,&dns);

    wifiManager.setDebugOutput(false);
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);
    wifiManager.autoConnect(identifier);

    strcpy(Config::mqtt_server, custom_mqtt_server.getValue());
    strcpy(Config::username, custom_mqtt_user.getValue());
    strcpy(Config::password, custom_mqtt_pass.getValue());

    if (shouldSaveConfig) {
        Config::save();
    } else {
        // For some reason, the read values get overwritten in this function
        // To combat this, we just reload the config
        // This is most likely a logic error which could be fixed otherwise
        Config::load();
    }

    WiFi.hostname(identifier);
    
    sgp.getBaseline();
    sgp.initAirQuality();

    if (Config::sgp30ECo2Base > 0 && Config::sgp30TvocBase > 0) {
        sgp.setBaseline(Config::sgp30ECo2Base, Config::sgp30TvocBase);
        Serial.println(F("SGP30 baseline restored"));
    }

    setupOTA();

    mqttClient.setClient(wifiClient);
    mqttClient.setServer(Config::mqtt_server, MQTT_PORT);
    mqttClient.setKeepAlive(10);
    mqttClient.setBufferSize(2048);
    mqttClient.setCallback(mqttCallback);

    tHandleMqttClient.setId(50);
    tHandleParticleSensor.setId(60);

    runner.setHighPriorityScheduler(&highPriority);

    webServer.serveStatic("/bootstrap.min.css", LittleFS, "/bootstrap.min.css");
    webServer.serveStatic("/style.css", LittleFS, "/style.css");
    webServer.serveStatic("/bootstrap.bundle.min.js", LittleFS, "/bootstrap.bundle.min.js");

    webServer.onNotFound([](AsyncWebServerRequest *request) { 
        request->send(404, "text/plain", F("Page not found"));
    });

    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { 
        request->send(LittleFS, "/index.html", String());
    });

    webServer.on("/api/voc-sensor/baseline", HTTP_GET, [](AsyncWebServerRequest *request) {
        SGP30ERR error;
        error = sgp.getBaseline();

        if (error != SGP30_SUCCESS) {
            Serial.println(ERROR_SGP30_BASELINE_MESURMENT);
            request->send(500, "text/plain", ERROR_SGP30_BASELINE_MESURMENT);
            return;
        }
        
        Serial.print(F("\n****Baseline values: eCO2: 0x")); Serial.print(sgp.baselineCO2, HEX);
        Serial.print(F(" & TVOC: 0x")); Serial.println(sgp.baselineTVOC, HEX);

        AsyncResponseStream *response = request->beginResponseStream("application/json");

        DynamicJsonDocument json(128);

        json["current_eco2_base_hex"] = sgp.baselineCO2;
        json["current_tvoc_base_hex"] = sgp.baselineTVOC;
        json["init_eco2_base_hex"] = Config::sgp30ECo2Base;
        json["init_tvoc_base_hex"] = Config::sgp30TvocBase;

        serializeJson(json, *response);
        request->send(response);
    });

    webServer.on("/api/voc-sensor/save-baseline", HTTP_GET, [](AsyncWebServerRequest *request) {
        if(millis() <= SGP30_DELAY) {
            Serial.println(ERROR_SGP30_DELAY);
            request->send(500, "text/plain", ERROR_SGP30_DELAY);
            return;
        }

        runner.disableAll(true);

        SGP30ERR error;
        error = sgp.getBaseline();

        if (error != SGP30_SUCCESS) {
            Serial.println(ERROR_SGP30_BASELINE_MESURMENT);
            request->send(500, "text/plain", ERROR_SGP30_BASELINE_MESURMENT);
            return;
        }

        Config::sgp30ECo2Base = sgp.baselineCO2;
        Config::sgp30TvocBase = sgp.baselineTVOC;
        Config::save();

        runner.enableAll(true);

        AsyncResponseStream *response = request->beginResponseStream("application/json");
        DynamicJsonDocument json(256);

        Serial.print(F("\n****Baseline values: eCO2: 0x")); Serial.print(sgp.baselineCO2, HEX);
        Serial.print(F(" & TVOC: 0x")); Serial.println(sgp.baselineTVOC, HEX);

        json["current_eco2_base_hex"] = sgp.baselineCO2;
        json["current_tvoc_base_hex"] = sgp.baselineTVOC;

        serializeJson(json, *response);
        request->send(response);
    });

    webServer.begin();

    Serial.printf("Hostname: %s\n", identifier);
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

    Serial.println(F("-- Current PM 2.5 Sensor GPIO Configuration --"));
    Serial.printf("PIN_UART_RX: %d\n", SerialCom::PIN_UART_RX);

    runner.enableAll(true);
}

void setupOTA() {
    ArduinoOTA.onStart([]() { 
        Serial.println(F("Start"));
    });
    ArduinoOTA.onEnd([]() { 
        Serial.println(F("\nEnd"));
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println(F("Auth Failed"));
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println(F("Begin Failed"));
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println(F("Connect Failed"));
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println(F("Receive Failed"));
        } else if (error == OTA_END_ERROR) {
            Serial.println(F("End Failed"));
        }
    });

    ArduinoOTA.setHostname(identifier);

    // This is less of a security measure and more a accidential flash prevention
    ArduinoOTA.setPassword(identifier);
    ArduinoOTA.begin();
}

void loop() {
    ArduinoOTA.handle();
    runner.execute();
}

double RHtoAbsolute (float relHumidity, float tempC) {
    const double eSat = 6.11 * pow(10.0, (7.5 * tempC / (237.7 + tempC)));
    const double vaporPressure = (relHumidity * eSat) / 100; //millibars
    const double absHumidity = 1000 * vaporPressure * 100 / ((tempC + 273) * 461.5); //Ideal gas law with unit conversions
    return absHumidity;
}

uint16_t doubleToFixedPoint(double number) {
    const int power = 1 << 8;
    const double number2 = number * power;
    const uint16_t value = floor(number2 + 0.5);
    return value;
}

void tReadSensorCallback(void) {
    if (aht.available()) {
        tempHummidityState.temperature = aht.getTemperature();
        tempHummidityState.humidity = aht.getHumidity();
        tempHummidityState.isCalibrated = aht.isCalibrated();

        Serial.printf("New AHT20 values: Temperature: %f C, Humidity: %f RHT\n", 
            tempHummidityState.temperature, tempHummidityState.humidity);

        const double absHumidity = RHtoAbsolute(tempHummidityState.humidity, tempHummidityState.temperature);
        //Convert the double type humidity to a fixed point 8.8bit number
        const uint16_t sensHumidity = doubleToFixedPoint(absHumidity);
        sgp.setHumidity(sensHumidity);
    }

    pressureState.temperature = bmp.readTemperature();
    pressureState.pressure = bmp.readPressure();

    Serial.printf("New BMP280 values: Pressure: %f Pa, Temperature: %fC\n", 
        pressureState.pressure, pressureState.temperature);

    int co2 = mhz19.getCO2();
    int8_t temperature = mhz19.getTemperature();

    for (uint8_t i = 0; i < 3; ++i) {
        if (co2 != 0) {
            co2SensorState.co2 = co2;
            co2SensorState.temperature = temperature;
            Serial.printf("New MHZ-19X values: CO2: %d ppm, Temperature: %dC\n", 
                co2SensorState.co2, co2SensorState.temperature);
            break;
        } else {
            Serial.println(F("*** MHZ-19X filter was triggered ***"));
            co2 = mhz19.getCO2();
            temperature = mhz19.getTemperature();
        }
    }
}

void tVocSensorCallback(void) {
    SGP30ERR error;
    error = sgp.measureAirQuality();

    if (error == SGP30_ERR_BAD_CRC) {
        Serial.println(F("SGP30 - CRC Failed"));
    } else if (error == SGP30_ERR_I2C_TIMEOUT) {
        Serial.println(F("SGP30 - I2C Timed out"));
    } else {
        vocState.sumECO2 += sgp.CO2;
        vocState.sumTVOC += sgp.TVOC;
    
        vocState.measurementIdx = (vocState.measurementIdx + 1) % 30;

        if (vocState.measurementIdx == 0) {
            float avgECO2 = 0.0f;
            float avgTVOC = 0.0f;

            avgECO2 = vocState.sumECO2 / 30.0f;
            avgTVOC = vocState.sumTVOC / 30.0f;

            vocState.avgECO2 = avgECO2;
            vocState.avgTVOC = avgTVOC;
            vocState.sumECO2 = 0;
            vocState.sumTVOC = 0;

            vocState.valid = true;
            Serial.printf("New SGP30 values: eCO2: %d ppm, TVOC: %d ppb\n", vocState.avgECO2, vocState.avgTVOC);
        }
    }
}

void tParticleSensorLoop(void) {
    SerialCom::handleUart(state);
}

void tMqttClientLoopCallback(void) {
    mqttClient.loop();
}

void tPublishStateCallback(void) {
    if (mqttClient.connected()) {
        if (state.valid && vocState.valid) {
            printf("Publish state via MQTT\n");
            publishState();
        }
    } 
    else {
        printf("Connecting to MQTT broker\n");
        if (mqttClient.connect(identifier, Config::username, Config::password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE)) {
            mqttClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
            publishAutoConfig();

            // Make sure to subscribe after polling the status so that we never execute commands with the default data
            mqttClient.subscribe(MQTT_TOPIC_COMMAND);
        }
    }
}

void publishState() {
    DynamicJsonDocument vocJson(192);
    DynamicJsonDocument wifiJson(192);
    DynamicJsonDocument humidityJson(192);
    DynamicJsonDocument pressureJson(192);
    DynamicJsonDocument co2Json(192);

    DynamicJsonDocument stateJson(1024);
    char payload[1024];

    vocJson["eco2"] = vocState.avgECO2;
    vocJson["tvoc"] = vocState.avgTVOC;

    humidityJson["humidity"] = tempHummidityState.humidity;
    humidityJson["temp"] = tempHummidityState.temperature;
    humidityJson["is_calibrated"] = tempHummidityState.isCalibrated;

    pressureJson["temp"] = pressureState.temperature;
    pressureJson["pressure"] = pressureState.pressure;

    wifiJson["ssid"] = WiFi.SSID();
    wifiJson["ip"] = WiFi.localIP().toString();
    wifiJson["rssi"] = WiFi.RSSI();

    co2Json["co2"] = co2SensorState.co2;
    co2Json["temp"] = co2SensorState.temperature;

    stateJson["pm25"] = state.avgPM25;

    stateJson["uptime"] = (uint32_t) millis() / 60000;

    stateJson["humidity_sensor"] = humidityJson.as<JsonObject>();
    stateJson["pressure_sensor"] = pressureJson.as<JsonObject>();
    stateJson["voc_sensor"] = vocJson.as<JsonObject>();
    stateJson["co2_sensor"] = co2Json.as<JsonObject>();
    stateJson["wifi"] = wifiJson.as<JsonObject>();

    serializeJson(stateJson, payload);
    mqttClient.publish(&MQTT_TOPIC_STATE[0], &payload[0], true);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) { }

void publishAutoConfig() {
    char mqttPayload[2048];
    DynamicJsonDocument device(256);
    DynamicJsonDocument autoconfPayload(1024);
    StaticJsonDocument<64> identifiersDoc;
    JsonArray identifiers = identifiersDoc.to<JsonArray>();

    identifiers.add(identifier);

    device["identifiers"] = identifiers;
    device["manufacturer"] = F("IKEA");
    device["model"] = F("VINDRIKTNING");
    device["name"] = identifier;
    device["sw_version"] = FIRMWARE_VERSION;

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" WiFi");
    autoconfPayload["value_template"] = F("{{value_json.wifi.rssi}}");
    autoconfPayload["unique_id"] = identifier + String("_wifi");
    autoconfPayload["unit_of_measurement"] = F("dBm");
    autoconfPayload["json_attributes_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["json_attributes_template"] = F("{\"ssid\": \"{{value_json.wifi.ssid}}\", \"ip\": \"{{value_json.wifi.ip}}\"}");
    autoconfPayload["icon"] = F("mdi:wifi");

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" PM 2.5");
    autoconfPayload["unit_of_measurement"] = F("μg/m³");
    autoconfPayload["value_template"] = F("{{value_json.pm25}}");
    autoconfPayload["unique_id"] = identifier + String("_pm25");
    autoconfPayload["icon"] = F("mdi:air-filter");

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PM25_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" eCO₂");
    autoconfPayload["unit_of_measurement"] = F("ppm");
    autoconfPayload["value_template"] = F("{{value_json.voc_sensor.eco2}}");
    autoconfPayload["unique_id"] = identifier + String("_voc_sensor_eco2");
    autoconfPayload["icon"] = F("mdi:molecule-co2");

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_ECO2_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" VOC");
    autoconfPayload["unit_of_measurement"] = F("ppb");
    autoconfPayload["value_template"] = F("{{value_json.voc_sensor.tvoc}}");
    autoconfPayload["unique_id"] = identifier + String("_voc_sensor_tvoc");
    autoconfPayload["icon"] = F("mdi:molecule");

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_VOC_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Temperature");
    autoconfPayload["unit_of_measurement"] = F("˚C");
    autoconfPayload["value_template"] = F("{{(value_json.humidity_sensor.temp)|round(1)}}");
    autoconfPayload["unique_id"] = identifier + String("_temp");
    autoconfPayload["icon"] = F("mdi:thermometer");

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Humidity");
    autoconfPayload["unit_of_measurement"] = F("% RH");
    autoconfPayload["value_template"] = F("{{(value_json.humidity_sensor.humidity)|round(1)}}");
    autoconfPayload["unique_id"] = identifier + String("_humidity");
    autoconfPayload["icon"] = F("mdi:water-percent");

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" Pressure");
    autoconfPayload["unit_of_measurement"] = F("hPa");
    autoconfPayload["value_template"] = F("{{((value_json.pressure_sensor.pressure)|float/100)|round(1)}}");
    autoconfPayload["unique_id"] = identifier + String("_pressure");
    autoconfPayload["icon"] = F("mdi:gauge");

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_PRESURE_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();

    autoconfPayload["device"] = device.as<JsonObject>();
    autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
    autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
    autoconfPayload["name"] = identifier + String(" CO₂");
    autoconfPayload["unit_of_measurement"] = F("ppm");
    autoconfPayload["value_template"] = F("{{value_json.co2_sensor.co2}}");
    autoconfPayload["unique_id"] = identifier + String("_co2_sensor_co2");
    autoconfPayload["icon"] = F("mdi:molecule-co2");

    serializeJson(autoconfPayload, mqttPayload);
    mqttClient.publish(&MQTT_TOPIC_AUTOCONF_CO2_SENSOR[0], &mqttPayload[0], true);

    autoconfPayload.clear();
}
