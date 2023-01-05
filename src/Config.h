#pragma once

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <FS.h>

#define CONFIG_FILE_PATH "/config.json"

namespace Config
{
    char mqtt_server[80] = "";
    char username[24] = "";
    char password[24] = "";

    void save()
    {
        DynamicJsonDocument json(512);
        json["mqtt_server"] = mqtt_server;
        json["username"] = username;
        json["password"] = password;

        File configFile = LittleFS.open(CONFIG_FILE_PATH, "w");
        if (!configFile)
        {
            Serial.println("Failed to open configurationfile for reading");
            return;
        }

        serializeJson(json, configFile);
        configFile.close();
    }

    void load()
    {
        if (!LittleFS.begin())
        {
            Serial.println("An Error has occurred while mounting LittleFS");
            return;
        }

        if (!LittleFS.exists(CONFIG_FILE_PATH))
        {
            Serial.println("Configuration file does not exist");
            return;
        }

        File configFile = LittleFS.open(CONFIG_FILE_PATH, "r");
        if (!configFile)
        {
            Serial.println("Failed to open configurationfile for reading");
            return;
        }

        const size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(512);
        DeserializationError error = deserializeJson(json, buf.get());
        if (error)
        {
            Serial.println("JSON deserialization failed");
            return;
        }

        strcpy(mqtt_server, json["mqtt_server"]);
        strcpy(username, json["username"]);
        strcpy(password, json["password"]);
    }
} // namespace Config
