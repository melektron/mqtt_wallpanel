# Disclaimer

This is a PlatformIO "port" of an old of an old ESP8266 Arduino project that I did several years ago (melektron/mqtt_sockets), that was then adjusted for new functionality of a control panel.
This code is by no means clean or memory safe and there are no guarantees of it working as expected. The only reason this old project is published in this state is so I can preserve it and fix bugs that arise over time as I need this in daily production. It has been working mostly reliably but use it at your own risk. 


# MQTT Wallpanel

This project allows an ESP8266 connected to an MQTT server through WiFi and send JSON commands when buttons are pressed and potentiometers are turned. This adheres to the standard JSON protocol used by SmartHome systems like HomeAssistant or zigbee2mqtt. Therefore, this can be used to directly control e.g. lights.

## Setup

To use the project, you need to specify WIFI secrets, fingerprints and other configuration in the ```esp_controller/include/secrets.h``` file. Example content:

```c
#pragma once

#define WIFI_SSID "MyHomeWifi"
#define WIFI_PSK "MyWiFiPassword"

#define MQTT_SERVER "192.168.1.13"
#define MQTT_PORT 1883
// generate the fingerprint on the server using this command:
// echo | openssl s_client -connect localhost:1883 | openssl x509 -fingerprint -noout
#define MQTT_FINGERPRINT "07 ED 59 67 2C CD 31 6C EF 27 F7 AC 38 02 07 9C 60 EC FA 0A" // this isn't used at the moment because it didn't work
#define MQTT_BASE_TOPIC "/devices/sub0/espdev2051a/"
#define MQTT_USER "deviceuser"
#define MQTT_PWD  "devicepwd"

// when using DHCP do the following
#define WIFI_DHCP

// when using static IP do the following (replace with your correct IP settings)
// when WIFI_DHCP is defined, these values are ignored
#define WIFI_STATIC_IP 192,168,1,122
#define WIFI_STATIC_ROUTER 192,168,1,1
#define WIFI_STATIC_NETMASK 255,255,255,0

```

Of course, all the information has to be replaced by data applicable to your network setup.

Note to myself:
The values needed for my setup are found in the infrastructure setup notes.

