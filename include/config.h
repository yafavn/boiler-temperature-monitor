
#ifndef CONFIG_H
#define CONFIG_H

#define TELNET_PORT 23

// WiFi Settings
#define WIFI_SSID "madmax"
#define WIFI_PASS "9093668a"

// MQTT Settings
#define MQTT_BROKER_IP "192.168.6.234" // רק ה-IP, בלי //:mqtt
#define MQTT_PORT 1883                 // הפורט הרגיל של Mosquitto הוא 1883
#define MQTT_USER "mqttuser"
#define MQTT_PASSWORD "9093668a"

#endif // CONFIG_H