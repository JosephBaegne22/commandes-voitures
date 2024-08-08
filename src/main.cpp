#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "esp_camera.h"
#include "Freenove_4WD_Car_WiFi.h"
#include "Freenove_4WD_Car_Emotion.h"
#include "Freenove_4WD_Car_WS2812.h"
#include "Freenove_4WD_Car_For_ESP32.h"
#include <PubSubClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <limits.h>

#define STREAM_CONTENT_BOUNDARY "123456789000000000000987654321"

// Command IDs
#define CMD_FORWARD 10
#define CMD_BACKWARD 11
#define CMD_LEFT 12
#define CMD_RIGHT 13

// Ressources
// https://randomnerdtutorials.com/esp32-websocket-server-arduino/#1
// https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
// https://randomnerdtutorials.com/esp32-static-fixed-ip-address-arduino-ide/
// https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32/tree/master

char *ssid_wifi = "xxxxx";            // Le nom du réseau WiFi
char *password_wifi = "xxxxx";        // Le password du WiFi

const char *mqtt_server = "192.168.64.40"; // L'IP de votre broker MQTT
const int mqtt_interval_ms = 5000;          // L'interval en ms entre deux envois de données

IPAddress localIP(192, 168, 64, 50); // l'IP que vous voulez donner à votre voiture
IPAddress localGateway(192, 168, 64, 136); // L'IP de la gateway de votre réseau
IPAddress localSubnet(255, 255, 255, 0);  // Le masque de sous réseau
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // Changez le nom de ce point d'accès pour "sécuriser" l'accès à votre voiture

WiFiClient espClient;
PubSubClient client(espClient);

WiFiServer server_Camera(7000);
bool videoFlag = 0;

long last_message = 0;

int distance[4];          // Storage of ultrasonic data
int sensor_v;             // Int cast of track sensor data
char buff[6];             // Buffer to store the battery voltage data
char ultrasonic_buff[10]; // Buffer to store the Ultrasonic data

// put function declarations here:
void WiFi_Init();
void loopTask_Camera(void *pvParameters);
void notifyClients();
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSocket();
void reconnect();

void WiFi_Init()
{
    ssid_Router = ssid_wifi;         // Modify according to your router name
    password_Router = password_wifi; // Modify according to your router password
    ssid_AP = "Sunshine";            // ESP32 turns on an AP and calls it Sunshine
    password_AP = "Sunshine";        // Set your AP password for ESP32 to Sunshine
    frame_size = FRAMESIZE_CIF;      // 400*296
}

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    if (!WiFi.config(localIP, localGateway, localSubnet, primaryDNS, secondaryDNS))
    {
        Serial.println("STA Failed to configure");
    }

    Buzzer_Setup(); // Buzzer initialization
    WiFi_Init();    // WiFi parameters initialization
    WiFi_Setup(0);  // Start AP Mode. If you want to connect to a router, change 1 to 0.
    server_Camera.begin(7000); // Turn on the camera server

    cameraSetup(); // Camera initialization
    camera_vflip(true);
    camera_hmirror(true);
    Emotion_Setup();    // Emotion initialization
    WS2812_Setup();     // WS2812 initialization
    PCA9685_Setup();    // PCA9685 initialization
    Light_Setup();      // Light initialization
    Track_Setup();      // Track initialization
    Ultrasonic_Setup(); // Initialize the ultrasonic module

    disableCore0WDT(); // Turn off the watchdog function in kernel 0
    xTaskCreateUniversal(loopTask_Camera, "loopTask_Camera", 8192, NULL, 0, NULL, 0);
    xTaskCreateUniversal(loopTask_WTD, "loopTask_WTD", 8192, NULL, 0, NULL, 0);

    client.setServer(mqtt_server, 1883);

    initWebSocket();

    server.begin();

    Emotion_SetMode(1);
    WS2812_SetMode(1);
}

void loop()
{
    ws.cleanupClients();

    Emotion_Show(emotion_task_mode); // Led matrix display function
    WS2812_Show(ws2812_task_mode);   // Car color lights display function

    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    long now = millis();
    if (now - last_message > mqtt_interval_ms)
    {
        last_message = now;

        Track_Read();
        sensor_v = static_cast<int>(sensorValue[3]);
        char const *n_char = std::to_string(sensor_v).c_str();
        client.publish("esp32/track", n_char);

        dtostrf(Get_Sonar(), 5, 2, ultrasonic_buff);
        client.publish("esp32/sonar", ultrasonic_buff);

        dtostrf(Get_Photosensitive(), 5, 2, ultrasonic_buff);
        client.publish("esp32/light", ultrasonic_buff);
    }
}

void notifyClients()
{
    ws.textAll("ok");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;

    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
        data[len] = 0;

        StaticJsonDocument<200> doc;

        DeserializationError error = deserializeJson(doc, (char *)data);

        if (error)
        {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            return;
        }

        int cmd = doc["cmd"];

        if (1 == cmd)
        {
            JsonArray data = doc["data"];
            int data_0 = data[0];
            int data_1 = data[1];
            int data_2 = data[2];
            int data_3 = data[3];

            Motor_Move(data_0, data_1, data_2, data_3);
        }
        else if (2 == cmd)
        {
            int data = doc["data"];
            Emotion_SetMode(data);
        }
        else if (3 == cmd)
        {
            JsonArray angles = doc["data"];
            int angle_0 = angles[0];
            int angle_1 = angles[1];
            Servo_1_Angle(angle_0); // Set the Angle value of servo 1 to 0 to 180°
            Servo_2_Angle(angle_1);
        }
        else if (4 == cmd)
        {
            int led_mode = doc["data"];
            WS2812_SetMode(led_mode);
        }
        else if (5 == cmd)
        {
            JsonArray led_color = doc["data"];
            int led_color_0 = led_color[0];
            int led_color_1 = led_color[1];
            int led_color_2 = led_color[2];
            int led_color_3 = led_color[3];

            WS2812_Set_Color_1(led_color_0, led_color_1, led_color_2, led_color_3);
        }
        else if (6 == cmd)
        {
            JsonArray led_color_2 = doc["data"];
            int led_color_2_0 = led_color_2[0];
            int led_color_2_1 = led_color_2[1];
            int led_color_2_2 = led_color_2[2];
            int led_color_2_3 = led_color_2[3];

            WS2812_Set_Color_2(led_color_2_0, led_color_2_1, led_color_2_2, led_color_2_3);
        }
        else if (7 == cmd)
        {
            bool alarm = doc["data"] == 1;
            Buzzer_Alarm(alarm);
        }
        else if (8 == cmd)
        {
            JsonArray buzzer_data = doc["data"];
            int alarm_on = buzzer_data[0] == 1;
            int frequency_hz = buzzer_data[1];
            Buzzer_Variable(alarm_on, frequency_hz);
        }
        else if (9 == cmd)
        {
            bool video_activation = doc["data"] == 1;
            videoFlag = video_activation;
        }
        else if (CMD_FORWARD == cmd)
        {
            // Commande pour avancer
            Motor_Move(4000, 4000, 4000, 4000); // Vitesse positive pour avancer
        }
        else if (CMD_BACKWARD == cmd)
        {
            // Commande pour reculer
            Motor_Move(-4000, -4000, -4000, -4000); // Vitesse négative pour reculer
        }
        else if (CMD_LEFT == cmd)
        {
            // Commande pour tourner à gauche
            Motor_Move(-2000, 2000, -2000, 2000); // Ajustez les valeurs pour tourner à gauche
        }
        else if (CMD_RIGHT == cmd)
        {
            // Commande pour tourner à droite
            Motor_Move(2000, -2000, 2000, -2000); // Ajustez les valeurs pour tourner à droite
        }

        notifyClients();
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
        client->ping();
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id(), arg ? *(uint16_t *)arg : 0);
    }
    else if (type == WS_EVT_ERROR)
    {
        Serial.printf("ws[%s][%u] error: %s\n", server->url(), client->id(), (char *)arg);
    }
    else if (type == WS_EVT_PONG)
    {
        Serial.printf("ws[%s][%u] pong: %u\n", server->url(), client->id(), arg ? *(uint16_t *)arg : 0);
    }
    else if (type == WS_EVT_MESSAGE)
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->opcode == WS_TEXT)
        {
            handleWebSocketMessage(arg, data, len);
        }
    }
}

void initWebSocket()
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client"))
        {
            Serial.println("connected");
            client.subscribe("esp32/track");
            client.subscribe("esp32/sonar");
            client.subscribe("esp32/light");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}
