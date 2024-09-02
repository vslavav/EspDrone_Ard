#include <Arduino.h>
#include <WiFi.h>


void setup_STA();
void setup_AP();


#define WIFI_TIMEOUT 10000 // WiFi connection timeout (ms)

const char *ssid_STA = "BELL810";
const char *password_STA = "27452E1FF2E7";

//const char *ssid_STA = "slava_s9";
//const char *password_STA = "hehv4187";

const char *ssid_AP = "ESP_Slava";    //Enter the router name
const char *password_AP = "slava=10"; //Enter the router password

IPAddress local_IP(192, 168, 4, 100); //Set the IP address of ESP32 itself
IPAddress gateway(192, 168, 4, 10);   //Set the gateway of ESP32 itself
IPAddress subnet(255, 255, 255, 0);   //Set the subnet mask for ESP32 itself

void WiFiMgr_setup()
{
    setup_STA();
    //setup_AP();
    
}

void setup_STA()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_STA, password_STA);

    int wait = 200;
    int waited = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(wait);
        waited += wait;
        //Serial.print(".");

        if (waited > WIFI_TIMEOUT)
        {
            Serial.println("connection failed");
            return;
        }
    }

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    delay(100);
}

void setup_AP()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    //Serial.println("Setting soft-AP ... ");
    bool result = WiFi.softAP(ssid_AP, password_AP);
}

void loop_WiFiMgr()
{
    return;
    static bool bIsReported = false;
    if(WiFi.softAPgetStationNum() > 0 && !bIsReported)
    {
        Serial.println("WiFi got connection");
        bIsReported = true;
    }
    else if (WiFi.softAPgetStationNum() == 0)
    {
        bIsReported = false;
    }
}