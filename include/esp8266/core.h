#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <SD.h>


#define LOG(...) Core::Log(String("") + __VA_ARGS__)

class Core {
public:

    /** Initializes the chip. 
     */
    static void Setup(bool disableWiFi = false) {
        // instead of 115200 start at 74880, which is the baudrate at which ESP bootloader prints its stuff as well so both are legible
        Serial.begin(74880);
        delay(50);
        LOG("Initializing ESP8266...");
        // disable caching the SSID and password in memory, which wears it down excessively
        // https://github.com/esp8266/Arduino/issues/1054
        WiFi.persistent(false);
        if (disableWiFi)
            DisableWiFi();
        LOG("mac address: " + WiFi.macAddress());
        LOG("Initializing LittleFS");
        LittleFS.begin();
        FSInfo fs_info;
        LittleFS.info(fs_info); 
        LOG("LittleFS Stats:");
        LOG("  Total bytes:      " +fs_info.totalBytes);
        LOG("  Used bytes:       " +fs_info.usedBytes);
        LOG("  Block size:       " +fs_info.blockSize);
        LOG("  Page size:        " +fs_info.pageSize);
        LOG("  Max open files:   " +fs_info.maxOpenFiles);
        LOG("  Max path lenghth: " +fs_info.maxPathLength);
        LOG("Initializing OTA");
        ArduinoOTA.onStart(OTAStart);
        ArduinoOTA.onEnd(OTAEnd);
        ArduinoOTA.onProgress(OTAProgress);
        ArduinoOTA.onError(OTAError);
        ArduinoOTA.begin();
        LOG("Setting up web server");
        Server.onNotFound(http404);
        Server.serveStatic("/", LittleFS, "/index.html");
        Server.serveStatic("/favicon.ico", LittleFS, "/favicon.ico");
    }
    
    static void Log(String const & str) {
        Serial.print(String(millis() / 1000) + ": ");
        Serial.println(str);
    }

    /** Disables the wifi immediately. 
  
        Disables the wifi and enters power saving mode for it. 
     */
    static void DisableWiFi() {
        WiFi.mode(WIFI_OFF);
        WiFi.forceSleepBegin();
        LOG("WiFi disabled");
    }

    /** Enters deep sleep mode. 

        During the deep sleep cpu is off and can only wake up by applying a pulse to the RST pin. 
     */
    static void DeepSleep() {
        LOG("Entering deep sleep");
        ESP.deepSleep(0);
    }
    
    static void Connect(std::initializer_list<char const *> networks) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        WiFi.scanNetworksAsync([=](int n) {
            LOG("WiFi scan done." + n + " networks found");
            for (int i = 0; i < n; ++i) {
                for (auto ni = networks.begin(), ne = networks.end(); ni < ne; ni += 2) {
                    if (WiFi.SSID(i) == *ni) {
                        LOG("Connecting to " + *ni);
                        WiFi.begin(*ni, *(ni + 1));
                        WiFi.scanDelete();
                        return;
                    }
                }
            }
            LOG("No valid network, available:");
            for (int i = 0; i < n; ++i)
                LOG("    " + WiFi.SSID(i) + ", rssi: " + WiFi.RSSI(i));
            LOG("Known:");
            for (auto ni = networks.begin(), ne = networks.end(); ni < ne; ni += 2)
                LOG("    " + *ni);
            WiFi.scanDelete();
        });
    }

    static void Loop() {
        ArduinoOTA.handle();
        Server.handleClient();
    }

    static inline ESP8266WebServer Server{80};


private:

    static void http404() {
        Server.send(404, "text/json","{ \"response\": 404, \"uri\": \"" + Server.uri() + "\" }");
    }

    static void OTAStart() {
        LOG("OTA update of " + (ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "spiffs") + " started.");
    }

    static void OTAEnd() {
        LOG("OTA update done");
    }

    static void OTAProgress(unsigned progress, unsigned total) {
        LOG("OTA progress " + progress + " of " + total);  
    }

    static void OTAError(ota_error_t error) {
        LOG("OTA error: " + error);
        switch (error) {
        case OTA_AUTH_ERROR:
            LOG("Auth failed");
            break;
        case OTA_BEGIN_ERROR:
            LOG("Begin Failed");
            break;
        case OTA_CONNECT_ERROR:
            LOG("Connect Failed");
            break;
        case OTA_RECEIVE_ERROR:
            LOG("Receive Failed");
            break;
        case OTA_END_ERROR:
            LOG("End Failed");
            break;
        default:
            break;
        }
    }    
    
}; // Core
