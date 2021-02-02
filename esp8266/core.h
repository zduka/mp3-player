#include <ESP8266WiFi.h>


#define LOG(...) Core::Log(String("") + __VA_ARGS__)


class Core {
public:
    /** Simple RAII class to manage scanned networks
     */
    class Networks {
        friend class Core;
    public:
        Networks(Networks && from):
            n_{from.n_} {
            from.n_ = -1;
        }
        
        ~Networks() {
            if (n_ >= 0)
                WiFi.scanDelete();
        }
        
        int size() const {
            return n_;
        }

        String const & ssid(int i) const {
            return WiFi.SSID(i);
        }

        int rssi(int i) const {
            return WiFi.RSSI(i);
        }

    private:

        Networks(int n):
            n_{n} {
        }
        
        int n_;
    }; // Core::Networks

    /** Initializes the chip. 
     */
    static void Initialize() {
        Serial.begin(115200);
        delay(50);
        LOG("Initializing ESP8266...");
        // disable caching the SSID and password in memory, which wears it down excessively
        // https://github.com/esp8266/Arduino/issues/1054
        WiFi.persistent(false);
        
        FSInfo fs_info;
        SPIFFS.info(fs_info); 
        LOG("SPIFFS Stats:");
        LOG("  Total bytes:      " +fs_info.totalBytes);
        LOG("  Used bytes:       " +fs_info.usedBytes);
        LOG("  Block size:       " +fs_info.blockSize);
        LOG("  Page size:        " +fs_info.pageSize);
        LOG("  Max open files:   " +fs_info.maxOpenFiles);
        LOG("  Max path lenghth: " +fs_info.maxPathLength);        
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

    /** Initiates network scan and returns the number of networks found.  
     */
    static Networks ScanNetworks() {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        Networks result{WiFi.scanNetworks()};
        for (int i = 0, e = result.size(); i < e; ++i)
            LOG(result.ssid(i) + ", rssi: " + result.rssi(i));
        return result;
    }

    /*
    static void EnableWiFi(char const * ssid, char const * password) {
        WiFi.begin(ssid, password);
    }
    */

    


    

    
}; // Core


