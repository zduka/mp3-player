#pragma once

#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include "helpers.h"

//#define HTTPS_SEND(...) Serial.printf_P(__VA_ARGS__); https_.printf_P(__VA_ARGS__)
#define HTTPS_SEND(...) https_.printf_P(__VA_ARGS__)

class TelegramBot {
public:
    /** Initializes the telegram bot. 
     
        Takes the bot id and access token as arguments. Furthermore, a certificate for the server must be provided for HTTPS to work. If this is explicitly set to null, the HTTPS will proceed in the insecure mode. Note that this is really insecure. 
     */
    void initialize(int64_t id, String && token, char const * cert) {
        LOG("TelegramBot: id: %lli, token: %s %s", id, token.c_str(), cert == nullptr ? "INSECURE" : "");
        id_ = id;
        token_ = std::move(token);
        if (cert != nullptr) {
            certs_.append(cert);
            https_.setTrustAnchors(& certs_);
        } else {
            https_.setInsecure();
        }
    }

    /** Sends a simple text message to the given chat. 
     */
    bool sendMessage(int64_t chatId, char const * message) {
        if (!connect())
            return false;
        HTTPS_SEND(PSTR("GET /bot%lli:%s/sendMessage?chat_id=%lli&text="), id_, token_.c_str(), chatId);
        UrlEncode(https_, message);
        HTTPS_SEND(PSTR(" HTTP/1.1\r\nHost: api.telegram.org\r\nAccept: application/json\r\nCache-Control: no-cache\r\n"));
        HTTPS_SEND(PSTR("\r\n"));
        return responseOk();
    }

    using UploadCallback = std::function<void(uint16_t, uint16_t)>;

    /** Sends given audio file to the specified chat. 
     */
    bool sendAudio(int64_t chatId, File & f, char const * filename, char const * mime, UploadCallback callback) {
        if (!connect())
            return false;
        size_t fileSize = f.size();
        size_t len = 0;
        char buffer[512];
        len += snprintf_P(buffer + len, sizeof(buffer) - len, PSTR("--%s\r\n"), BOUNDARY);
        len += snprintf_P(buffer + len, sizeof(buffer) - len, PSTR("Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n%lli\r\n"), chatId);
        len += snprintf_P(buffer + len, sizeof(buffer) - len, PSTR("--%s\r\n"), BOUNDARY);
        len += snprintf_P(buffer + len, sizeof(buffer) - len, PSTR("Content-Disposition: form-data; name=\"audio\"; filename=\"%s\"\r\n"), filename);
        len += snprintf_P(buffer + len, sizeof(buffer) - len, PSTR("Content-Type: %s\r\n\r\n"), mime);

        char buffer2[128];
        size_t len2 = snprintf_P(buffer2, sizeof(buffer2), PSTR("\r\n--%s--\r\n"), BOUNDARY);
        HTTPS_SEND(PSTR("POST /bot%lli:%s/sendAudio"), id_, token_.c_str());
        HTTPS_SEND(PSTR(" HTTP/1.1\r\nHost: api.telegram.org\r\nAccept: application/json\r\nCache-Control: no-cache\r\n"));
        HTTPS_SEND(PSTR("Content-Type: multipart/form-data; boundary=%s\r\n"), BOUNDARY);
        HTTPS_SEND(PSTR("Content-Length: %u\r\n"), len + len2 + fileSize);
        HTTPS_SEND(PSTR("\r\n"));
        https_.write(pointer_cast<uint8_t *>(buffer), len);
        uint16_t chunks = fileSize / sizeof(buffer) + 2;
        uint16_t i = 1;
        while (f.available()) {
            if (callback)
                callback(i++, chunks);
            size_t n = f.readBytes(buffer, sizeof(buffer));
            https_.write(pointer_cast<uint8_t *>(buffer), n);
        }
        https_.write(pointer_cast<uint8_t *>(buffer2), len2);
        if (callback)
            callback(i++, chunks);
        return responseOk();
    }

    bool getUpdate(JsonDocument & into, int64_t offset = 0) {
        into.clear();
        if (!connect())
            return false;
        HTTPS_SEND(PSTR("GET /bot%lli:%s/getUpdates?limit=1&offset=%lli"), id_, token_.c_str(), offset);
        HTTPS_SEND(PSTR(" HTTP/1.1\r\nHost: api.telegram.org\r\nAccept: application/json\r\nCache-Control: no-cache\r\n"));
        HTTPS_SEND(PSTR("\r\n"));
        if (skipResponseHeaders() != HTTP_OK) {
            https_.stop();
            return false;
        }
        deserializeJson(into, https_);
        https_.stop();
        return true;
    }

    /** Getting a file is a two-step process.
     */
    bool getFile(char const * fileId, JsonDocument &fileInfo, File & into) {
        fileInfo.clear();
        if (!connect())
            return false;
        HTTPS_SEND(PSTR("GET /bot%lli:%s/getFile?file_id=%u"), id_, token_.c_str(), fileId);
        HTTPS_SEND(PSTR(" HTTP/1.1\r\nHost: api.telegram.org\r\nAccept: application/json\r\nCache-Control: no-cache\r\n"));
        HTTPS_SEND(PSTR("\r\n"));
        if (skipResponseHeaders() != HTTP_OK) {
            https_.stop();
            return false;
        }
        deserializeJson(fileInfo, https_);
        https_.stop();
        // actually request the file
        if (!connect())
            return false;
        HTTPS_SEND(PSTR("GET /file/bot%lli:%s/%s"), id_, token_.c_str(), fileInfo["result"]["file_path"].as<char const *>());
        HTTPS_SEND(PSTR(" HTTP/1.1\r\nHost: api.telegram.org\r\nCache-Control: no-cache\r\n"));
        HTTPS_SEND(PSTR("\r\n"));
        if (skipResponseHeaders() != HTTP_OK) {
            https_.stop();
            return false;
        }
        // TODO actually store the file
        uint32_t count = 0;
        do {
            if (https_.available()) {
                while (https_.available()) {
                    char c = https_.read();
                    into.write(c);
                    ++count;
                    // TODO callback
                }
            } else {
                delay(1);
            }
        } while (https_.connected());
        https_.stop(); // just to be sure
        return true;
    }



    /** All is good.
     */
    static inline constexpr uint16_t HTTP_OK = 200;
    /** Network timeout (anywhere). 
     */
    static inline constexpr uint16_t HTTP_TIMEOUT = 504;
    /** Simplified any other state. 
     */
    static inline constexpr uint16_t HTTP_ERROR = 400;

private:

    static inline constexpr char const * PROGMEM BOUNDARY = "------------73e5a323s031399w96f31669we93";

    bool connect() {
        //api.telegram.org // IPAddress(149,154,167,220)
        if (! https_.connected() && !https_.connect("api.telegram.org", 443)) {
            LOG("Cannot connect to api.telegram.org");
            return false;
        } else {
            return true;
        }
    }

    /** Waits for the response to the initiated request and verifies that it is a success. Returns true if success, false otherwise. 
     */
    bool responseOk() {
        if (skipResponseHeaders() == HTTP_OK) {
            skipResponseBody();
            https_.stop();
            return true;
        } else {
            https_.stop();
            return false;
        }
    }    

    /** Skips the response headers entirely, but returns the status code
     */
    uint16_t skipResponseHeaders() {
        uint32_t t = millis() + timeout_;
        uint32_t last = 0;
        uint16_t result = 0;
        do {
            if (https_.available()) {
                while (https_.available()) {
                    char c = https_.read();
                    last = (last << 8) | c;
                    if (last == 0x0d0a0d0a)
                        return result;
                    if (result == 0 && (last & 0xff000000) == 0x20000000) {
                        result = ((last >> 16) & 0xff) - '0';
                        result = result * 10 + (((last >> 8) & 0xff) - '0');
                        result = result * 10 + ((last & 0xff) - '0');
                    }
                }
            } else {
                delay(1);
            }
        } while (millis() < t);
        LOG("Response headers timeout");
        return HTTP_TIMEOUT;
    }

    uint32_t skipResponseBody() {
        uint32_t t = millis() + timeout_;
        uint32_t count = 0;
        do {
            if (https_.available()) {
                while (https_.available()) {
                    char c = https_.read();
                    ++count;
                }
            } else {
                delay(1);
            }
        } while (millis() < t && https_.connected());
        return count;
    }

    X509List certs_;
    WiFiClientSecure https_;    
    uint32_t timeout_ = 2000;

    int64_t id_;
    String token_;
}; 

#undef HTTPS_SEND