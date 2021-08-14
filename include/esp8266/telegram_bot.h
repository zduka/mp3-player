#pragma once

#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include "helpers.h"

//#define HTTPS_SEND(...) Serial.printf_P(__VA_ARGS__); https_.printf_P(__VA_ARGS__)
#define HTTPS_SEND(...) https_.printf_P(__VA_ARGS__)

class TelegramBot {
public:
    /** Sent or received message.
     */
    class Message {
        String chatId;
        String senderId;
        String text;
    }; // TelegramBot::Message

    TelegramBot() {
    }

    void initialize(String && id, String && token) {
        id_ = std::move(id);
        token_ = std::move(token);
        https_.setInsecure();
    }

    void initialize(String && id, String && token, char const * cert) {
        id_ = std::move(id);
        token_ = std::move(token);
        certs_.append(cert);
        https_.setTrustAnchors(& certs_);
    }

    bool isValid() const {
        return ! id_.isEmpty();
    }

    /** Sends the simple text message specified to the given chat. 
     
        Returns true if successful, false otherwise. 
     */
    bool sendMessage(char const * chatId, char const * text) {
        if (!connect())
            return false;
        HTTPS_SEND(PSTR("GET /bot%s:%s/sendMessage?chat_id=%s&text="), id_.c_str(), token_.c_str(), chatId);
        UrlEncode(https_, text);
        HTTPS_SEND(PSTR(" HTTP/1.1\r\nHost: api.telegram.org\r\nAccept: application/json\r\nCache-Control: no-cache\r\n"));
        HTTPS_SEND(PSTR("\r\n"));
        return responseOk();
    }

    bool sendAudio(int64_t chatId, File & f, char const * filename, char const * mime) {
        if (!connect())
            return false;
        HTTPS_SEND(PSTR("POST /bot%s:%s/sendAudio"), id_.c_str(), token_.c_str());
        HTTPS_SEND(PSTR(" HTTP/1.1\r\nHost: api.telegram.org\r\nAccept: application/json\r\nCache-Control: no-cache\r\n"));
        HTTPS_SEND(PSTR("Content-Type: multipart/form-data; boundary=%s\r\n"), BOUNDARY);
        char buf[22];
        uint32_t contentLength = 44 + 50 + 2 + snprintf_P(buf, 21, PSTR("%lli"), chatId) + 44 + 59 + strlen(filename) + 18 + strlen(mime) + f.size() + 48;
        //LOG("Sending document, contents length: %u, file size: %u", contentLength, f.size());
        HTTPS_SEND(PSTR("Content-Length: %u\r\n"), contentLength);
        HTTPS_SEND(PSTR("\r\n"));
        // body
        HTTPS_SEND(PSTR("--%s\r\n"), BOUNDARY); // 44
        HTTPS_SEND(PSTR("Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n")); // 50
        HTTPS_SEND(PSTR("%s\r\n"), buf); // 2 + strlen(chatId)
        HTTPS_SEND(PSTR("--%s\r\n"), BOUNDARY); // 44
        HTTPS_SEND(PSTR("Content-Disposition: form-data; name=\"audio\"; filename=\"%s\"\r\n"), filename); // 59 + strlen(filename)
        HTTPS_SEND(PSTR("Content-Type: %s\r\n\r\n"), mime); // 18 + strlen(mime)
        uint8_t buffer[256];
        while (f.available()) {
            size_t n = f.readBytes(pointer_cast<char *>(&buffer), 256);
            https_.write(buffer, n);
            Serial.print(".");
        }
        //https_.write(f); // f.size()
        HTTPS_SEND(PSTR("\r\n--%s--\r\n"), BOUNDARY); // 48
        return responseOk();
    }

    bool getUpdate(JsonDocument & into, uint32_t offset = 0) {
        into.clear();
        if (!connect())
            return false;
        HTTPS_SEND(PSTR("GET /bot%s:%s/getUpdates?limit=1&offset=%u"), id_.c_str(), token_.c_str(), offset);
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
        HTTPS_SEND(PSTR("GET /bot%s:%s/getFile?file_id=%u"), id_.c_str(), token_.c_str(), fileId);
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
        HTTPS_SEND(PSTR("GET /file/bot%s:%s/%s"), id_.c_str(), token_.c_str(), fileInfo["result"]["file_path"].as<char const *>());
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

    static inline constexpr char const * PROGMEM BOUNDARY = "------------73e5a323s031399w96f31669we93";

private:

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
                    Serial.print(c);
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

    String id_;
    String token_;
    X509List certs_;
    WiFiClientSecure https_;    
    uint32_t timeout_ = 2000;
};

#undef HTTPS_SEND