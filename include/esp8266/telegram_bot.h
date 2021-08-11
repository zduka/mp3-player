#pragma once

#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include "helpers.h"

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
        https_.printf_P(PSTR("GET /bot%s:%s/sendMessage?chat_id=%s&text="), id_.c_str(), token_.c_str(), chatId);
        UrlEncode(https_, text);
        https_.printf_P(PSTR(" HTTP/1.1\r\nHost: api.telegram.org\r\nAccept: application/json\r\nCache-Control: no-cache\r\n\r\n"));
        return responseOk();
    }

    /** All is good.
     */
    static constexpr uint16_t HTTP_OK = 200;
    /** Network timeout (anywhere). 
     */
    static constexpr uint16_t HTTP_TIMEOUT = 504;
    /** Simplified any other state. 
     */
    static constexpr uint16_t HTTP_ERROR = 400;

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
        uint32_t t = millis() + timeout_;
        uint16_t result = skipResponseHeaders();
        skipResponseBody();
        return result == HTTP_OK;
    }    

    static const uint8_t WAIT_FOR_SPACE = 0;
    static const uint8_t WORKING = 1;
    static const uint8_t DONE = 2;

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
                    if (result == 0 && (last & 0x20000000) == 0x20000000) {
                        result = ((last >> 16) & 0xff) - '0';
                        result = result * 10 + (((last >> 8) & 0xff) - '0');
                        result = result * 10 + ((last & 0xff) - '0');
                    }
                }
            } else {
                delay(1);
            }
        } while (millis() < t);
        return HTTP_TIMEOUT;
    }

    uint32_t skipResponseBody() {
        uint32_t t = millis() + timeout_;
        uint32_t count = 0;
        do {
            if (https_.available()) {
                while (https_.available()) {
                    char c = https_.read();
                    Serial.print(c);
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