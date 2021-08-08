#pragma once

#include <ArduinoJson.h>

#include "helpers.h"

template<typename CLIENT>
class TelegramBot {
public:
    /** Sent or received message.
     */
    class Message {
        String chatId;
        String senderId;
        String text;
    }; // TelegramBot::Message

    TelegramBot(String && token):
        token_{std::move(token)} {
    }


    /** Sends the simple text message specified to the given chat. 
     
        Returns true if successful, false otherwise. 
     */
    bool sendMessage(String const & chatId, String const & text) {
        GET(STR("/sendMessage?chat_id=" + chatId + "&text=" + text));

    }

private:

    /** Initializes a simple get request, i.e. all parameters in the url. 
     */
    void GET(String const & url) {
        client_->print(PSTR("GET /bot"));
        client_->print(token_);
        client_->print(url);
        client_->print(PSTR(" HTTP/1.1\n"));
        client_->print(PSTR("Host: api.telegram.org\n"));
        client_->print(PSTR("Accept: application/json\n"));
        client_->print(PSTR("Cache-Control: no-cache\n\n"));
    }

    /** Waits for the response to the initiated request and verifies that it is a success. Returns true if success, false otherwise. 
     */
    bool responseOk() {

    }    

    String response() {

    }

    /** Skips the response headers entirely
     */
    void skipResponseHeaders() {

    }

    char * token_;
    CLIENT client_;    
};