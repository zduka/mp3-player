
/** A simple button. 
 */
class Button {
public:
    Button(uint8_t pin, bool internalPullUp = false):
        pin_{pin} {
        pinMode(pin, internalPullUp ? INPUT_PULLUP : INPUT);
        state_.state = digitalRead(pin);
    }

    void setInterrupt(void (*isr)()) const {
        attachInterrupt(digitalPinToInterrupt(pin_), isr, CHANGE);    
    }

    bool changed() const {
        return state_.changed;
    }

    bool pressed() const {
        state_.changed = 0;
        return ! state_.state; 
    }

    bool poll() {
        uint8_t state = digitalRead(pin_);
        if (state != state_.state) {
            state_.changed = 1;
            state_.state = state;
            return true;
        } else {
            return false;
        }
    }
private:
    uint8_t pin_;

    mutable volatile struct {
        uint8_t changed : 1;
        uint8_t state : 1;
    } state_;
};

class RotaryEncoder {
public:
    RotaryEncoder(uint8_t pinA, uint8_t pinB, uint8_t maxValue, bool internalPullUp = false):
        a_{pinA},
        b_{pinB},
        maxValue_{maxValue} {
        pinMode(a_, internalPullUp ? INPUT_PULLUP : INPUT);
        pinMode(b_, internalPullUp ? INPUT_PULLUP : INPUT);
    }

    void setInterrupt(void (*isr)()) const {
        attachInterrupt(digitalPinToInterrupt(a_), isr, CHANGE);    
    }

    void clearInterrupt() {
        detachInterrupt(digitalPinToInterrupt(a_));
    }

    /** Updates the value of the encoder.

        Safe to be called from within an ISR.
     */
    bool poll() {
        uint8_t lastState = state_.state;
        uint8_t state = digitalRead(a_) * 2 + digitalRead(b_);
        if ((lastState & 2) != (state & 2)) {
            lastState = ((lastState << 2) + state) & 0xf;
            state_.state = lastState;
            switch (lastState) {
            case 6 : // from 01 to 10
                return inc();
            case 3 : // from 00 to 11
                return dec();
            }
        }
        return false;
    }

    bool inc() {
      if (value_ + 1 < maxValue_) {
          ++value_;
          state_.changed = 1;
          return true;
      } else {
          return false;
      }
    }

    bool dec() {
      if (value_ > 0) {
          --value_;
          state_.changed = 1;
          return true;
      } else {
        return false;
      }
    }

    bool changed() const {
        return state_.changed;
    }

    uint16_t value() const {
        state_.changed = 0;
        return value_;
    }

    void setValue(uint16_t value) {
        value_ = value;
        state_.changed = 1;
    }

    uint16_t maxValue() const {
        return maxValue_;
    }
    
    void setMaxValue(uint16_t value) {
        maxValue_ = value;
        if (value_ != 0 && value_ >= maxValue_)
            value_ = maxValue_ - 1;
    }

    void setValues(uint16_t value, uint16_t max) {
        value_ = value;
        maxValue_ = max;
    }

private:

    uint8_t a_;
    uint8_t b_;
    uint16_t maxValue_;
    volatile uint16_t value_ = 0;

    mutable volatile struct {
        uint8_t changed : 1;
        uint8_t state : 4;
    } state_;
  
};
