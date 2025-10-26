/**
 * @file main.ino
 * @brief Clock program for an 8-bit computer learning project.
 * @author HobbyHacker
 * @version 0.1.4
 * @date 2025-10-20
 *
 * @details
 * This experimental project is intended to help me and others learn and
 * better understand how a computer works by using an ATmega and other
 * integrated circuits to construct the basic computational components of
 * a simple CPU-like system.
 *
 * The file contains the main application logic and several small utility
 * classes used by the system (Heartbeat, RGB LED, Buzzer, Button and
 * MachineState). Documentation has been cleaned up throughout to improve
 * clarity and correct spelling/grammar.
 */

/* ------------------------------------------------------------ */
/*                      PIN CONFIGURATION                       */
/* ------------------------------------------------------------ */

struct PinMap
{
    int heartbeatLed;
    int counterReset;
    int buzzer;
    int buttonReset;
    int buttonPause;
    int buttonPlay;
    int buttonStep;
    int rgbRed;
    int rgbGreen;
    int rgbBlue; 
};

PinMap pins =
{
    13, // heartbeat LED
    2,  // counter reset pin (pulse output)
    7,  // buzzer
    8,  // reset button
    9,  // pause button
    10, // play button
    11, // step button
    3,  // RGB red
    5,  // RGB green
    6   // RGB blue
};

/* ------------------------------------------------------------ */
/*                        CLASS SECTION                         */
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*                       HEARTBEAT CLASS                        */
/* ------------------------------------------------------------ */

/**
 * @class HeartBeat
 * @brief Non-blocking LED pulse/blink helper for system feedback.
 *
 * @details
 * The HeartBeat class generates a timed blink or pulse on a digital
 * output (typically an LED) using millis() for timing so it does not
 * block the main loop. It can be used as a status/heartbeat indicator
 * and also to pulse a counter-reset line.
 *
 * Behavior summary:
 *  - Toggles an output at a regular interval (default 500 ms).
 *  - Setting the interval to 0 disables automatic blinking (output is held LOW).
 *  - stepBeat() performs a short, immediate pulse for manual triggering.
 *  - pulseOnce() provides a single non-blocking pulse of a given duration.
 *
 * Usage:
 *  HeartBeat hb(pins, 500);
 *  hb.begin();
 *  // in loop:
 *  hb.update();        // non-blocking timing logic
 *  hb.refreshOutput(); // apply the current output level to the pin
 */
class HeartBeat
{
    public:
        /**
         * @brief Construct a HeartBeat instance.
         * @param pins Reference to the project PinMap.
         * @param ms Blink interval in milliseconds (0 = disabled).
         */
        HeartBeat(const PinMap& pins, unsigned long ms = 500);

        /**
         * @brief Initialize hardware pins and internal state.
         * @note Call once from setup().
         */
        void begin();

        /**
         * @brief Update timing logic (non-blocking).
         * @details Should be called frequently from loop(); it will flip
         * the internal state when the configured interval elapses.
         */
        void update();

        /**
         * @brief Apply the internal output state to the hardware pin.
         * @details Call after update() or when a manual change is needed.
         */
        void refreshOutput();

        /**
         * @brief Force a short manual pulse using blocking delay.
         * @note Uses a short delay; intended for diagnostic/manual trigger only.
         */
        void stepBeat();

        /**
         * @brief Start a non-blocking single pulse.
         * @param duration Duration in ms the LED remains HIGH for the pulse.
         * @details Call repeatedly in loop(); the method manages its own state
         * and will return after the pulse completes.
         */
        void pulseOnce(unsigned long duration = 50);

        /**
         * @brief Pulse the counter-reset output for a short fixed time.
         * @details This uses a blocking delay; it is intended for simple reset pulses.
         */
        void resetCounter();

        void setInterval(unsigned long ms);
        void setOutput(bool level);
        unsigned long getInterval() const;
        bool getState() const;

    private:
        const PinMap& pins;
        unsigned long previousMillis;
        unsigned long interval;
        bool isOn;
};

/* ------------------------------------------------------------ */
/*                        RGBLed Class                          */
/* ------------------------------------------------------------ */

/**
 * @class RGBLed
 * @brief Simple RGB LED driver for visual system state.
 *
 * @details
 * Controls a common RGB LED using three digital pins. Provides a small set
 * of named colors and a setColor() convenience method. The implementation
 * uses digitalWrite and analogWrite where a mix of channels is desired.
 *
 * Usage:
 *  RGBLed statusLed(pins);
 *  statusLed.begin();
 *  statusLed.setColor(RGBLed::RgbColor::Green);
 */
class RGBLed
{
public:
    /**
     * @enum RgbColor
     * @brief Predefined colors that the RGB LED can show.
     */
    enum class RgbColor {Red, Orange, Yellow, Green, Blue, Violet, Off};

    /**
     * @brief Construct an RGBLed controller.
     * @param pins Reference to the global PinMap with RGB pins defined.
     */
    RGBLed(const PinMap& pins);

    /**
     * @brief Configure the RGB pins as outputs and set a known state.
     * @note Call from setup() once.
     */
    void begin();

    /**
     * @brief Placeholder for reading inputs that might change LED behavior.
     * @details Currently unused but provided for future expansion.
     */
    void handleInput();

    /**
     * @brief Placeholder for animation/timing updates (if added later).
     */
    void update();

    /**
     * @brief Log or otherwise report the current color to Serial.
     * @note This returns void; use getColorName() if you want a string return instead.
     */
    void getColor();

    /**
     * @brief Set the LED to one of the predefined colors.
     * @param color The desired color from the RgbColor enum.
     */
    void setColor(RgbColor color);

private:
    RgbColor currentRGBLedColor;
    const PinMap& pins;
};

/* ------------------------------------------------------------ */
/*                        BUZZER CLASS                          */
/* ------------------------------------------------------------ */

/**
 * @class Buzzer
 * @brief Non-blocking buzzer/tone manager.
 *
 * @details
 * The Buzzer class turns a digital output on for a specified duration using
 * millis() for non-blocking timing. It optionally drives the RGB LED to
 * a warning color while beeping.
 *
 * Usage:
 *  Buzzer buzzer(pins, 100, &rgbLed);
 *  buzzer.begin();
 *  buzzer.beep(200);
 *  // in loop:
 *  buzzer.update();
 *  buzzer.refreshOutput();
 */
class Buzzer
{
    public:
        /**
        * @brief Construct a Buzzer controller.
        * @param pins Reference to the PinMap.
        * @param duration Default beep duration in milliseconds.
        * @param led Optional pointer to an RGBLed instance (used to show a color while beeping).
        */
        Buzzer(const PinMap& pins, unsigned long duration, RGBLed* led = nullptr);

        /**
        * @brief Configure the buzzer pin and perform a startup beep.
        */
        void begin();

        /**
        * @brief Start a beep for a specific duration (non-blocking).
        * @param duration How long the buzzer stays on, in milliseconds.
        */
        void beep(unsigned long duration);

        /**
        * @brief Check the time and stop the beep when the duration expires.
        * @note Must be called regularly from loop().
        */
        void update();

        /**
        * @brief Apply the buzzer state to the hardware pin.
        */
        void refreshOutput();        

        /**
        * @brief Query whether the buzzer is currently beeping.
        * @return true if active, false otherwise.
        */
        bool getState();

    private:
    RGBLed* rgbLed;
    const PinMap& pins;

    int buzzerPin; ///< Output pin connected to the buzzer.
    unsigned long beepDuration; ///< Duration of the current beep in ms.
    unsigned long previousMillis; ///< Timestamp for non-blocking timing.
    bool isBeeping; ///< Whether the buzzer is currently active.
};

/* ------------------------------------------------------------ */
/*                        BUTTON CLASS                          */
/* ------------------------------------------------------------ */

/**
 * @class Button
 * @brief Reads and debounces multiple control buttons (Reset, Pause, Play, Step).
 *
 * @details
 * Buttons are configured with INPUT_PULLUP. A pressed button reads LOW.
 * The class currently implements a simple debounce delay and reports a
 * single event for each press transition (HIGH -> LOW).
 */
class Button
{
    public:
        /**
        * @enum ButtonEvent
        * @brief Events produced by the button handler.
        */
        enum class ButtonEvent{ None, ResetPressed, PausedPressed, PlayPressed, StepPressed};
        
        /**
        * @brief Construct a Button manager.
        * @param pins Reference to the project's PinMap.
        */
        Button(const PinMap& pins);

        /**
        * @brief Initialize button pins (INPUT_PULLUP).
        * @note Call once from setup().
        */
        void begin();

        /**
        * @brief Poll all buttons and return the first detected press event.
        * @return A ButtonEvent value indicating which button transitioned to pressed.
        * @details Uses a small debounce delay (debounceDelay). Returns None if no
        * valid press is detected.
        */
        ButtonEvent handleInput();

        /**
        * @brief Reserved for future debouncing or multi-button logic.
        */
        void update();

        bool getButtonStatus();
        
    private:
        const PinMap& pins;

        int resetButton; ///< Digital pin number for Reset button.
        int pauseButton; ///< Digital pin number for Pause button.
        int playButton; ///< Digital pin number for Play button.
        int stepButton; ///< Digital pin number for Step button.

        unsigned long lastDebounceTime;
        const unsigned long debounceDelay = 100;

        bool buttonPressed;
};

/* ------------------------------------------------------------ */
/*                        MACHINESTATE CLASS                    */
/* ------------------------------------------------------------ */

/**
 * @class MachineState
 * @brief Central controller for machine mode and subsystem coordination.
 *
 * @details
 * Manages machine modes (Reset, Pause, Run, Step) and coordinates the
 * HeartBeat, RGBLed, Buzzer and Button subsystems to reflect the current
 * state and process user input.
 */
class MachineState
{
    public:
        /**
        * @enum State
        * @brief Machine operating modes.
        */
        enum class State {Reset, Pause, Run, Step};

        /**
        * @brief Construct the MachineState controller.
        * @param pins Reference to the project's PinMap.
        */
        MachineState(const PinMap& pins);

        /**
        * @brief Initialize all subsystems and hardware (call once from setup()).
        */
        void begin();

        /**
        * @brief Read and process external input signals (call early in loop()).
        */
        void handleInput();

        /**
        * @brief Update internal logic and subsystem states (call each loop).
        */
        void update();

        /**
        * @brief Flush and apply all output changes to hardware (call at end of loop()).
        */
        void refreshOutput();

        /**
        * @brief Get the current machine state.
        * @return Current State enum.
        */
        State getState() const;

        /**
        * @brief Change the machine state (prints transitions to Serial).
        * @param newState Desired state to transition to.
        */
        void setState(State newState);

    private:
        const PinMap& pins;
        State currentState;
        HeartBeat heartbeat; 
        RGBLed rgbLed;
        Buzzer buzzer;
        Button button;
};

/* ------------------------------------------------------------ */
/*                  GLOBAL OBJECT CONSTRUCTED                   */
/* ------------------------------------------------------------ */

MachineState machineState(pins);

/* ------------------------------------------------------------ */
/*                      MAIN PROGRAM FLOW                       */
/* ------------------------------------------------------------ */

void setup()
{
    machineState.begin();
}

void loop()
{
    machineState.handleInput();
    machineState.update();
    machineState.refreshOutput();
}

/* ------------------------------------------------------------ */
/*                    CLASS IMPLEMENTATION                      */
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*                 MachineState Implementation                  */
/* ------------------------------------------------------------ */
MachineState::MachineState(const PinMap& pins) 
    :   pins(pins),
        currentState(State::Reset), 
        heartbeat(pins, 500), 
        rgbLed(pins), 
        buzzer(pins, 100, &rgbLed), 
        button(pins)
{
    // Start in Reset mode (begin() will transition to Run)
}

void MachineState::begin()
{
    // Initialize subsystems and hardware
    delay(500); // short startup delay
    Serial.begin(9600);
    Serial.println("MachineState initialized on 9600 baud");

    heartbeat.begin();
    rgbLed.begin();
    buzzer.begin();
    button.begin();

    // Transition to running state by default
    setState(State::Run);
}

void MachineState::handleInput()
{
    Button::ButtonEvent event = button.handleInput();

    switch (event)
    {
        case Button::ButtonEvent::ResetPressed:  
            {
                buzzer.beep(100); 
                setState(State::Reset);
                delay(1000);
                setState(State::Run);
            }
            break;
        case Button::ButtonEvent::PausedPressed: 
            {
                buzzer.beep(100); 
                setState(State::Pause);
                heartbeat.setOutput(false);
            }
            break;
        case Button::ButtonEvent::PlayPressed: 
            {
                buzzer.beep(100); 
                setState(State::Run);
            }
            break;
        case Button::ButtonEvent::StepPressed: 
            {
                buzzer.beep(100); 
                setState(State::Step);

                // Immediately return to Pause and reflect the step button state
                setState(State::Pause);  
                bool stepHeld = (digitalRead(pins.buttonStep) == LOW);
                heartbeat.setOutput(stepHeld);
            }
            break;
        case Button::ButtonEvent::None: 
            {
                // No action
            }
            break;
    }
}

void MachineState::update()
{
    switch(currentState)
    {
        case State::Reset: heartbeat.setInterval(0); break;
        case State::Pause: heartbeat.setInterval(0); break;
        case State::Run: heartbeat.setInterval(500); break;
        case State::Step: heartbeat.setInterval(0); break;
    }

    // If paused, expose the step button hold state to the heartbeat output
    if (currentState == State::Pause) 
    {
        bool stepHeld = (digitalRead(pins.buttonStep) == LOW); 
        heartbeat.setOutput(stepHeld);
    }
    
    heartbeat.update();
    buzzer.update();
}

void MachineState::refreshOutput()
{
    heartbeat.refreshOutput();
    buzzer.refreshOutput();

    bool isAnyButtonHeld = 
        digitalRead(pins.buttonReset) == LOW ||
        digitalRead(pins.buttonPause) == LOW ||
        digitalRead(pins.buttonPlay) == LOW ||
        digitalRead(pins.buttonStep) == LOW;

    if (isAnyButtonHeld)
    {
        rgbLed.setColor(RGBLed::RgbColor::Red);
    }
    else
    {
        switch (currentState)
        {
            case State::Reset: rgbLed.setColor(RGBLed::RgbColor::Violet); break;
            case State::Pause: rgbLed.setColor(RGBLed::RgbColor::Blue); break;
            case State::Run:   rgbLed.setColor(RGBLed::RgbColor::Green); break;
            case State::Step:  rgbLed.setColor(RGBLed::RgbColor::Yellow); break;
        }
    }

}

MachineState::State MachineState::getState() const
{
    return currentState;
}

void MachineState::setState(State newState)
{
    if (newState != currentState)
    {
        currentState = newState;

        Serial.print("State changed to: ");
        switch (newState)
        {
            case State::Reset: Serial.println("Reset"); break;
            case State::Pause: Serial.println("Pause"); break;
            case State::Run: Serial.println("Run"); break;
            case State::Step: Serial.println("Step"); break;
        }
    }
}

/* ------------------------------------------------------------ */
/*                   HeartBeat Implementation                   */
/* ------------------------------------------------------------ */

HeartBeat::HeartBeat(const PinMap& pins, unsigned long ms)
    : pins(pins), previousMillis(0), interval(ms), isOn(false) {}

void HeartBeat::begin()
{
    pinMode(pins.heartbeatLed, OUTPUT);
    pinMode(pins.counterReset, OUTPUT);
    digitalWrite(pins.heartbeatLed, LOW);
    digitalWrite(pins.counterReset, LOW);

    Serial.println("HeartBeat initialized");
}

void HeartBeat::update()
{
    if (interval == 0)
    {
        return;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        isOn = !isOn;
    }
}

void HeartBeat::refreshOutput()
{
    digitalWrite(pins.heartbeatLed, isOn ? HIGH : LOW);
}

void HeartBeat::stepBeat()
{
    // Manual short pulse (blocking)
    digitalWrite(pins.heartbeatLed, HIGH);
    delay(10);
    digitalWrite(pins.heartbeatLed, LOW);
}

void HeartBeat::pulseOnce(unsigned long duration)
{
    static bool pulseActive = false;
    static unsigned long pulseStart = 0;

    unsigned long currentTime = millis();

    if (!pulseActive) {
        // start pulse (go HIGH)
        digitalWrite(pins.heartbeatLed, HIGH);
        pulseStart = currentTime;
        pulseActive = true;
    } 
    else if (pulseActive && (currentTime - pulseStart >= duration)) {
        // end pulse (return to LOW)
        digitalWrite(pins.heartbeatLed, LOW);
        pulseActive = false;
    }
}

void HeartBeat::resetCounter()
{
    digitalWrite(pins.counterReset, HIGH);
    delay(100);
    digitalWrite(pins.counterReset, LOW);
}

void HeartBeat::setInterval(unsigned long ms)
{
    interval = ms;
}

void HeartBeat::setOutput(bool level)
{
    isOn = level;
    digitalWrite(pins.heartbeatLed, level ? HIGH : LOW);
}

/* ------------------------------------------------------------ */
/*                   RGBLed Implementation                      */
/* ------------------------------------------------------------ */

RGBLed::RGBLed(const PinMap& pins)
    : pins(pins), currentRGBLedColor(RgbColor::Off)
{

}

void RGBLed::begin()
{
    pinMode(pins.rgbRed, OUTPUT);
    pinMode(pins.rgbGreen, OUTPUT);
    pinMode(pins.rgbBlue, OUTPUT);

    setColor(RGBLed::RgbColor::Red);
    getColor();

    Serial.println("RGBLed initialized");
}

void RGBLed::getColor()
{
    Serial.print("Current Color: ");
    switch (currentRGBLedColor)
    {
        case RgbColor::Red:    Serial.println("Red"); break;
        case RgbColor::Orange: Serial.println("Orange"); break;
        case RgbColor::Yellow: Serial.println("Yellow"); break;
        case RgbColor::Green:  Serial.println("Green"); break;
        case RgbColor::Blue:   Serial.println("Blue"); break;
        case RgbColor::Violet: Serial.println("Violet"); break;
        case RgbColor::Off:    Serial.println("Off"); break;
        default:               Serial.println("Unknown"); break;
    }
}

void RGBLed::setColor(RgbColor color)
{
    // Turn all channels off first
    digitalWrite(pins.rgbRed, LOW);
    digitalWrite(pins.rgbGreen, LOW);
    digitalWrite(pins.rgbBlue, LOW);

    // Store current color
    currentRGBLedColor = color;

    switch(color)
    {
        case RgbColor::Red:
            digitalWrite(pins.rgbRed, HIGH);
            break;

        case RgbColor::Orange:
            digitalWrite(pins.rgbRed, HIGH);
            analogWrite(pins.rgbGreen, 128); // mix red + half green
            break;

        case RgbColor::Yellow:
            digitalWrite(pins.rgbRed, HIGH);
            digitalWrite(pins.rgbGreen, HIGH);
            break;

        case RgbColor::Green:
            digitalWrite(pins.rgbGreen, HIGH);
            break;

        case RgbColor::Blue:
            digitalWrite(pins.rgbBlue, HIGH);
            break;

        case RgbColor::Violet:
            digitalWrite(pins.rgbRed, HIGH);
            digitalWrite(pins.rgbBlue, HIGH);
            break;

        case RgbColor::Off:
            // already off
            break;
    }
}

/* ------------------------------------------------------------ */
/*                   BUZZER Implementation                      */
/* ------------------------------------------------------------ */

Buzzer::Buzzer(const PinMap& pins, unsigned long duration, RGBLed* led)
    : pins(pins), beepDuration(duration), rgbLed(led), isBeeping(false), previousMillis(0) {}

void Buzzer::begin()
{
    pinMode(pins.buzzer, OUTPUT);
    beep(100);
    Serial.println("Buzzer initialized");
}

void Buzzer::beep(unsigned long duration)
{
    beepDuration = duration;       
    isBeeping = true;
    previousMillis = millis();     
    digitalWrite(pins.buzzer, HIGH); 

    if (rgbLed)
        rgbLed->setColor(RGBLed::RgbColor::Red);
}

void Buzzer::update()
{
    if (isBeeping && (millis() - previousMillis >= beepDuration))
    {
        digitalWrite(pins.buzzer, LOW); // turn off after time passes
        isBeeping = false;
    }
}

void Buzzer::refreshOutput()
{
    if (isBeeping) 
    {
        digitalWrite(pins.buzzer, HIGH);
    } 
    else 
    {
        digitalWrite(pins.buzzer, LOW);
    }
}

/* ------------------------------------------------------------ */
/*                   BUTTON Implementation                      */
/* ------------------------------------------------------------ */

Button::Button(const PinMap& pins)
    : pins(pins), lastDebounceTime(0), buttonPressed(false)
{

}

void Button::begin()
{
    pinMode(pins.buttonReset, INPUT_PULLUP);
    pinMode(pins.buttonPause, INPUT_PULLUP);
    pinMode(pins.buttonPlay, INPUT_PULLUP);
    pinMode(pins.buttonStep, INPUT_PULLUP);

    Serial.println("Buttons initialized");
}

Button::ButtonEvent Button::handleInput()
{
    static bool lastResetState  = HIGH;
    static bool lastPauseState  = HIGH;
    static bool lastPlayState   = HIGH;
    static bool lastStepState   = HIGH;

    static unsigned long pulseStartTime = 0;
    static bool pulseActive = false;

    unsigned long currentTime = millis();

    bool resetState = digitalRead(pins.buttonReset);
    bool pauseState = digitalRead(pins.buttonPause);
    bool playState  = digitalRead(pins.buttonPlay);
    bool stepState  = digitalRead(pins.buttonStep);

    // --- Handle counter reset pulse timing (non-blocking) ---
    if (pulseActive && (currentTime - pulseStartTime >= 100)) {
        digitalWrite(pins.counterReset, LOW);
        pulseActive = false;
    }

    // Simple debounce: ignore changes until debounceDelay has passed
    if (currentTime - lastDebounceTime < debounceDelay)
        return ButtonEvent::None;

    // Detect button press transition (HIGH -> LOW)
    if (resetState == LOW && lastResetState == HIGH) 
    {
        lastDebounceTime = currentTime;
        lastResetState = resetState;
        Serial.println("Reset pressed");

        // start a short counter reset pulse
        digitalWrite(pins.counterReset, HIGH);
        pulseStartTime = currentTime;
        pulseActive = true;

        return ButtonEvent::ResetPressed;
    }
    if (pauseState == LOW && lastPauseState == HIGH) 
    {
        lastDebounceTime = currentTime;
        lastPauseState = pauseState;
        Serial.println("Pause pressed");
        return ButtonEvent::PausedPressed;
    }
    if (playState == LOW && lastPlayState == HIGH) 
    {
        lastDebounceTime = currentTime;
        lastPlayState = playState;
        Serial.println("Play pressed");
        return ButtonEvent::PlayPressed;
    }
    if (stepState == LOW && lastStepState == HIGH) 
    {
        lastDebounceTime = currentTime;
        lastStepState = stepState;
        Serial.println("Step pressed");
        return ButtonEvent::StepPressed;
    }

    // Save last states for the next call
    lastResetState = resetState;
    lastPauseState = pauseState;
    lastPlayState  = playState;
    lastStepState  = stepState;

    return ButtonEvent::None;
}

bool Button::getButtonStatus()
{
    return buttonPressed;
}







