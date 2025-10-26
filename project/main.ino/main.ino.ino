/**
 * @file main.ino
 * @brief Clock program
 * @author HobbyHacker
 * @version 0.1.4
 * @date 2025-10-20
 *
 * @details
 * This is an experimental project to help me and maybe others learn and better understand
 * how a computer works by utilizing an ATmega and other integrated circuits
 * to construct the basic computational components of a simple CPU system.
 */

/* ------------------------------------------------------------ */
/*                      PIN CONFIGUARATION                      */
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
    2,  // counter reset pin
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
 * @brief Provides a non-blocking blinking or pulse signal for system feedback.
 *
 * @details
 * The HeartBeat class is used to create a timed pulse or LED blink that
 * represents a system heartbeat. It uses `millis()` for timing, so it does not
 * block the main loop — allowing other code to run simultaneously.
 * 
 * ### Behavior
 * - Toggles a digital pin at a regular interval (default 500 ms).
 * - Setting the interval to 0 disables the heartbeat (output forced LOW).
 * - Useful for showing system status, timing, or “alive” signals.
 * 
 * ### Usage Example
 * HeartBeat heartbeat(13, 500);  // LED on pin 13, 500ms blink interval
 *
 * void setup() {
 *     heartbeat.begin();
 * }
 *
 * void loop() {
 *     heartbeat.update();
 *     heartbeat.tick();
 * }
 * ```
 *
 * ### Notes
 * - @ref update() should be called continuously inside `loop()`.
 * - @ref tick() performs the actual output toggle.
 * - @ref stepBeat() can be used to manually trigger a pulse.
 */
class HeartBeat
{
    public:
        /**
         * @brief Default constructor.
         * @details
         * Initializes internal timing variables but does not start the heartbeat.
         * Call @ref begin() to configure the output pin and start operation.
         */
        HeartBeat(const PinMap& pins, unsigned long ms = 500);

        /**
         * @brief Prepares the heartbeat system for operation.
         * @details
         * Configures the designated output pin (such as an LED) and initializes
         * internal timing values. Should be called once during setup().
         */
        void begin();

        /**
         * @brief Updates the heartbeat logic.
         * @details
         * Called continuously within the main loop to check timing intervals
         * and determine whether a pulse event should occur. Does not block.
         */
        void update();

        /**
         * @brief Performs a timed output pulse or LED toggle.
         * @details
         * Called when the heartbeat interval has elapsed to produce the
         * actual output effect (such as toggling an LED or producing a short
         * tone). Intended to be triggered from @ref update().
         */
        void refreshOutput();

        /**
         * @brief Forces an immediate heartbeat pulse.
         * @details
         * Can be used for diagnostic purposes or to synchronize subsystems
         * with an external event.
         */
        void stepBeat();

        void pulseOnce(unsigned long duration = 50);

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
 * @brief Controls an RGB LED to display various color states or visual feedback.
 *
 * @details
 * The RGBLed class provides an interface for driving a tri-color (Red, Green, Blue)
 * LED module using three digital pins. Each color channel can be toggled independently
 * or used in combination to produce composite colors.
 *
 * This class is designed for use in embedded systems where an RGB LED represents
 * system states (e.g., Reset, Run, Pause) or provides general visual feedback.
 *
 * ### Example Usage
 * 
 * RGBLed statusLed(9, 10, 11);
 * 
 * void setup() {
 *     statusLed.begin();
 *     statusLed.setColor(RGBLed::rgbColor::Green);
 * }
 * 
 * void loop() {
 *     statusLed.update();
 * }
 *
 */
class RGBLed
{
public:
    /**
     * @enum rgbColor
     * @brief Enumerates predefined color states for the RGB LED.
     *
     * @details
     * Each enum value represents a basic color that can be produced
     * by combining the red, green, and blue channels.
     */
    enum class RgbColor {Red, Orange, Yellow, Green, Blue, Violet, Off};

    /**
    * @brief Creates a new RGB LED controller.
    * 
    * @param rPin The digital pin connected to the red channel of the LED.
    * @param gPin The digital pin connected to the green channel of the LED.
    * @param bPin The digital pin connected to the blue channel of the LED.
    * 
    * @details
    * This constructor sets up an RGB LED object and stores the pin numbers for each color channel.
    * It does not configure the pins or send any signals yet — call @ref begin() in setup() 
    * to initialize the hardware before changing colors.
    */
    RGBLed(const PinMap& pins);

    /**
    * @brief Prepares the RGB LED for use.
    *
    * @details
    * This function tells the microcontroller which pins control the LED’s red,
    * green, and blue channels. It also turns all channels off to start from
    * a known state. Think of it as “plugging in the LED and turning it off”
    * before any colors are shown.
    *
    * Should be called once in setup().
    */
    void begin();

    /**
     * @brief Handles input or interaction logic.
     * 
     * @details
     * Intended for reading input (such as buttons or machine states) that
     * influence the LED’s behavior. This method can be expanded to interpret
     * system signals and automatically change the LED color.
     */
    void handleInput();

    /**
     * @brief Updates LED state or timing logic.
     * 
     * @details
     * Called continuously within the main loop to refresh LED output,
     * animate color transitions, or apply timed effects.
     * 
     * In its base form, it can be left empty until timing-based features
     * (like fades or blinking) are implemented.
     */
    void update();

    /**
     * @brief Retrieves the current LED color.
     * 
     * @details
     * Returns the currently active color, which can be used for
     * debugging, display updates, or synchronized subsystem behavior.
     */
    void getColor();

    /**
     * @brief Changes the LED color output.
     * 
     * @details
     * Applies a new color by adjusting the LED output channels. The implementation
     * can either use predefined enum colors (from @ref rgbColor) or direct RGB values
     * depending on future expansion.
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
 * @brief Provides non-blocking tone or beep functionality.
 *
 * @details
 * The Buzzer class allows timed beeps using millis() instead of delay(),
 * ensuring that system timing and other logic continue to run smoothly.
 *
 * ### Behavior
 * - Call @ref beep(duration) to start a timed beep.
 * - @ref update() automatically turns it off after the duration expires.
 * - Can be tied into @ref refreshOutput() for synchronized system updates.
 *
 * ### Example
 * ```cpp
 * Buzzer buzzer(7, 100);
 * 
 * void setup() {
 *     buzzer.begin();
 * }
 * 
 * void loop() {
 *     buzzer.update();
 *     buzzer.refreshOutput();
 * }
 * ```
 */
class Buzzer
{
    public:
        /**
        * @brief Creates a new buzzer controller.
        * @param pin The digital pin connected to the buzzer.
        * @param duration The default duration (ms) of each beep.
        */
        Buzzer(const PinMap& pins, unsigned long duration, RGBLed* led = nullptr);

        /**
        * @brief Initializes the buzzer pin and ensures it starts off.
        */
        void begin();

        /**
        * @brief Starts a beep for a given duration.
        * @param duration The duration (ms) the buzzer stays on.
        */
        void beep(unsigned long duration);

        /**
        * @brief Updates the buzzer logic.
        * @details
        * Checks if the beep duration has elapsed and stops the buzzer automatically.
        * Must be called frequently in the main loop.
        */
        void update();

        /**
        * @brief Outputs the buzzer state to hardware.
        * @details
        * Writes HIGH or LOW to the buzzer pin based on current state.
        */
        void refreshOutput();        

        /**
        * @brief Returns whether the buzzer is currently active.
        * @return true if beeping, false otherwise.
        */
        bool getState();

    private:
    RGBLed* rgbLed;
    const PinMap& pins;

    int buzzerPin; ///< Output pin connected to the buzzer.
    unsigned long beepDuration; ///< Duration of current beep.
    unsigned long previousMillis; ///< Timestamp for non-blocking timing.
    bool isBeeping; ///< Whether the buzzer is currently active.
};

/* ------------------------------------------------------------ */
/*                        BUTTON CLASS                          */
/* ------------------------------------------------------------ */

/**
 * @class Button
 * @brief Handles input from multiple physical buttons for machine control.
 *
 * @details
 * The Button class provides an interface for reading and managing the state of 
 * multiple momentary push buttons, such as those used to control machine functions 
 * (Reset, Pause, Play, Step). It reads the button states through digital inputs and 
 * interprets which control action was triggered.
 * 
 * All button pins are configured with `INPUT_PULLUP`, meaning they read **HIGH** when 
 * not pressed and **LOW** when pressed. This eliminates the need for external resistors 
 * and ensures stable logic levels.
 *
 * ### Behavior
 * - Reads up to four control buttons (Reset, Pause, Play, Step).
 * - Uses an enum-based event system to simplify event handling.
 * - Designed for non-blocking reads and continuous polling within the main loop.
 * - Can be extended later to include **debouncing** or **long-press detection**.
 *
 * @note 
 * - Uses `INPUT_PULLUP`, so a pressed button reads LOW.
 * - For stable operation with mechanical buttons, consider adding a 
 *   software debounce delay or hardware RC filter.
 * - Use @ref handleInput() in the main loop to continuously read states.
 */
class Button
{
    public:
        /**
        * @enum ButtonEvent
        * @brief Represents the specific action triggered by a button press.
        *
        * @details
        * Each event corresponds to one of the available control buttons.
        * The `None` value indicates that no buttons are currently pressed.
        */
        enum class ButtonEvent{ None, ResetPressed, PausedPressed, PlayPressed, StepPressed};
        
        /**
        * @brief Creates a new Button manager.
        *
        * @param resetBtnPin The pin connected to the Reset button.
        * @param pauseBtnPin The pin connected to the Pause button.
        * @param playBtnPin The pin connected to the Play button.
        * @param stepBtnPin The pin connected to the Step button.
        *
        * @details
        * This constructor stores the assigned pin numbers for each button.
        * It does not configure the pins yet — call @ref begin() in setup() 
        * before calling @ref handleInput().
        */
        Button(const PinMap& pins);

        /**
        * @brief Initializes button pins for input.
        *
        * @details
        * Configures all connected button pins as `INPUT_PULLUP`, ensuring 
        * they default to a HIGH state when not pressed. This prevents 
        * floating inputs and allows direct wiring to ground.
        *
        * Should be called once in `setup()`.
        */
        void begin();

        /**
        * @brief Reads button inputs and determines which event occurred.
        *
        * @details
        * This method performs a quick scan of all connected buttons and 
        * returns the first detected press as a `ButtonEvent`.  
        * 
        * It is designed for simple polling — call it once per loop cycle.
        * Currently, there is no debouncing; future revisions may add timing-based filtering.
        *
        * @return A value of @ref ButtonEvent indicating which button was pressed.
        */
        ButtonEvent handleInput();

        
        /**
        * @brief Placeholder for future input logic or debouncing.
        * 
        * @details
        * Currently unused. Reserved for possible input stabilization or 
        * multi-button logic (e.g., long press, double-click).
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
 * @brief Controls the overall flow of the machine.
 * @details
 * The MachineState class manages the current operational mode of the system,
 * handling transitions such as Reset, Pause, Run, and Step. It acts as the
 * central controller that determines how other subsystems (LEDs, buzzer,
 * heartbeat, etc.) behave based on the active state.
 */
class MachineState
{
    public:
        /**
        * @enum State
        * @brief Represents the current operating mode of the machine.
        * @details
        * Each state corresponds to a distinct phase of system operation:
        * - **Reset** — Initializes or restarts the system.
        * - **Pause** — Temporarily halts execution.
        * - **Run** — Normal operating state.
        * - **Step** — Executes a single instruction or cycle.
        */
        enum class State {Reset, Pause, Run, Step};

        /**
        * @brief Default constructor.
        * @details
        * Initializes the machine in the Reset state.
        */
        MachineState(const PinMap& pins);

        /**
        * @brief Initializes subsystems or hardware.
        * @details
        * Configures serial, LEDs, and other components.
        * Called once in setup() before the main loop begins.
        */
        void begin();

        /**
        * @brief Reads and processes external input signals.
        * @details
        * Captures button presses or sensor values and stores them 
        * for later use by the update() method. This function should 
        * be called at the beginning of each main loop cycle to ensure 
        * that all user interactions are registered before logic is processed.
        */
        void handleInput();

        /**
        * @brief Updates the machine logic and subsystem states.
        * @details
        * Executes one update cycle for the machine, applying logic based on 
        * the current system state (Reset, Pause, Run, or Step). This function 
        * is responsible for coordinating all subsystems, such as LEDs, buzzer, 
        * and heartbeat, to ensure synchronized behavior.
        * 
        * Typical flow:
        * - Evaluates the current machine state.
        * - Applies any pending transitions from handleInput().
        * - Updates subsystem outputs accordingly.
        * 
        * @note Should be called continuously from loop() after handleInput().
        */
        void update();

        /**
        * @brief Sends all changes out to the hardware.
        *
        * @details
        * This runs after all updates have been calculated.
        * It makes LEDs blink, buzzers beep, and anything else
        * that should respond to the current state actually happen.        */
        void refreshOutput();

        /**
        * @brief Retrieves the current operating state of the machine.
        * @details
        * Returns the current system state (Reset, Pause, Run, or Step)
        * so that other components or debug routines can query and respond
        * to the active mode of operation.
        * 
        * @return The current machine state as a value of the @ref State enum.
        */
        State getState() const;

        /**
        * @brief Updates the machine's operating state.
        * @details
        * Changes the system state to the specified new state. If the new state
        * differs from the current one, a transition occurs and a debug message
        * is printed to the serial monitor to indicate the change.
        * 
        * Typical uses include:
        * - Switching between Run and Pause modes.
        * - Resetting the system to a known state.
        * - Entering Step mode for manual clock cycles.
        * 
        * @param newState The desired next state of the machine.
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
    // Start in Reset mode (will transition to Run in begin)
}

void MachineState::begin()
{
    // Initialize Subsystems and hardware
    delay(500); // Waiting half a second
    Serial.begin(9600);
    Serial.println("MachinState Intialized, port 9600");

    // Hardware initialization.....
    heartbeat.begin();
    rgbLed.begin();
    buzzer.begin();
    button.begin();

    // State Transition
    setState(State::Run);
}

void MachineState::handleInput()
{
    // Handle all input
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

                setState(State::Pause);  
                bool stepHeld = (digitalRead(pins.buttonStep) == LOW);
                heartbeat.setOutput(stepHeld);
            }
            break;
        case Button::ButtonEvent::None: 
            {

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

    // --- handle step hold behaviour ---
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
    // handle output and end cycle
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

        Serial.print("State Changed to: ");
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

HeartBeat::HeartBeat(const PinMap& pins, unsigned long ms = 500)
    : pins(pins), previousMillis(0), interval(ms), isOn(false) {}

void HeartBeat::begin()
{
    pinMode(pins.heartbeatLed, OUTPUT);
    pinMode(pins.counterReset, OUTPUT);
    digitalWrite(pins.heartbeatLed, LOW);
    digitalWrite(pins.counterReset, LOW);

    Serial.println("HeartBeat initialized....");
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
    // Manual logic to beat the heart
    digitalWrite(pins.heartbeatLed, HIGH);
    delay(10);
    digitalWrite(pins.heartbeatLed, LOW);
}

void HeartBeat::pulseOnce(unsigned long duration = 50)
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
    : pins(pins)
{

}

void RGBLed::begin()
{
    pinMode(pins.rgbRed, OUTPUT);
    pinMode(pins.rgbGreen, OUTPUT);
    pinMode(pins.rgbBlue, OUTPUT);

    setColor(RGBLed::RgbColor::Red);
    getColor();

    Serial.print("RGBLed Initialized");
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
        default:               Serial.println("Unknown"); break;
    }
}

void RGBLed::setColor(RgbColor color)
{
    // Start with all colors off
    digitalWrite(pins.rgbRed, LOW);
    digitalWrite(pins.rgbGreen, LOW);
    digitalWrite(pins.rgbBlue, LOW);

    // Store the current color
    currentRGBLedColor = color;

    switch(color)
    {
        case RgbColor::Red:
            {
                digitalWrite(pins.rgbRed, HIGH);
            }            
            break;

        case RgbColor::Orange:
            {
                digitalWrite(pins.rgbRed, HIGH);
                analogWrite(pins.rgbGreen, 128); // mix red + half green
            }
            break;

        case RgbColor::Yellow:
            {
                digitalWrite(pins.rgbRed, HIGH);
                digitalWrite(pins.rgbGreen, HIGH);
            }
            break;

        case RgbColor::Green:
            {
                digitalWrite(pins.rgbGreen, HIGH);
            }
            break;

        case RgbColor::Blue:
            {
                digitalWrite(pins.rgbBlue, HIGH);
            }
            break;

        case RgbColor::Violet:
            {
                digitalWrite(pins.rgbRed, HIGH);
                digitalWrite(pins.rgbBlue, HIGH);
            }
            break;
    }
}

/* ------------------------------------------------------------ */
/*                   BUZZER Implementation                      */
/* ------------------------------------------------------------ */

Buzzer::Buzzer(const PinMap& pins, unsigned long duration, RGBLed* led)
    : pins(pins), beepDuration(duration), rgbLed(led) {}

void Buzzer::begin()
{
    pinMode(pins.buzzer, OUTPUT);
    beep(100);
    Serial.print("Buzzer Initialized...");
}

void Buzzer::beep(unsigned long duration)
{
    beepDuration = duration;       
    isBeeping = true;
    previousMillis = millis();     
    digitalWrite(pins.buzzer, HIGH); 

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
    : pins(pins)
{

}

void Button::begin()
{
    pinMode(pins.buttonReset, INPUT_PULLUP);
    pinMode(pins.buttonPause, INPUT_PULLUP);
    pinMode(pins.buttonPlay, INPUT_PULLUP);
    pinMode(pins.buttonStep, INPUT_PULLUP);

    Serial.print("Buttons Initialized");
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

    // --- Handle pulse timing ---
    if (pulseActive && (currentTime - pulseStartTime >= 100)) {
        digitalWrite(pins.counterReset, LOW);
        pulseActive = false;
    }

    // debounce timer
    if (currentTime - lastDebounceTime < debounceDelay)
        return ButtonEvent::None;

    // Detect button press transition (HIGH → LOW)
    if (resetState == LOW && lastResetState == HIGH) 
    {
        lastDebounceTime = currentTime;
        lastResetState = resetState;
        Serial.println("Reset Pressed.....");

        // start pulse
        digitalWrite(pins.counterReset, HIGH);
        pulseStartTime = currentTime;
        pulseActive = true;

        return ButtonEvent::ResetPressed;
    }
    if (pauseState == LOW && lastPauseState == HIGH) 
    {
        lastDebounceTime = currentTime;
        lastPauseState = pauseState;
        Serial.println("Pause Pressed.....");
        return ButtonEvent::PausedPressed;
    }
    if (playState == LOW && lastPlayState == HIGH) 
    {
        lastDebounceTime = currentTime;
        lastPlayState = playState;
        Serial.println("Play Pressed.....");
        return ButtonEvent::PlayPressed;
    }
    if (stepState == LOW && lastStepState == HIGH) 
    {
        lastDebounceTime = currentTime;
        lastStepState = stepState;
        Serial.println("Step Pressed.....");
        return ButtonEvent::StepPressed;
    }

    // Save last states
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









