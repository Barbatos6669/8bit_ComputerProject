/**
 * @file main.ino
 * @brief Clock program
 * @author HobbyHacker
 * @version 0.1.1
 * @date 2025-10-20
 *
 * @details
 * This is an experimental project to help me and maybe others learn and better understand
 * how a computer works by utilizing an ATmega and other integrated circuits
 * to construct the basic computational components of a simple CPU system.
 */

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
 * ```cpp
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
        HeartBeat(int pin, unsigned long ms = 500);

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

        void setInterval(unsigned long ms);
        unsigned long getInterval() const;
        bool getState() const;

    private:
        int ledPin;
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
    enum class RgbColor {Red, Orange, Yellow, Green, Blue, Violet};

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
    RGBLed(int rPin, int gPin, int bPin);

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

    int redLedPin; ///< Assigned GPIO pin for controlling the RGB LED or color channel.
    int greenLedPin; ///< Assigned GPIO pin for controlling the RGB LED or color channel.
    int blueLedPin; ///< Assigned GPIO pin for controlling the RGB LED or color channel.
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
        MachineState();

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
        State currentState;
        HeartBeat heartbeat; 
        RGBLed rgbLed;
};

/* ------------------------------------------------------------ */
/*                  GLOBAL OBJECT CONSTRUCTED                   */
/* ------------------------------------------------------------ */

MachineState machineState;

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
MachineState::MachineState() 
    :   currentState(State::Reset), 
        heartbeat(13, 500), 
        rgbLed(3, 5, 7)
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

    // State Transition
    setState(State::Run);
}

void MachineState::handleInput()
{
    // Handle all input
    // Serial.println("Handling Input"); // Muted
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
    
    heartbeat.update();
}

void MachineState::refreshOutput()
{
    // handle output and end cycle
    // Serial.println("Tick"); Muted
    heartbeat.refreshOutput();
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

HeartBeat::HeartBeat(int pin, unsigned long ms)
    : ledPin(pin), previousMillis(0), interval(ms), isOn(false) {}

void HeartBeat::begin()
{
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
}

void HeartBeat::update()
{

}

void HeartBeat::refreshOutput()
{
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval)
    {
        Serial.print("Previous Time: "); Serial.println(previousMillis);        
        previousMillis = currentMillis;
        Serial.print("Elapsed Time: "); Serial.println(currentMillis);
        
        isOn = !isOn;
        digitalWrite(ledPin, isOn ? HIGH : LOW);
    }
    else if (interval == 0)
    {
        digitalWrite(ledPin, LOW);
    }

}

void HeartBeat::stepBeat()
{
    // Manual logic to beat the heart
}

void HeartBeat::setInterval(unsigned long ms)
{
    interval = ms;
}

/* ------------------------------------------------------------ */
/*                   RGBLed Implementation                      */
/* ------------------------------------------------------------ */

RGBLed::RGBLed(int rPin, int gPin, int bPin)
    : redLedPin(rPin), greenLedPin(gPin), blueLedPin(bPin)
{

}





