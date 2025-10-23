/**
 * @file main.ino
 * @brief Clock program
 * @author HobbyHacker
 * @version 0.1.0
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
        void tick();

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
     * @brief Constructs a new RGBLed object.
     * 
     * @param pin The digital pin connected to the RGB LED module or control channel.
     * 
     * @details
     * Initializes the RGB LED instance but does not configure the pin hardware.
     * Call @ref begin() before using other functions.
     */
    RGBLed(int pin);

    /**
     * @brief Initializes the LED hardware.
     * 
     * @details
     * Sets the assigned RGB LED pin(s) as OUTPUT and ensures all channels
     * are turned off at startup. Should be called once in setup().
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
    void setColor();

private:
    int rgbLedPin; ///< Assigned GPIO pin for controlling the RGB LED or color channel.
};


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
        * @brief Executes time-based or output update operations.
        * @details
        * Called at the end of each loop cycle to perform periodic actions
        * that depend on timing or visual feedback. Typical uses include
        * blinking LEDs, generating heartbeat signals, or playing timed tones.
        * 
        * @note This method should run after update() to ensure all logic
        * has been processed before outputs are refreshed.
        */
        void tick();

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
};

/* ------------------------------------------------------------ */
/*                  Global Object Constructed                   */
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
    machineState.tick();
}

/* ------------------------------------------------------------ */
/*                    CLASS IMPLEMENTATION                      */
/* ------------------------------------------------------------ */

/// MachineState
MachineState::MachineState() 
    :   currentState(State::Reset), 
        heartbeat(13, 500)
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

void MachineState::tick()
{
    // handle output and end cycle
    // Serial.println("Tick"); Muted
    heartbeat.tick();
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

/// HeartBeat
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

void HeartBeat::tick()
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




