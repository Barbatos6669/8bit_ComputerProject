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
    //Serial.println("Handling Input"); // Muted
}

void MachineState::update()
{
    switch(currentState)
    {
        case State::Reset: heartbeat.setInterval(500); break;
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

}

void HeartBeat::stepBeat()
{
    // Manual logic to beat the heart
}

void HeartBeat::setInterval(unsigned long ms)
{
    interval = ms;
}




