/**
 * @file main.ino
 * @brief Clock program
 * @author HobbyHacker
 * @version 0.1.0
 * @date 2025-10-20
 *
 * @details
 * This is an experimental project to help me learn and better understand
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

    private:


};

/* ------------------------------------------------------------ */
/*                      MAIN PROGRAM FLOW                       */
/* ------------------------------------------------------------ */

void setup()
{

}

void loop()
{

}

/* ------------------------------------------------------------ */
/*                    CLASS IMPLEMENTATION                      */
/* ------------------------------------------------------------ */


MachineState::MachineState()
{
    // Start in run mode
}


void MachineState::begin()
{
    // Initialize Subsystems and hardware
}

void MachineState::handleInput()
{
    // Handle all input
}

void MachineState::update()
{
    // Update program based on input
}

void MachineState::tick()
{
    // handle output and end cycle
}



