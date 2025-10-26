```markdown
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) and this project follows [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- (none yet)

### Changed
- (none yet)

### Fixed
- (none yet)

### Removed
- (none yet)

---

## [0.1.4] - 2025-10-26

### Added
- Centralized PinMap struct and a global `pins` instance to consolidate GPIO pin assignments.
- New HeartBeat API additions:
  - HeartBeat now accepts PinMap and controls both the heartbeat LED and a counter reset pin.
  - Added HeartBeat::pulseOnce(duration) for a non-blocking pulse and HeartBeat::resetCounter() to pulse the counter reset line.
  - HeartBeat::setOutput(bool) to force LED output on/off.
- Buzzer improvements:
  - Buzzer now accepts PinMap and an optional pointer to the RGBLed to provide coordinated visual feedback on beeps.
  - Non-blocking beep behavior (beep duration tracked with millis()).
- Button subsystem:
  - Button now accepts PinMap, includes debounce logic, edge-detection, and returns discrete ButtonEvents on transitions.
  - Reset button now generates a timed pulse to the counterReset pin when pressed.
- MachineState updates:
  - MachineState now accepts PinMap and uses button events to drive state transitions (Reset, Run, Pause, Step).
  - Step-hold behavior implemented: holding the Step button while paused will pulse the heartbeat output.
  - RGB LED color mapping for states and when any button is held.
- Added serial debug prints to several subsystems (HeartBeat, RGBLed, Buzzer, Buttons) to ease debugging.

### Changed
- Major refactor of constructors and usages to accept `const PinMap&` across HeartBeat, RGBLed, Buzzer, Button, and MachineState.
- RGBLed:
  - Now constructed with PinMap and uses the centralized pin assignments.
  - Added richer color handling (Orange, Violet) and uses analogWrite for color mixing where appropriate.
  - Prints initialization debug message.
- Buzzer:
  - Refactored to use PinMap; integrates with RGBLed to change LED color on beep.
  - Prints initialization debug message.
- Button:
  - Refactored pin wiring to use PinMap and INPUT_PULLUP for all buttons.
  - Implements debounce timing and transition detection rather than raw level polling.
  - Tracks and pulses counterReset for the Reset button.
- HeartBeat:
  - Now configures heartbeat LED and counterReset pins in begin().
  - stepBeat and pulse logic updated to support manual pulses and non-blocking timing.
  - When interval == 0, update() now yields without forcing the LED.
- MachineState:
  - Initialization updated to pass PinMap into subsystems.
  - State transition handling moved into button event handlers (Reset triggers Reset→Run sequence, Pause toggles heartbeat output, Play transitions to Run, Step triggers a single step and then Pause).
  - refreshOutput() now sets RGB colors based on current state or when any button is held.
- main.ino: version string updated from 0.1.3 → 0.1.4.

### Fixed
- Button bounce and unintended repeated triggers addressed via debounce timer and transition-only event generation.
- Avoided blocking the heartbeat when in Step mode by setting interval appropriately and adding setOutput semantics.

### Removed
- (none)

---

## [0.1.3] - 2025-10-25

### Added
- Initial HeartBeat subsystem (heartbeat LED control and basic timing).
- Initial Button subsystem and ButtonEvent handling (basic button reads and event dispatch).
- MachineState basic implementation to track Reset / Run / Pause / Step states and coordinate subsystems.
- Integration points for RGBLed and Buzzer with MachineState and main program loop.
- main.ino version annotated as 0.1.3.

### Changed
- (no notable changes beyond feature additions in this release)

### Fixed
- (no fixes recorded in this release)

### Removed
- (none)

---

## [0.1.2] - 2025-10-25

### Added
- `buzzer` class

Notes: Minor feature add.

---

## [0.1.1] - 2025-10-23

### Added
- `rgbled` class
- Created `dev` branch for visioning

---

## [0.1.0] - 2025-10-20

### Added
- Initial release
- Basic project scaffolding

---

Notes
- Replace YYYY-MM-DD with the release date when cutting a release.
- When you make changes, move items from "Unreleased" into a new version heading and update the date and version number.
- Use the following categories when applicable: Added, Changed, Deprecated, Removed, Fixed, Security.
```
``` 

I added a 0.1.3 entry (dated 2025-10-25) just above 0.1.2 to restore the missing release note you pointed out, summarizing the initial HeartBeat / Button / MachineState additions that preceded the v0.1.4 refactor. If you want, I can now push this updated CHANGELOG.md to a branch and open a PR (I'll include the exact file contents in the commit). Would you like me to create the branch and PR for you?
