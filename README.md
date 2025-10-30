# 8-bit Computer Project

This repository serves as a small experimental collection for building and running an 8-bit-style computer using Arduino and standalone ATmega microcontrollers. It contains sketches, notes, and wiring experiments developed during breadboard prototyping.

## Quick Overview
*   Use the included `Main.ino` sketch to run basic experiments on an Arduino development board.
*   You can also program a standalone ATmega (e.g., ATmega328P) using an Arduino acting as an In-System Programmer (ISP).
*   This `README.md` provides clear, step-by-step setup instructions, a parts list, wiring tips, and troubleshooting notes.

## Parts & Tools (Suggested)
*   1x Arduino Uno (or compatible) — used as the development board and/or ISP programmer
*   1x ATmega328P (or similar AVR microcontroller) — if running standalone
*   Breadboards (several recommended for prototyping)
*   Jumper wires
*   16 MHz crystal (or resonator) + 22 pF capacitors (if using an external clock)
*   10kΩ resistor (for ATmega `RESET` pin pull-up)
*   0.1 µF ceramic decoupling capacitors (for VCC/GND)
*   External power supply (if powering breadboards separately)
*   Optional: LEDs, resistors, switches, and other components for I/O experiments

## File Layout
*   `Main.ino` — The primary sketch to load onto an Arduino development board. Open directly in the Arduino IDE.
*   `/examples` or other folders — Any example sketches or helper code. Check the repository tree for specifics.

## Uploading `Main.ino` to an Arduino (Fastest Method)
1.  Open the Arduino IDE.
2.  Create a new sketch or open `Main.ino` from this repository.
3.  Select the correct board under `Tools > Board` and the correct `COM Port` / `Serial Port`.
4.  Click `Upload` (or `Sketch > Upload`). The sketch will be compiled and uploaded to the Arduino.

## Programming a Standalone ATmega with "Arduino as ISP"
These steps assume you're using an Arduino Uno as the programmer and an ATmega328P on a breadboard.

1.  In the Arduino IDE, open the example sketch: `File > Examples > 11.ArduinoISP > ArduinoISP` and upload it to the Arduino you'll use as the programmer.
2.  Wire the Arduino programmer to the target ATmega:
    *   `Arduino MISO (pin 12)` -> Target ATmega `MISO`
    *   `Arduino MOSI (pin 11)` -> Target ATmega `MOSI`
    *   `Arduino SCK (pin 13)` -> Target ATmega `SCK`
    *   `Arduino D10` -> Target ATmega `RESET` pin (or follow specific wiring for your chosen programmer board)
    *   `Arduino 5V` -> Target ATmega `VCC`
    *   `Arduino GND` -> Target ATmega `GND`
    *   Add 16 MHz crystal and 22 pF capacitors if using an external clock.
    *   Add 0.1 µF decoupling capacitors on VCC/GND for the ATmega.
    *   Add 10kΩ pull-up resistor from Target ATmega `RESET` to `VCC`.
3.  In the IDE, set:
    *   `Tools > Board` to the target chip/board (e.g., "ATmega328 on breadboard (8 MHz internal)" if you have a custom cores package, or "Arduino Uno" if the fuse/clock is set accordingly).
    *   `Tools > Programmer > "Arduino as ISP"`.
4.  Optional but recommended: Burn bootloader first (`Tools > Burn Bootloader`) — this sets fuses and clock configuration.
5.  To upload your sketch to the target chip via the programmer, use: `Sketch > Upload Using Programmer` (or hold `Shift` when clicking `Upload`, depending on your IDE version).

Note: If using an Arduino Uno as the programmer, place a 10 µF electrolytic capacitor between the programmer Arduino's `RESET` pin and `GND`. Ensure the capacitor's **negative lead is connected to GND and the positive lead to the RESET pin**. This disables auto-reset, preventing the programmer Arduino from resetting when the IDE opens the serial port.

## Wiring & Power Notes
*   Power the target circuit reliably. If powering both the Arduino programmer and the breadboard from USB, ensure a common ground connection.
*   Check `VCC` pins and ensure 0.1 µF ceramic decoupling capacitors are placed close to the AVR microcontroller's power pins.
*   If using an external crystal, the correct 22 pF capacitors are required.
*   If you see `"avrdude: stk500_recv(): programmer is not responding"` or similar sync errors, re-check all wiring and ensure the `ArduinoISP` sketch is running on the programmer board.

## Troubleshooting Tips
*   **No serial output:** Confirm correct baud rate in your sketch and in the Serial Monitor.
*   **"Programmer is not responding":** Confirm `MOSI`/`MISO`/`SCK`/`RESET` wiring and that the `ArduinoISP` sketch is uploaded to the programmer board.
*   **Fuse/clock mismatch:** If the target's fuses expect an external clock and it's not present, provide a clock or use a chip with the correct fuse settings.
*   **Power issues:** Use a multimeter to check for 5V at `VCC` and proper `GND` connections.

## Contributing
If you have improvements, wiring diagrams, corrected code, or new experiments, please open an issue or submit a pull request (PR). Add clear notes about hardware used, wiring, and the exact sketch version tested.

## License
Currently, no `LICENSE` file is included in this repository. Add a `LICENSE` file if you want to set reuse permissions (e.g., `MIT`, `Apache-2.0`).