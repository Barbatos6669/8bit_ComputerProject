```markdown
# 8-bit Computer Project

A small experimental repository for building and running an 8-bit-style computer using Arduino and standalone ATmega microcontrollers. This repo contains sketches, notes, and wiring experiments used while prototyping on breadboards.

## Quick overview
- Use the included Main.ino sketch to run basic experiments on an Arduino board.
- You can also program a standalone ATmega (e.g., ATmega328P) using an Arduino as an ISP programmer.
- This README gives clear, step-by-step setup instructions, parts list, wiring tips, and troubleshooting notes.

## Parts & tools (suggested)
- 1x Arduino Uno (or compatible) — used as the development board and/or ISP programmer
- 1x ATmega328P (or similar AVR) — if running standalone
- Breadboards (several recommended for prototyping)
- Jumper wires
- 16 MHz crystal (or resonator) + 22pF caps (if using an external clock)
- 10kΩ pull-up resistor (RESET)
- 0.1 µF decoupling capacitors for Vcc/GND
- External power supply (if powering breadboards separately)
- Optional: LEDs, resistors, switches, and other components for I/O experiments

## File layout
- Main.ino — primary sketch to load to an Arduino (copy/paste into Arduino IDE or open the file).
- /examples or other folders — any example sketches or helper code (check repo tree).

## Uploading Main.ino to an Arduino (fastest method)
1. Open Arduino IDE.
2. Create a new sketch or open Main.ino from this repo.
3. Select the correct board under Tools > Board and the correct COM/serial port.
4. Click Upload (or Sketch > Upload). The sketch will be compiled and sent to the Arduino.

## Programming a standalone ATmega with "Arduino as ISP"
These steps assume you're using an Arduino Uno as the programmer and an ATmega328P on a breadboard.

1. In Arduino IDE, open the example sketch: File > Examples > 11.ArduinoISP > ArduinoISP and upload it to the Arduino you’ll use as the programmer.
2. Wire the Arduino to the target ATmega:
   - MISO -> MISO
   - MOSI -> MOSI
   - SCK  -> SCK
   - RESET (target) -> pin 10 (on programmer Arduino) to use as programmer RESET (or follow wiring for your chosen board)
   - Vcc -> Vcc (5V) and GND -> GND
   - Add crystal and caps if using an external clock; add decoupling capacitors on Vcc/GND
   - Add 10k pull-up resistor on RESET of target
3. In the IDE, set:
   - Tools > Board to the target chip/board (e.g., "ATmega328 on breadboard (8 MHz internal)" if you have a custom cores package, or "Arduino Uno" if the fuse/clock is set accordingly)
   - Tools > Programmer > "Arduino as ISP"
4. Optional but recommended: Burn bootloader first (Tools > Burn Bootloader) — this sets fuses and clock configuration.
5. To upload your sketch to the target chip via the programmer, use: Sketch > Upload Using Programmer (or hold Shift when clicking Upload, depending on IDE version).

Note: If using an Arduino Uno as the programmer, place a 10 µF electrolytic capacitor between RESET and GND on the programmer board to disable auto-reset (negative to RESET, positive to GND). This prevents the Uno from resetting when the IDE opens the serial port.

## Wiring & power notes
- Power the target circuit reliably. If powering both the Arduino programmer and the breadboard from USB, ensure common ground.
- Check Vcc pins and decoupling capacitors (0.1 µF) close to the AVR power pins.
- If using an external crystal, the correct caps (typically 22pF) are required.
- If you see "avrdude: stk500_recv(): programmer is not responding" or sync errors, re-check wiring and ensure the ArduinoISP sketch is running on the programmer.

## Troubleshooting tips
- No serial output: confirm correct baud in sketch and Serial Monitor.
- "Programmer is not responding": confirm MOSI/MISO/SCK/RESET wiring and that the ArduinoISP sketch is uploaded to the programmer board.
- Fuse/clock mismatch: if the target's fuses expect an external clock and it's not present, provide a clock or use a chip with the correct fuse settings.
- Use a multimeter to check for 5V at Vcc and proper GND connections.

## Contributing
If you have improvements, wiring diagrams, corrected code, or new experiments, please open an issue or submit a pull request. Add clear notes about hardware used, wiring, and the exact sketch tested.

## License
No license file included. Add a LICENSE if you want to set reuse permissions (MIT, Apache-2.0, etc.).

## Contact / Next steps
If you want, I can:
- Clean up and commit this README to the repository,
- Add a short wiring diagram image and example photos,
- Convert Main.ino into a top-level sketches folder with a usage example.

If you'd like me to commit these changes, tell me whether to create a new branch or update the README on the default branch.
```
