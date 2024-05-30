grblCR: a fork of grbl for the Coast Runner CNC

grbl is a standards-compliant g-code controller that will look up to 16 motions into the future to deliver smooth acceleration and jerk-free cornering.

* [Licensing](https://github.com/gnea/grbl/wiki/Licensing): GrblCR is free software, released under the GPLv3 license.

* Built on the wonderful Grbl v1.1c (2019) firmware written by Sungeun "Sonny" Jeon, Ph.D. (USA) aka @chamnit

* Built on the wonderful Grbl v0.6 (2011) firmware written by Simen Svale Skogsrud (Norway).

***
## SETTING UP GRBLCR IN ARDUINO STUDIO

These instructions are somewhat cribbed from the master grbl Build docs (https://github.com/gnea/grbl/wiki/Compiling-Grbl)

1. Install Arduino IDE 2.x (these instructions are not written for 1.x) 
1. Clone or download grblCR from this repo.
1. Locate the "grblCR" subfolder inside of the grblCR repo. NOTE: you should see a file called "grbl.h" inside this folder.
1. Copy the entire "grblCR" subfolder into your Arduino Libraries folder. The Libraries folder can be found as follows:
  * Windows: <User>/Documents/Arduino/libraries
  * Mac: ~/Documents/Arduino/libraries
  You can also find this folder by opening the Arduino IDE preferences (File > Preferences) and looking at the path shown in the "Sketchbook location" field there.

  The correct installation structure must look like this:

  ```text
    Arduino/
    ├── libraries/
    │   └── grbl/
    │       ├── examples/
    │       │   └── ...
    │       ├── grbl.h
    │       └── ...
    └── ...
    ```

You are now set up to build and / or upload grblCR to an Arduino Uno 328p.

WHEN MAKING CHANGES TO THE GRBLCR SOURCE CODE, MAKE SURE THOSE CHANGES ARE MADE TO THE CODE IN THE SUBFOLDER THAT WE COPIED INTO THE ARDUINO LIBRARIES. Don't copy the folder, forget you did it, and then make changes in a different copy, and then wonder why your changes aren't working.

***

### COMPILING AND UPLOADING GRBLCR

These instructions will show you how to upload the compiled grblCR firmware directly to the Arduino Uno 328p.

1. Launch Arduino IDE
1. Select File > Examples > grblCR > grblUpload from the Arduino IDE menus.
  * ⚠ Do not alter this example in any way! Grbl does not use any Arduino code. Altering this example may cause compilation to fail.
1. Plug in your Arduino Uno to the computer
1. Ensure that Tools > Board > Arduino AVR Boards > Arduino Uno is selected. It should automatically select when your Uno is plugged in, but double check to make sure.
1. Select the port of your Uno from the Tools > Port menu. Again, this should auto-select, but check to be sure.
1. Select Sketch > Verify / Compile from the Arduino IDE menus to check for errors.
1. If no errors are found, select Sketch > Upload to upload to the Uno 328p.

***

### RETRIEVING THE COMPILED HEX FILE

Sometimes you may want to grab the compiled hex file that is uploaded to the Arduino.
To do so, run the COMPILE AND UPLOAD instructions above, and then do the following:

1. Open the IDE Output window. It should still have the logs from the last upload.
1. Scroll up and find the avrdude section of the logs. It should be one of the most recent log sections.
1. In this section you should see a line that starts with "avrdude: reading input file" followed by a filepath.
  You can also copy the output and search "grblUpload.ino.hex", which should also turn up the filepath.
1. Navigate to this filepath on your computer to find the hex file. Note that this is a temporary location, which Arduino deletes when you close the grblUpload file.

***

```
List of Supported G-Codes:
  - Non-Modal Commands: G4, G10L2, G10L20, G28, G30, G28.1, G30.1, G53, G92, G92.1
  - Motion Modes: G0, G1, G2, G3, G38.2, G38.3, G38.4, G38.5, G80
  - Feed Rate Modes: G93, G94
  - Unit Modes: G20, G21
  - Distance Modes: G90, G91
  - Arc IJK Distance Modes: G91.1
  - Plane Select Modes: G17, G18, G19
  - Tool Length Offset Modes: G43.1, G49
  - Cutter Compensation Modes: G40
  - Coordinate System Modes: G54, G55, G56, G57, G58, G59
  - Control Modes: G61
  - Program Flow: M0, M1, M2, M30*
  - Spindle Control: M3, M4, M5
  - (CRWrite only) Find midpoint between two WCS (see manual): M100
  - (CRWrite only) Verify absolute distance is less than x (see manual): M101
  - Stepper High power mode (next motion command only): M17
  - Stepper Zero power mode (turn steppers off until next motion command): M18
  - Valid Non-Command Words: F, I, J, K, L, N, P, R, S, T, X, Y, Z
  - '|' (shift+\) now resets grbl (in addition to control+x, which is not a keyboard character).
```

