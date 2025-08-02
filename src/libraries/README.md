# Libraries for the HaHaHo Interactive Pole

A few generic code modules are split off as libraries, so that they can be used in both the pole-firmware and the standalone sketches.

## Use in Arduino IDE

Beware: the Arduino IDE might be familiar but it is unbearably slow for ESP32 development. While building in the Arduino IDE can take tens of seconds, it only takes seconds in PlatformIO. However, this git repo maintains compatibiity with the Arduino IDE to provide newcomers with an easy start.

In the Arduino IDE preferences find the default sketch location (e.g. c:|Users\username\Documents\Arduino in Win11 or $HOME/Arduino in Linux). Copy the folders in the src/libraries folder of this git repo to the libraries folder of the Arduino IDE default sketch location.
Now your sketch can have the following statements:
```c
# include <web_ota.h>>
```

Instead of copying the files, it is possible to create a symbolic link from a library in the Arduino libraries folder to corresponding library in the git repo libraries folder. Using an "as administrator" CMD terminal in Microsoft Windows:
```bat
mklink /d c:\Users\username\Documents\Arduino\libraries\Interactive-Pole-Utils c:\Users\username\Projects\hahaho-interactive-pole\src\libraries\Interactive-Pole-Utils
```

## Use in Platform IO

In the platformio.ini file of a /standalone script add the following section:

```ini
[platformio]
src_dir = .
lib_dir = ../../../src/libraries
```
In the platformio.ini file of a complete /src/poleN firmware add the following section:

```ini
[platformio]
src_dir = .
lib_dir = ../libraries
```
