# Libraries for the HaHaHo Interactive Pole

A few generic code modules are split off as libraries, so that they can be used in both the pole-firmware and the standalone sketches.

## Use in Arduino IDE

In the Arduino IDE preferences find the default sketch location (e.g. %USER_PROFILE%/Documents/Arduino in Win11 or $HOME/Arduino in Linux). Copy the folders in the src/libraries folder of this git repo to the libraries folder of the Arduino IDE default sketch location.
Now your sketch can have the following statements:
```c
# include <web_ota.h>>
```

## Use in Platform IO

Tbd
