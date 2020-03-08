# Array antenna emulator for satellite communications with STM32F1 and AD9959

This program generates the signals received by the elements of a 1x4 array antenna when a LEO satellite passes over the antenna broadside transmitting a sinusoidal signal.

It is thought to be a standalone solution, so no computer is needed during the execution.
Just change the parameters and variables of the code, program the microcontroller, and if connected correctly to an AD9959, the simulation should be running and the DDS should output the aforementioned signals.

## Hardware connections

The pins of the ADD9959 must be connected to the microcontroller as shown in *conections.png*.

To change the pin connection, please, modify the #defines at *ad9957.ino* file.

## STM32 microcontroller with Arduino libraries

Follow this [guide](https://www.luisllamas.es/programar-stm32-con-ide-de-arduino-y-st-link-v2/) (in Spanish) to use STM32 microcontroller with Arduino IDE and libraries with ST-Link v2.
Download before ST-Link drivers from this [link](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-link009.html#get-software).

## Model variables

There are three variables that determine the model.

The flying altitude of the satellite *h*, the base frequency of the transmitted signal *f0*, and the distance between array elements *d_elem*.

These variables are defined at *signals_model.ino* and their default values are:

h = 650.0       // [km]

f0 = 150e6      // [Hz]

d_elem = 1.0    // [m]

## Time or theta scanning

The embedded model can be set to generate the signals as a function of time or theta, the position of the satellite.

This is controlled by the *#define TIME_SCANNING* at *array_antenna_emulator.ino*.
If uncommented, the model will be time dependent, if commented, it will be theta dependent.

## Model refresh rate

The model is updated with a Timer interrupt.
The *#define INTERRUPT_PERIOD* at *array_antenna_emulator.ino* controls the Timer's interrupt period, in microseconds, and thus the refresh rate of the emulator.

Setting this value to 1000 gives a refresh rate of 1 kHz.

There are some print commands in the simulation. If the refresh rate is 1 kHz or faster, they **will not work correctly**, please, comment them.

## Time acceleration

The *#define TIME_ACCEL* at *array_antenna_emulator.ino* multiplies the time steps in the model.

By setting this value to 1, we will get a real-time simulation, that is, the model will run at the same time as the actual physical system.
If set to a higher value, the model will run faster than the physical system.
