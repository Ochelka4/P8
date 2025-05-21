## Control of Asynchronous motor and drive
This project includes all the code from the 2. semester project in EMSD by group 2.002.

## PWM Signals
This code configures a nucleo board to determine PWM signals for the VFD. It is therefore written in C code and the PWM signals is determined using V/F control. The code also includes the control for the peripherals of the Nucleo board, which includes: ADC, GPIO, a timer and interrupts. 

The main for the code is located in: PWM signals/Core/Src/Main

## SVPWM
This is a matlab file that simulates SVPWM. It uses a reference AC signal, along with a DC voltage and modulation period, to generate switching sequences for the VFD that modulate the reference signal.
