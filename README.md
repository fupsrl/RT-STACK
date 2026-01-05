# RT-STACK
V12 GDI ready real-time ECU stack

<img width="1724" height="1058" alt="RT_STACK_front" src="https://github.com/user-attachments/assets/7c1804ee-9972-4c92-b98b-580d52981549" />
<img width="1724" height="1058" alt="RT_STACK_back" src="https://github.com/user-attachments/assets/91e5549d-2c8a-48a6-b664-45340de67f9b" />

# Features

* 12 Injector drivers (general purpose GDI or PFI)
* 12 Inductive spark channels
* 9 General purpose triggers (can be used for crank/cams wheel, turbocharger sensor, ...)
* High voltage DCDC for injector drive
* 2 CJ125 lambda controllers
* STM32G4 cpu for automatic injector drive with analog comparators and DACs
* 186 pin standard ECU connector
* 12 general purpose output (push, pull selectable)

All the features of the projects can be used standalone for the most basic applications.
However the board is designed to be shielded to the raspberry breakout board to enable complex and high precision engine controls.

# Component selection

* STM32G474VETx for CPU
* TO-252-2 mosfet for injection driver
* IR2101 gate driver for high sides
* TC4427 gate driver for low sides
* ACS722 for injectors current sensing
* RGPR30BM40 for ignition IGBT
* IFX007TAUMA1 for push/pull GPO
* ADCMP354 comparator for trigger edge detection (compatible up to 22V)
* CJ125 for lambda control
