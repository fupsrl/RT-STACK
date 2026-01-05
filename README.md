# RT-STACK
The ultimate Engine Control Unit (ECU) platform capable of taming any Internal Combustion Engine (ICE) out there.

The core philosophy driving this development is:
* **Keep it Simple:** Easy to pick up if you just want it to run, deep enough if you want to tinker.
* **Universal:** Adapt the software architecture to your needs and control virtually anything: 2-stroke, 4-stroke, Wankel, Diesel... you name it.
* **Unlimited Power:** By leveraging a Raspberry Pi for high-level control, you can say goodbye to RAM and CPU bottlenecks. Plus, with Simulink support, you can use graphical programming and run your engine without writing a single line of code.
* **Pro-Grade:** Don't let the simplicity fool you. The design requirements are built to handle complex beasts, from turbocharged setups to high-revving naturally aspirated V12s.
* **Platforms independent:** Measurement and Calibration with XCP protocol (a2l + hex). Can be used with OEM softwares like INCA/CANAPE (free version available) or with any other software like Vehicle Spy, Open vector XCP stack, ...

_Disclaimer: The board is not intended to be used on road legal vehicles and it is provided wihtout any warranty._

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
