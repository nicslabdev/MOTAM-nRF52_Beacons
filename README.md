# **Nordic Semiconductor nRF52 beacons** #  

MOTAM project uses nRF52840 PDK by Nordic Semiconductor in order to improve some aspects of the developed beacons like more security or a longer coverage range.

  

This is possible thanks to some of the new improvements of Bluetooth 5: long range and extended advertisements.

  

## Requirements ##

  

Currently, this project uses:

- nRF5 SDK version 15.3.0

- S140 SoftDevice v6.1.1 API

- nRF52840 PDK (PCA10056) and nRF52840 Dongle (PCA10059)

- Particle Boron

- Particle deviceOS@1.2.1-rc.1

  

## Get started ##

### Nordic Semiconductor Development Kit

Each project has a hex folder with the precompiled application and its SoftDevice hex file.
In order to program the device (nRF52840 PDK or Dongle), just use the Programmer application of Nordic Semiconductor [nRF Connect Desktop](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF-Connect-for-Desktop). 
1. Program the SoftDevice (included in hex folder).
2. Program the application: PCA10056 hex file for nRF52840 PDK and pca10059 for nRF52840 Dongle.

If you modify the code and you need to recompile the code, put the folders inside nRF52 into the  \nRF5_SDK_15.3.0\MOTAM_apps* folder. 
There are several ways for developing code in nRF52, we have done this with [SEGGER Embedded Studio for ARM V4.12](https://infocenter.nordicsemi.com/pdf/getting_started_ses.pdf).

### Particle Boron

Particle Boron code has been developed using Particle Workbench. May be necessary [leaving listening mode](https://community.particle.io/t/boron-constantly-in-listening-mode/45882/30) on Particle Boron.

## Applications' description

-  **Intelligent Traffic Light**: This application controls a traffic light with improved characteristics. We have develop a prototype of this, that looks like a standard traffic light, but inside it a nRF52840 is working transmitting through Bluetooth Long Range (PHY CODED) the information in real time about the state of the traffic light (what light is turned on) and what time left for the next state. This information is transmitted in a secured frame by means of cryptographic signature and the MOTAM PKI.

LTE Cat M1 connection is stablished with Particle Boron. Serial communication between Particle Boron and nRF52840 allows integration between cloud data and BLE broadcasting.

![Intelligent Traffic Light prototype](https://i.imgur.com/85sdWkW.png)