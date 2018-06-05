# **Nordic Semiconductor nRF52 beacons** #

Here, you can find MOTAM beacons which has been developed for nRF52.

MOTAM project uses nRF52840 PDK by Nordic Semiconductor in order to improve some aspects of the developed beacons like more security or a longer coverage range.

This is possible thanks to the new improvements of Bluetooth 5: long range and extended advertisements.

## nRF52 software version ##

Currently, this project use:

- nRF5 SDK v15.0.0
- S140 SoftDevice v6.0.0 API

## Get started ##

To compile it, clone the repository into the *\nRF5_SDK_15.0.0\examples\ble_peripheral* folder.

## Applications' description ##

- **Stop_Sign**:  This application is the starting point of BT5 secure beacon development. It generates a secure signature, in order to authenticate beacon data, and send a beacon frame that includes data and the generated digital signature. For this, ECDSA cryptographic algorithm is used.

> Note: Currently, Stop_Sign development is stopped. With the current nRF5 SDK is not possible to use extended advertisements. The development of this application will be resumed with the release of the next version of nRF5 SDK. [More info here](https://devzone.nordicsemi.com/f/nordic-q-a/34695/how-to-enable-bluetooth-5-extended-advertisement). In the meantime, you can try with *Stop_Sign-Detached_Signature.*

- **Stop_Sign-Detached_Signature**: This is an alternative to Stop_Sign application developed for devices that don't support extended advertisements. This beacon generates a secure signature, in order to authenticate beacon data, and send two linked frames that include data and a secure digital signature. ECDSA cryptographic algorithm is used for this.

> Note: Currently under development. An alpha version of *Stop_Sign-Detaches_Signature* will be released soon.