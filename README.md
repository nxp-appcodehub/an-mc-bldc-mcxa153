# NXP Application Code Hub

[`<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>`](https://www.nxp.com)

## Sensorless BLDC motor control on MCXA153

This application note demonstrates how to implement the six-step commutation control of brushless direct current(BLDC) motor on the MCXA family of MCUs. The NXP Freedom board FRDM-MC-LVBLDC
is used as a hardware platform for the motor
control reference solution. The hardware-dependent part of the motor control
software is addressed as well, including a detailed peripheral setup and driver
description. The FreeMASTER operation is also described in this document.

#### Boards: FRDM-MCXA153

#### Categories: Motor Control

#### Peripherals: ADC, PWM

#### Toolchains: MCUXpresso IDE

## Table of Contents

1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [FAQs](#step5)
6. [Support](#step6)
7. [Release Notes](#step7)

## 1. Software

- Download and install [MCUXpresso IDE V11.9.0 or later](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE).
- Download and install the latest version of [FreeMASTER](https://www.nxp.com/design/software/development-software/freemaster-run-time-debugging-tool:FREEMASTER)(3.2.2.2).
- Download the code from Git repository an-mc-bldc-mcxa153 [!--[an-mc-bldc-mcxa153](https://github.com/nxp-appcodehub/an-mc-bldc-mcxa153/browse)--](!--%5Ban-mc-bldc-mcxa153%5D(https://github.com/nxp-appcodehub/an-mc-bldc-mcxa153/browse)--).
- MCUXpresso for Visual Studio Code: This example supports MCUXpresso for Visual Studio Code, for more information about how to use Visual Studio Code please refer [here](https://www.nxp.com/design/training/getting-started-with-mcuxpresso-for-visual-studio-code:TIP-GETTING-STARTED-WITH-MCUXPRESSO-FOR-VS-CODE).

## 2. Hardware

- [FRDM-MCXA153 Development Board](https://www.nxp.com/design/design-center/development-boards/general-purpose-mcus/frdm-development-board-for-mcx-a14x-a15x-mcus:FRDM-MCXA153).
- [FRDM-MC-LVBLDC](https://www.nxp.com/design/design-center/development-boards/general-purpose-mcus/nxp-freedom-development-platform-for-low-voltage-3-phase-bldc-motor-control:FRDM-MC-LVBLDC).
- [LINIX 45ZWN24 motor](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/low-voltage-3-phase-motor-for-frdm-platform:FRDM-MC-LVMTR).
- Personal Computer
- USB type-C cable and 12V adaptor.

## 3. Setup

1. Open MCUXpresso IDE, in the Quick Start Panel, choose **Import from Application Code Hub**

   ![](image/import_project_1.png)
2. Enter the demo name in the search bar.

   ![](image/import_project_2.png)
3. Click **Copy GitHub link**, MCUXpresso IDE will automatically retrieve project attributes, then click **Next>**.

   ![](image/import_project_3.png)
4. Select **main** branch and then click **Next>**, Select the MCUXpresso project, click **Finish** button to complete import.

   ![](image/import_project_4.png)
5. Click **Build** to start compiling the project.

   ![](image/build_project.png)

### 3.2 Step 2

Connect the **FRDM-MC-LVBLDC** shield to the **J1~J4** arduino connector of **FRDM-MCXA153** board. Connect the 3-phase wire of the motor to the J7 connector on FRDM-MC-LVBLDC according to phase sequence(**White** wide--phase**A**; **Bule** wide--phase**B**; **Green** wide--phase**C**). Power the FRDM-MC-LVBLDC board with a 12V adaptor.
![hardware_connection](image/hardware_connection.png)
Compile the project and use a USB cable to connect to the EVK board via **J15** connector. Use MCU-LINK to download program into MCU.

### 3.3 Step 3

Compile the project and download program into MCU.  Press SW2 on FRDM board to control the motor to run or stop. Or use FreeMASTER project **"bldc_frac.pmpx"** in the code package to control the motor, change rotor speed and obverse the speed or other value.

## 4. Results

The FreeMASTER can be used to completely control the motor control application. The operation in FreeMASTER are described more closely herE.
Select the right com in FreeMASTER project, in Project->Options-Comm:
![FM_COM](image/FM_COM.png)
Operation state and speed command can be configured in FreeMASTER.
![FM_operation](image/FM_operation.png)

## 5. FAQs

No FAQs have been identified for this project.

## 6. Support

#### Project Metadata

<!----- Boards ----->

[![Board badge](https://img.shields.io/badge/Board-FRDM–MCXA153-blue)](https://github.com/search?q=org%3Anxp-appcodehub+FRDM-MCXA153+in%3Areadme&type=Repositories)

<!----- Categories ----->

[![Category badge](https://img.shields.io/badge/Category-MOTOR%20CONTROL-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+motor_control+in%3Areadme&type=Repositories)

<!----- Peripherals ----->

[![Peripheral badge](https://img.shields.io/badge/Peripheral-ADC-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+adc+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-PWM-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+pwm+in%3Areadme&type=Repositories)

<!----- Toolchains ----->

[![Toolchain badge](https://img.shields.io/badge/Toolchain-MCUXPRESSO%20IDE-orange)](https://github.com/search?q=org%3Anxp-appcodehub+mcux+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

> **Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected funcionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/@NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/Twitter-Follow%20us%20on%20Twitter-white.svg)](https://twitter.com/NXP)

## 7. Release Notes

| Version | Description / Update                    |                                    Date |
| :-----: | --------------------------------------- | --------------------------------------: |
|   1.0   | Initial release on Application Code Hub | February 23 `<sup>`rd `</sup>` 2024 |
