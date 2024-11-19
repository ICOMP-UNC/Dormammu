# Installation and Setup Guide
## Overview
This guide provides detailed instructions for building, flashing, and running the firmware on the target hardware. It includes information about the required tools, dependencies, and supported platforms.

## Prerequisites
Before you begin, ensure you have the following tools and dependencies installed:

- PlatformIO: A cross-platform build system for embedded development.
- Visual Studio Code: Required for PlatformIO.
- ST-Link: A tool for flashing the firmware onto the target hardware.
- Git: For cloning the repository.
Make sure the following software versions are installed:

- PlatformIO: Latest version.
- Visual Studio Code: Version 1.x.x or higher.
- ST-Link: Ensure drivers are up to date for your platform.
- Git: Version 2.x or higher.
## Supported Platforms (tested on)
- Operating Systems: Windows, Linux
- Microcontroller: STM32F103T8
## Installation Steps
### 1. Install Visual Studio Code
If you don’t already have Visual Studio Code installed, you can download it from here. After installation, open VSCode and proceed to the next step.

### 2. Install PlatformIO
PlatformIO is a cross-platform build system for embedded development. To install it, follow these steps:

Open Visual Studio Code.
Go to the Extensions view by clicking the Extensions icon in the Activity Bar.
Search for PlatformIO IDE and click Install.
For more information on PlatformIO, refer to [their website](https://platformio.org/).

### 3. Clone the Repository
Clone the project repository to your local machine by running the following command in your terminal:

`git clone https://github.com/ICOMP-UNC/Dormammu.git`

`cd Dormammu`

### 4. Build the Firmware
Use PlatformIO's project tasks to build the firmware. In Visual Studio Code:

Open the PlatformIO sidebar.
Click Build (check the platformio.ini file for board and environment configurations).

### 5. Flash the Firmware
After building the firmware, flash it onto the target hardware using ST-Link. Make sure you have selected the correct STM32 board in the platformio.ini file (e.g., genericSTM32F103T8).

To flash the firmware using PlatformIO:

- In the PlatformIO sidebar, click Upload.

If you encounter issues with the ST-Link connection, ensure the drivers are properly installed and the correct interface is selected in the platformio.ini file.

### 6. Running the Firmware
After flashing the firmware, the microcontroller will automatically start running the firmware. To monitor the UART output, use a serial terminal program like PuTTY or Tera Term. Configure the terminal settings as follows:

- Speed: 115200 baud
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow Control: XON/XOFF
For PuTTY users, make sure to enable the following settings in Session > Line discipline options:

- Implicit CR in every LF: Checked
- Local echo: Force on
- Local line editing: Force off


## Additional Information
For more detailed information about the project, refer to the README.md file and the generated Doxygen documentation in the docs/ directory.



