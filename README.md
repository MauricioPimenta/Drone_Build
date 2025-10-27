# Drone_Build
Code and Hardware implementations for the drones built for the Laboratory of Autonomous Intelligent Robots - LAB-AIR, from the Federal University of Espírito Santo (UFES). This project is in active development, using a Teensy 4.0 microcontroller as the main board with Arduino IDE.

## VS-CODE development
To use vscode as the IDE for development instead of the Arduino IDE, we recommend installing the following extensions:
- [PlatformIO IDE](platformio.org)  **[Necessary]**
- Arduino Community Edition  **[Necessary]**
- arduino-snippets    **[Recommended]**
- serial-plotter      **[Recommended]**

### [PlatformIO IDE](platformio.org)
To install this extension, read and follow the instruction on their [documentation page](https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-ide-for-vscode).
The most important steps are described below:

#### Preparations
In a Linux Terminal, check your Python version using the command:
```
python3 --version
```
and install the python3-venv package in your system if it isn't installed:
```
sudo apt install python3.xx-venv
```
Example: `sudo apt install python3.8-venv`

#### Install from vscode Extensions Tab
Search for **PlatformIO** inside Visual Studio Code *Extensions* Tab and install the official [platformio ide extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).

Check the [PlatformIO QuickStart Guide](https://docs.platformio.org/en/latest/integration/ide/vscode.html#quick-start) to learn the basics.

