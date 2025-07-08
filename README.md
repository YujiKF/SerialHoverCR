# Project Description

This repository contains the files **needed to control the speed** of a **Gen2.1.1 Hoverboard board** (its generation is defined by [RoboDurden in this repository](https://https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/wiki)). It also allows you to **receive telemetry information** like current, speed, and voltage.

All the code is written in **C++**, leveraging the serial communication library from the [Lucidar website, by Lulu](https://lucidar.me/en/serialib/cross-plateform-rs232-serial-library/).

---

## How to use

1.**Ensure Correct Firmware:** Make sure your Hoverboard board is flashed with a compatible custom firmware, such as the [MasterUart from RoboDurden repository.](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/blob/main/BinariesReadyToFlash/hoverboard%202.1.1%20master%20Uart.bin)
2.**Connect Hardware:**
    *Connect the **RX pin** from the Hoverboard board to the **TX pin** on your USB-UART converter.
    *Connect the **TX pin** from the Hoverboard board to the **RX pin** on your USB-UART converter.
    *Ensure both devices share a common **GND (ground)** connection.
4.  **Configure COM Port:**
    *Open the main C++ source file (`main.cpp`).
    *Locate the line where the serial port is initialized (line 15, `#define SERIAL_PORT "\\.\\COM3"`).
    * Change **`"COM3"`** to the actual COM port assigned to your USB-UART converter. You can find the correct port in Device Manager by looking under the 'Ports (COM & LPT)' section and see which one disappears when you disconnect the converter.
5.  **Compile the Code:**
    *Navigate to the project's root directory in your terminal.
    *Compile the code using `g++`, making sure to link the `serialib` library. The command might look something like this:
        ```bash
        g++ -o hoverboard.exe main.cpp serialib.cpp
        ```
        *(**Note:** The exact compilation command depends on your project's structure and how `serialib` is included. You might need to adjust `main.cpp`, `serialib.cpp`, and any include/library paths.)*
6.  **Run the Application:**
    *Execute the compiled program from your terminal:
        ```bash
        .\hoverboard.exe
        ```
