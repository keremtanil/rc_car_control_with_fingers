# Hand Gesture Controlled Robot Car Project

This project was developed as part of the Digital Image Processing course in the Computer Engineering Department at Adnan Menderes University. The aim of the project is to wirelessly control a robot car using hand gestures detected in real-time via a camera.

[![Project Test Video - Classroom](https://img.youtube.com/vi/vu64colVOqY/0.jpg)](https://youtu.be/vu64colVOqY)

*Final project test in the classroom. See the "Visuals and Videos" section below for more videos.*

## Overview

The system detects the user's hand using a camera connected to a computer. Using Python and the OpenCV library, the number of fingers extended or whether a fist is made is identified. This information is then translated into specific commands (forward, backward, right, left, stop) and sent wirelessly via an HC-05 Bluetooth module to an Arduino UNO. The Arduino, in turn, controls the motors of a 4WD robot car through an L298N motor driver based on the received commands.

## Features

*   Real-time hand gesture detection (finger counting and fist detection).
*   Image processing and contour analysis using OpenCV.
*   Command generation and serial/Bluetooth communication using Python.
*   Wireless control via HC-05 Bluetooth module.
*   Motor control using Arduino UNO.
*   4WD car control with L298N motor driver.
*   Commands assigned to specific finger counts:
    *   **0 Fingers (Fist):** STOP
    *   **1 Finger:** MOVE BACKWARD
    *   **2 Fingers:** TURN RIGHT
    *   **3 Fingers:** TURN LEFT
    *   **4 or 5 Fingers:** MOVE FORWARD

## Technologies and Materials Used

### Hardware
*   **Microcontroller:** Arduino UNO R3 - SMD Clone (CH340 Chip)
*   **Wireless Communication:** HC-05 Bluetooth Module Board
*   **Motor Driver:** L298N Dual Motor Driver Board - Voltage Regulated
*   **Motors and Wheels:** 4 x 6V 250 Rpm DC Motor And Wheel Set
*   **Chassis:** 4WD Car chassis
*   **Power Supply:** 7.4V 2S Lipo Battery-Pil 2800 mAh 35C
*   **Camera:** A standard USB Webcam (accessed via `cv2.VideoCapture(0)` in the code)
*   **Other:** Electronic Breadboard (Large Size - 830 Pin), Mini Breadboard (170 Pin), Jumper Wires (Female-Female, Female-Male, Male-Male)

### Software
*   **Programming Languages:** Python 3.x, Arduino (C/C++)
*   **Libraries (Python):**
    *   OpenCV (`cv2`) - For image processing
    *   NumPy (`numpy`) - For numerical operations
    *   PySerial (`serial`) - For serial communication (also used for Bluetooth connection)
    *   `collections.deque` - For stabilizing finger count
    *   `time` - For timing operations
*   **Development Environment:** Arduino IDE, a preferred Python IDE (e.g., VS Code, PyCharm)

## Setup and Execution

### 1. Hardware Connections
1.  Mount the motors and wheels onto the 4WD chassis.
2.  Connect the L298N motor driver to the Arduino (PWM and direction pins).
3.  Connect the motors to the L298N motor driver.
4.  Connect the HC-05 Bluetooth module to the Arduino (TX-RX, RX-TX cross-connection, and VCC, GND).
    *   *Refer to the pin definitions in the Arduino code for default HC-05 connections (often pins 10 and 11 for SoftwareSerial).*
5.  Connect the Lipo battery to power the L298N motor driver and the Arduino (either via VIN pin or USB).
6.  Connect the webcam to your computer.

### 2. Software Installation
1.  **Python Libraries:**
    ```bash
    pip install opencv-python numpy pyserial
    ```
2.  **Arduino IDE:** Download and install the Arduino IDE from the [official website](https://www.arduino.cc/en/software).
3.  **Arduino Code:**
    *   Open the `.ino` Arduino code file from this repository with the Arduino IDE.
    *   Select the correct board (Arduino UNO) and port.
    *   Upload the code to the Arduino.
    *   Ensure the baud rate of the HC-05 Bluetooth module matches the Arduino code and Python script (typically 9600 or 38400). The PDF visuals show 38400 baud for HC-05, while the Python code uses 9600. Verify this consistency.

### 3. Running the System
1.  **Bluetooth Pairing:** Pair your computer with the HC-05 module via your computer's Bluetooth settings (default password is usually "1234" or "0000").
2.  **Python Script:**
    *   Update the serial port name in the Python script (`arduino = serial.Serial('COM3', 9600)`) to match the virtual serial port of your HC-05 on your computer (can be found in Device Manager on Windows or with `ls /dev/tty.*` on Linux/macOS).
    *   Run the Python script:
        ```bash
        python <your_script_name>.py
        ```
3.  The camera will open and start detecting your hand. The car should move according to your hand gestures.
4.  Press 'q' while the image window is active to quit the program.

## Code Structure (Example)

*   `main_control_script.py`: The main Python script containing image processing with OpenCV, gesture detection, and command sending to Arduino.
*   `robot_car_arduino.ino`: The code uploaded to the Arduino, which receives commands via Bluetooth and drives the motors.

## Challenges Faced and Solutions

*   **Orange Light Issue:** Orange lights in the house made it difficult for the camera to detect the hand accurately.
    *   **Solution:** The ambient lighting was changed to white lights, significantly improving skin tone detection accuracy.
*   **Face Detection Problem:** While trying to detect the hand, the camera sometimes identified the face as a contour.
    *   **Solution:** Contour detection was disabled for the top third of the camera frame, ensuring that only contours at hand level were detected.
*   **Missing Screws for the Chassis:** The chassis provided for hardware assembly did not include screws to secure the motors.
    *   **Solution:** Screws compatible with the chassis and other missing components were sourced from an industrial area.
*   **Using Double-Sided Tape:** A temporary solution was needed to attach the motors to the chassis while waiting for suitable screws.
    *   **Solution:** Double-sided tape was used to temporarily secure the motors to the chassis.

## Limitations

1.  **Ineffectiveness with Short-Sleeved Clothing:** The system may not work effectively when short-sleeved clothing is worn, as it might confuse the arm with the hand during contour detection.
2.  **Instability with Different Skin Tones:** The algorithm struggles to deliver stable results across various skin tones.
3.  **Incompatibility with Gloves or Accessories:** The system does not produce accurate results when gloves or other hand accessories are worn, as they alter the hand's shape and color.
4.  **Distance Sensitivity:** Due to camera limitations, accuracy decreases as the hand moves farther away from the camera.
5.  **Single-Hand Detection:** The system is designed to work with only one hand and cannot handle scenarios where both hands are used simultaneously.
6.  **Background Dependency:** The color of the clothing worn and the background plays a significant role. If the background or clothing color is similar to skin tones, detection accuracy drops.
7.  **Pre-Calibration for Positioning:** Before running the system with Arduino, it is recommended to run the code without Arduino to establish the initial position and calibration.
8.  **Fixed Camera Position Required:** The camera must remain stationary for the system to function correctly.
9.  **Works with Specific Finger Movements:** The system is optimized to work with certain specific finger movements and may not respond accurately to other hand gestures or random finger positions.

## Visuals and Videos

*   **Bluetooth Connections:** [https://youtube.com/shorts/wJF8DTcW8Yw](https://youtube.com/shorts/wJF8DTcW8Yw)
*   **Bluetooth Pairing (HC-05 AT Commands):** [https://youtube.com/shorts/VBzrtugx6I0](https://youtube.com/shorts/VBzrtugx6I0)
*   **Software Testing (Finger Detection):** [https://youtube.com/shorts/ZWpGjaC2EpU](https://youtube.com/shorts/ZWpGjaC2EpU)
*   **Car Movements Test:** [https://youtube.com/shorts/dAbZIFf_iVA](https://youtube.com/shorts/dAbZIFf_iVA)
*   **Final Test in Classroom:** [https://youtu.be/vu64colVOqY](https://youtu.be/vu64colVOqY)

## Contributors

*   **Developer:** Köksal Kerem TANIL
*   **Advisor:** Dr. Öğr. Üyesi Mahmut SİNECEN

## License

This project is licensed under the [MIT License](LICENSE.md). (Remember to add a `LICENSE.md` file to your project. MIT is a good starting point.)
