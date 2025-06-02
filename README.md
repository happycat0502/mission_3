# Line-Following Robot with Raspberry Pi and Arduino

This project implements a line-following robot that uses a Raspberry Pi for computer vision-based line detection and an Arduino for precise motor control. The Raspberry Pi processes images from a camera to identify the line and sends control commands to the Arduino, which then drives the robot's motors.

The system supports both autonomous line following and manual remote control. A web interface provides a camera feed and status information.

## Arduino Firmware (C++)

The Arduino component, located in `src/main.cpp`, is responsible for the low-level control of the robot. It runs on an Arduino Uno and performs the following key functions:

*   **Motor Control:** Generates PWM signals to control the speed and direction of the robot's motors based on commands received.
*   **Dual Mode Operation:**
    *   **Autonomous Mode:** Receives speed and steering commands from the Raspberry Pi via serial communication (e.g., "1550,1600" for speed and steering PWM values).
    *   **Manual Mode:** Reads PWM signals from an RC (Radio Control) receiver connected to its input pins (specifically CH2 for throttle and CH4 for steering) to allow for manual remote operation. A physical switch connected to a designated pin (MODE_PIN) is used to toggle between autonomous and manual modes.
*   **Task Management:** Utilizes the `TaskScheduler` library to manage periodic tasks like updating motor outputs and printing status information.
*   **RC Input Handling:** Employs the `PinChangeInterrupt` library to efficiently capture PWM signals from the RC receiver.

The firmware is built and uploaded using PlatformIO, with dependencies specified in `platformio.ini`.

## Raspberry Pi Controller (Python)

The Python application (`app.py`) runs on a Raspberry Pi and serves as the brain of the robot. Its main responsibilities include:

*   **Computer Vision:** Captures video frames from a Picamera2 module, processes them using OpenCV (`cv2`) to detect the line to be followed. This involves:
    *   Converting images to grayscale.
    *   Applying Gaussian blur and binary thresholding to isolate the line.
    *   Using contour detection to find the line and calculate its center.
*   **Control Logic:** Calculates steering adjustments based on the detected line's position relative to the image center. It determines the appropriate speed and steering PWM values.
*   **Serial Communication:** Sends the calculated speed and steering PWM commands to the Arduino via a USB serial connection. It includes logic for automatically finding and reconnecting to the Arduino if the connection is lost.
*   **Web Interface:** Hosts a Flask web application that provides:
    *   A live video stream from the camera, overlaid with debugging information (e.g., detected line, ROI).
    *   Status information such as current speed, steering values, operational mode, and serial connection status.
*   **User Interaction:** Allows starting and stopping the autonomous driving mode via keyboard input in the terminal.

Key Python libraries used include:
*   `opencv-python` (cv2) for image processing.
*   `Flask` and `Flask-Sock` for the web interface and WebSocket communication.
*   `pyserial` for serial communication with the Arduino.
*   `picamera2` for interfacing with the Raspberry Pi camera module.
*   `numpy` for numerical operations, often used with OpenCV.

## Hardware Requirements

To build and operate this line-following robot, you will need the following hardware components:

*   **Raspberry Pi:** Any model with CSI camera interface and USB ports (e.g., Raspberry Pi 3B+, Raspberry Pi 4).
*   **Raspberry Pi Camera Module:** A compatible Picamera2 camera (e.g., Camera Module V1, V2, or HQ Camera).
*   **Arduino Uno:** Or a compatible microcontroller board (e.g., Arduino Nano).
*   **Robot Chassis:** A physical base for the robot, including:
    *   Motors (typically two DC motors for differential drive).
    *   Wheels.
    *   Caster wheel (optional, for stability).
*   **Motor Driver:** A module capable of driving the robot's motors, compatible with Arduino PWM signals (e.g., L298N, DRV8833).
*   **RC Receiver and Transmitter (Optional):** For manual control. The receiver should output PWM signals (at least 2 channels for throttle and steering).
*   **Mode Switch (Optional):** A physical switch to toggle between autonomous and manual modes, connected to the Arduino.
*   **Power Supply:**
    *   A power source for the Raspberry Pi (e.g., 5V USB power adapter).
    *   A separate power source for the motors and Arduino, appropriate for the chosen motors (e.g., LiPo battery, AA battery pack).
*   **USB Cable:** To connect the Raspberry Pi to the Arduino (for serial communication and programming the Arduino initially).
*   **Jumper Wires and Breadboard (Optional):** For making connections between components.
*   **SD Card:** For the Raspberry Pi's operating system.

## Software Setup and Installation

### 1. Arduino (Firmware)

The Arduino firmware is located in the `src/main.cpp` file and is managed using PlatformIO.

*   **Install PlatformIO IDE:** The recommended way is to use VS Code with the PlatformIO IDE extension. Follow the instructions on the [PlatformIO website](https://platformio.org/install).
*   **Clone the Repository:**
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```
*   **Open Project in PlatformIO:** Open the cloned repository folder in VS Code (or your PlatformIO environment). PlatformIO should automatically recognize it as a PlatformIO project.
*   **Build and Upload:**
    *   Connect your Arduino Uno (or compatible board) to your computer via USB.
    *   PlatformIO will typically auto-detect the board and port. If not, you might need to configure it in the `platformio.ini` file or via the PlatformIO interface.
    *   Build the project and upload the firmware to the Arduino using the PlatformIO controls (usually a "Upload" button or command).

### 2. Raspberry Pi (Controller Software)

The controller software runs on a Raspberry Pi with a compatible Linux distribution (e.g., Raspberry Pi OS).

*   **Enable Camera Interface:**
    *   Run `sudo raspi-config`.
    *   Navigate to `Interface Options` -> `Camera`.
    *   Enable the camera and reboot if prompted.
*   **Install Python 3:** Ensure Python 3 is installed. It usually comes pre-installed on Raspberry Pi OS.
*   **Clone the Repository (if not already done):**
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```
*   **Install Python Dependencies:**
    Open a terminal on your Raspberry Pi and install the necessary Python libraries. It's recommended to use a virtual environment:
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    pip install opencv-python flask flask-sock pyserial picamera2 numpy
    ```
    *(Note: Installing OpenCV might take some time and might require additional system dependencies. Refer to OpenCV or Picamera2 documentation for detailed installation instructions if you encounter issues.)*

## How to Run

After completing the hardware assembly and software installation:

1.  **Connect Devices:**
    *   Ensure the Arduino is connected to the Raspberry Pi via a USB cable. This connection is used for serial communication.
    *   If using manual mode, ensure your RC receiver is connected to the Arduino and your RC transmitter is powered on.
    *   Ensure the mode switch (if used) is connected to the Arduino.
2.  **Power On:**
    *   Power on the Raspberry Pi.
    *   Power on the Arduino and the robot's motors.
3.  **Run the Python Controller:**
    *   Open a terminal on the Raspberry Pi.
    *   Navigate to the project directory where `app.py` is located.
    *   If you used a virtual environment for Python dependencies, activate it:
        ```bash
        source .venv/bin/activate
        ```
    *   Run the Python script:
        ```bash
        python3 app.py
        ```
4.  **Access the Web Interface:**
    *   Open a web browser on a device connected to the same network as the Raspberry Pi.
    *   Navigate to `http://<raspberry_pi_ip>:5000`, replacing `<raspberry_pi_ip>` with the actual IP address of your Raspberry Pi. You should see the camera feed and status information.
5.  **Operating the Robot:**
    *   **Mode Selection:**
        *   The Arduino firmware (`src/main.cpp`) uses a `MODE_PIN` (pin 4 by default, configured with an internal pull-up).
        *   **Autonomous Mode:** Set the `MODE_PIN` to HIGH. The robot will be controlled by the Raspberry Pi.
        *   **Manual Mode:** Set the `MODE_PIN` to LOW. The robot will be controlled by the RC transmitter.
        *   The initial mode and behavior on serial timeout are defined in the Arduino sketch.
    *   **Starting/Stopping Autonomous Drive (via Terminal):**
        *   The `app.py` script listens for keyboard commands in the terminal where it's running:
            *   Press `s` to start autonomous line following.
            *   Press `q` to stop autonomous driving and exit the Python script.
            *   Press `r` to attempt a serial reconnection to the Arduino.
    *   **Monitoring:** Observe the robot's behavior and the web interface for feedback. The terminal running `app.py` will also print status messages.

**Important Notes:**

*   **Safety First:** Always operate the robot in a safe environment, especially during initial testing. Be prepared to stop it quickly if it behaves unexpectedly.
*   **Calibration:** You might need to adjust parameters in `app.py` (e.g., `steering_sensitivity`, `base_speed`, ROI for line detection) and potentially in `src/main.cpp` (e.g., PWM ranges) to suit your specific robot hardware and track conditions. The comments in Korean within the source code might provide additional context for these parameters.
*   **Serial Port:** `app.py` attempts to auto-detect the Arduino's serial port. If it fails, you might need to specify it manually in the script or ensure the Arduino is properly recognized by the Raspberry Pi (`dmesg` command can be helpful for troubleshooting).

## Project Structure

Here's an overview of the key files and directories in this project:

*   **`README.md`**: This file, providing an overview and instructions for the project.
*   **`platformio.ini`**: Configuration file for PlatformIO, specifying the Arduino board, framework, and library dependencies for the firmware.
*   **`src/main.cpp`**: The main C++ source code for the Arduino firmware. It handles motor control, RC input, serial communication, and mode switching.
*   **`app.py`**: The main Python script for the Raspberry Pi. It performs image processing for line detection, sends control commands to the Arduino, and hosts the Flask web interface.
*   **`templates/`**: This directory contains HTML templates used by the Flask web application.
    *   **`index.html`**: The main page for the web interface, displaying the camera feed and robot status. (Note: The content of `index.html` was not fully analyzed but is assumed to serve this purpose).
*   **`lib/`**: This directory is typically used by PlatformIO to store project-specific libraries. It contains `README` files, suggesting it might hold or have held custom library code.
*   **`include/`**: This directory is typically used by PlatformIO for header files. It contains a `README`, suggesting it might hold or have held custom header files.
*   **`.gitignore`**: Specifies intentionally untracked files that Git should ignore (e.g., build artifacts, virtual environment directories like `.venv/`).
*   **`main.cpp` (root directory)**: Appears to be an older or alternative version of the Arduino firmware. The primary firmware is in `src/main.cpp`.

## Contributing

Contributions to this project are welcome! If you have improvements, bug fixes, or new features you'd like to suggest, please feel free to:

1.  Fork the repository.
2.  Create a new branch for your changes.
3.  Make your changes and commit them with clear messages.
4.  Submit a pull request for review.