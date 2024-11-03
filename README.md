# SERIAL PORT PROTOCOL PROJECT
This project involves developing a serial communication protocol in C. The goal is to implement and optimize communication via a serial port, ensuring transmission reliability and minimizing interference from old data.

## Project Structure

- `bin/`: Contains the compiled binaries.
- `src/`: Source code for implementing the communication protocol. This is where modifications should be made for the project.
- `include/`: Header files for the link-layer and application layer protocols. **These files should not be changed**.
- `cable/`: Virtual cable program to help test the serial port, especially with disconnections and noise.
- `main.c`: Main project file. **This file should not be changed**.
- `Makefile`: Makefile for building the project and running the application.
- `penguin.gif`: Sample file to be sent through the serial port.

## Features and Requirements

1. **Initial Connection (`llopen`)**
   - Establishes a connection on the serial port.
   - Defines the device role as either transmitter or receiver.
   - Includes retransmission and timeout handling, preventing issues when reconnecting quickly after disconnections.

2. **Packet Transmission (`llwrite`)**
   - Sends data via the serial protocol, ensuring the packet includes the header with fields like `C`, `S`, `l2`, and `l1`.
   - Integrates mechanisms to prevent interference from old data in retransmissions, ensuring packet integrity.

3. **Packet Reception (`llread`)**
   - Receives data, handling parameters like `dataSize` and `bufferSize`.
   - Includes logic to reject incorrect data and clear the buffer when needed.

4. **Interference and Reconnection Management**
   - Implements triggers to handle disconnections and noise in communication.
   - Prevents the program from "freezing" upon quick disconnections and reconnections.

## Instructions to Run the Project

1. **Editing and Compiling**
   - Edit the code in the `src/` directory according to the protocol requirements.
   - Compile the application and virtual cable program using the `Makefile`:
     ```bash
     $ make
     ```

2. **Running the Virtual Cable**
   - Run the virtual cable program to simulate the serial port connection:
     ```bash
     $ sudo ./bin/cable_app
     ```
   - Alternatively, use the Makefile target:
     ```bash
     $ sudo make run_cable
     ```

3. **Testing without Disconnections or Noise**
   - **Receiver**:
     ```bash
     $ ./bin/main /dev/ttyS11 rx penguin-received.gif
     ```
     Or use the Makefile:
     ```bash
     $ make run_rx
     ```

   - **Transmitter**:
     ```bash
     $ ./bin/main /dev/ttyS10 tx penguin.gif
     ```
     Or use the Makefile:
     ```bash
     $ make run_tx
     ```

   - **File Comparison**:
     Verify that the received file matches the sent file:
     ```bash
     $ diff -s penguin.gif penguin-received.gif
     ```
     Or use the Makefile:
     ```bash
     $ make check_files
     ```

4. **Testing with Disconnections and Noise**
   - **Steps**:
     1. Run the receiver and transmitter again.
     2. Switch to the cable console and use the following options:
        - `0`: Disconnect the cable.
        - `2`: Add noise.
        - `1`: Return to normal state.
     3. Check again if the received file matches the sent file, even with disconnections and noise.

## Important Notes

- **Error Handling and Reconnection**:
   The program should handle quick disconnections and reconnections properly to avoid "freezes" or data loss.

- **Data Integrity Maintenance**:
   Ensure that received packets are complete and free of unwanted noise, maintaining communication reliability.

This README serves as a guide to implementing and testing the serial communication protocol, focusing on transmission robustness and data integrity.

---

