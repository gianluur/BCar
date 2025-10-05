# 🚗 Bluetooth-Controlled Arduino Car

This project demonstrates a **Bluetooth-controlled robotic car** built using **Arduino**, **HC-05 Bluetooth module**, and **DC motors** controlled through an H-Bridge driver (e.g., L298N or L293D).  
The system consists of two Arduinos:

- **Master Arduino (Controller)** — Reads joystick input and sends commands via Bluetooth.
- **Slave Arduino (Car)** — Receives commands and drives the motors accordingly.

---

## 🧠 Overview

The setup allows you to control the car’s motion — **Forward, Backward, Left, Right, and Stop** — wirelessly using a joystick connected to the master Arduino. The master communicates commands (`F`, `B`, `L`, `R`, `S`) to the slave Arduino via Bluetooth.

---

## 🖼️ Photos

![](./media/IMG_7180.png)
![](./media/IMG_7181.png)
![](./media/IMG_7184.png)

---

## ⚙️ Features

- Wireless Bluetooth communication (HC-05/HC-06)
- Real-time joystick control
- Object-oriented DC motor control
- Adjustable speed control
- Modular design (separate master and slave code)
- Debug-friendly Serial monitoring

---

## 🧩 Hardware Requirements

| Component                  | Quantity | Description                            |
| -------------------------- | -------- | -------------------------------------- |
| Arduino UNO or Nano        | 2        | One for Master, one for Slave          |
| HC-05 Bluetooth Module     | 2        | One configured as Master, one as Slave |
| Dual H-Bridge Motor Driver | 1        | L298N or L293D                         |
| DC Motors                  | 2        | Left and Right drive motors            |
| Joystick Module            | 1        | With X, Y axis and push button         |
| Battery Pack               | 1        | To power the motors                    |
| Jumper Wires               | —        | For connections                        |

---

## 🔌 Circuit Connections

### **Slave Arduino (Car)**

| Pin     | Connected To            | Description                |
| ------- | ----------------------- | -------------------------- |
| 9       | ENA (Motor A Enable)    | PWM speed control          |
| 8       | IN1                     | Motor A direction          |
| 7       | IN2                     | Motor A direction          |
| 3       | ENB (Motor B Enable)    | PWM speed control          |
| 5       | IN3                     | Motor B direction          |
| 4       | IN4                     | Motor B direction          |
| 0 (RX)  | HC-05 TX                | Bluetooth communication    |
| 1 (TX)  | HC-05 RX                | Bluetooth communication    |
| GND, 5V | Common ground and power | Shared between all modules |

---

### **Master Arduino (Joystick Controller)**

| Pin     | Connected To            | Description                |
| ------- | ----------------------- | -------------------------- |
| A0      | Joystick X-axis         | Left/Right control         |
| A1      | Joystick Y-axis         | Forward/Backward control   |
| 2       | Joystick Button         | Stop command               |
| 0 (RX)  | HC-05 TX                | Bluetooth communication    |
| 1 (TX)  | HC-05 RX                | Bluetooth communication    |
| GND, 5V | Common ground and power | Shared between all modules |

> ⚠️ **Important:**  
> Do **not** connect both Arduinos’ USB cables to the same PC while the HC-05 modules are connected to pins 0 and 1. It can cause serial interference. Disconnect the Bluetooth RX/TX pins while uploading sketches.

---

## 🧠 Software Architecture

### **1. Slave Arduino**

- Contains two main classes:
  - `DCMotor`: Encapsulates individual motor control (speed and direction).
  - `Motors`: Controls both left and right motors together.
- Listens for incoming Bluetooth commands:
  - `'F'` → Move Forward
  - `'B'` → Move Backward
  - `'L'` → Turn Left
  - `'R'` → Turn Right
  - `'S'` → Stop

### **2. Master Arduino**

- Continuously reads joystick position (X/Y analog values).
- Translates movement direction into commands (`F`, `B`, `L`, `R`, `S`).
- Sends those commands via Bluetooth to the slave Arduino.

---

## 🧰 Installation & Setup

1. Open the project in the **Arduino IDE**.
2. Upload the following sketches:
   - `Joystick.ino` → to the car Arduino.
   - `BCar.ino` → to the controller Arduino.
3. Configure the **HC-05 modules**:
   - **Master:** set to _Master mode_.
   - **Slave:** set to _Slave mode_.
   - Pair them once via **AT commands**.

---

## 🎮 Usage

1. Power both Arduinos and ensure Bluetooth pairing is complete.
2. Move the joystick:
   - **Up** → Move forward
   - **Down** → Move backward
   - **Left** → Turn left
   - **Right** → Turn right
   - **Press joystick button** → Stop
3. Open **Serial Monitor (9600 baud)** on both boards to see real-time debug messages.

---

## 🪫 Troubleshooting

| Problem                 | Possible Cause                   | Solution                                        |
| ----------------------- | -------------------------------- | ----------------------------------------------- |
| Motors not moving       | Wrong wiring / power issue       | Check motor driver wiring and power connections |
| No Bluetooth connection | Incorrect HC-05 pairing          | Re-enter AT mode and configure roles properly   |
| Commands not received   | RX/TX conflict                   | Disconnect Bluetooth before uploading code      |
| Jerky or uneven motion  | Power fluctuation or low battery | Use separate power supply for motors            |

---

## 🧑‍💻 Author

**Gianluca Russo**  
🔗 [GitHub](https://github.com/gianluur) | ✉️ gianluca.rssu@protonmail.com

---

## 📝 License

This project is licensed under the **MIT License** — feel free to use and modify it.

---
