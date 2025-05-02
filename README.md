# 🚚 Autonomous Delivery Wagon (ADW) [In Progress]

A sidewalk-friendly autonomous delivery robot designed for scalable logistics.

<div align="center">
  <img src="https://github.com/user-attachments/assets/429021f5-e294-4863-8b56-89cd9b16ff1b" height="200" width: auto alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
  <img src="https://github.com/user-attachments/assets/a125ad9d-e70a-41b4-950b-f0db18e28c48" height="200" width: auto alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
  <img src="https://github.com/user-attachments/assets/4a7f62c9-dc78-49ad-9218-2073750102c7" height="200" width: auto alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
  <p><a href="#journal-log"> Scroll down for journal logs 📓😊!</a></p>
</div>

## 🛠️ Project Overview

The **Autonomous Delivery Wagon (ADW)** is a heavy-duty autonomous robot platform, based on a **modified garden wagon chassis**, and tailored for real-world delivery applications. Designed with affordability, modularity, and functionality in mind, ADW is engineered to carry up to **600 lbs** while navigating sidewalks and varied terrains autonomously.

tldr; designing an autonomous bot for local food delivery (ie. doordash, ubereats) by utilizing sidewalks

---

## ⚙️ Key Features

- **Modified Commercial Wagon Base**  
  - Reinforced steel-frame platform  
  - Custom **axle replacement** for improved durability  
  - Integrated **350W high-torque motor** and servo system for controlled mobility

- **Advanced Perception Stack**  
  - Mounted **3D LiDAR** with **360° horizontal** and **90° vertical FOV** for robust environmental awareness  
  - **Mono camera** for object detection and tracking  
  - Real-time obstacle detection and path planning

- **Compute & Autonomy**  
  - Powered by **NVIDIA Jetson Nano**  
  - Running **SLAM (Simultaneous Localization and Mapping)** and **ROS (Robot Operating System)** for localization and autonomous decision-making

- **Planned Capabilities**  
  - Manual control phase (for testing and calibration)  
  - Data collection for SLAM and AI model tuning  
  - Autonomous path planning and dynamic rerouting  
  - Secure delivery logic with compartment locking

---

## 📅 Development Roadmap

> **Current Phase:** Mechanical Build & Manual Control Setup

- ✅ Modify base wagon and install axle/motor system  
- ✅🔄 Integrate motor and servo components  
- ✅ Mount 3D LiDAR and mono camera  
- ✅ Building housing and electronics mount  
- ✅ Manual control setup  
- 🧪 Upcoming: SLAM data collection and initial autonomy tests

---

## 📚 Tech Stack

**Hardware:**  
`Modified Wagon Frame` • `Custom Axle` • `350W Motor` • `3D LiDAR (360°x90°)` • `Mono Camera` • `Servo Steering`

**Software & Compute:**  
`NVIDIA Jetson Nano` • `ROS` • `SLAM` • `Python` • `C++`

---

## 💡 Project Goal

To create a low-cost, sidewalk-compatible autonomous delivery robot platform inspired by real-world AVs. Aiming to utilize sidewalk infastracture for logistics (ie doordash, ubereats).

---

<a name="journal-log"></a>
## Journal Log

<div align="center">

### 🛞 status as of 05/02/25 [servo twitching... 🫠]

<img src="https://github.com/user-attachments/assets/963cd42a-231f-4b8f-b305-01be0b807ab1" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
<img src="https://github.com/user-attachments/assets/74b73835-13c5-408c-b898-a3c49e2c9064" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">

</div>

Steering mech solved!! but currently experiencing **servo twitching**, both with and without mechanical load.  
I'm in the process of debugging whether this is due to:

- A faulty servo, or  
- An unstable PWM signal from the Jetson.

As part of my wiring setup:

- The servo's built-in 22 AWG wires were extended.
  - **Signal wire** extended using the same 22 AWG.
  - **VCC and GND wires** extended with 14 AWG (since I ran out of 22 AWG and wanted to safely support 12V current).
- A **5A inline fuse** was added to the VCC line to prevent overcurrent, especially in stall conditions.

**Hypothesis:**  

The twitching might be caused by electrical noise from the AWG mismatch (22→14), particularly at the soldered joints.  
Next step: testing with consistent wire gauges and checking the PWM signal stability.

<div align="center">

### 🛞 status as of 04/30/25 [steering mechanism void 🥲]

<img src="https://github.com/user-attachments/assets/de53d2b0-0ec8-4076-bccc-81c74bc827a3" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
<img src="https://github.com/user-attachments/assets/e0bf1dd3-6656-43e6-9261-c58087abfb40" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
<img src="https://github.com/user-attachments/assets/da425b6c-fe27-4f57-b305-9cecef449351" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">

</div>

**Steering Mechanism Development Log**

Currently facing challenges in designing and printing parts for a stable and precise steering mechanism.

<div align="center">
  
<img src="https://upload.wikimedia.org/wikipedia/commons/8/89/Ackerman_Steering_Linkage.gif" height="200" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">

</div>

**Approaches Tried:**

- **Direct rotational steering**: Mounted the servo directly on the pivoting axle assembly.  
  - *Issue:* The servo moved with the steering assembly, causing instability and limiting the range of motion.

- **Lever-actuated steering using PVC arms**: Connected the servo to the wagon’s front axle with makeshift linkages.  
  - *Issue:* This setup lacked precision and mechanical robustness.

- **Current approach – Drag link steering**:
<div align="center">
  <img src="https://i.makeagif.com/media/11-25-2015/7iNiAo.gif" height="200" alt="Axle Installed" style="border-radius: 8px; margin: 8px;"><br>
  <em>Credits: Mekanizmalar</em>
</div>
  Mounting the servo to the fixed chassis and connecting it to the steering arm via a push-pull linkage (drag link).  
  - *Goal:* Achieve better mechanical control, increase range of motion, and keep the servo stationary for consistent torque application.

<div align="center">

### 🛞 status as of 04/25/25

<img src="https://github.com/user-attachments/assets/a125ad9d-e70a-41b4-950b-f0db18e28c48" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">

</div>

<div align="center">

### 🛞 status as of 04/17/25

<img src="https://github.com/user-attachments/assets/3cb33db3-39e0-4885-9f0d-bc38f3951564" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
<img src="https://github.com/user-attachments/assets/da425b6c-fe27-4f57-b305-9cecef449351" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">

</div>

<div align="center">

### 🛞 status as of 04/15/25

<img src="https://github.com/user-attachments/assets/fd757812-911e-4f8b-9586-a58fbf3eb619" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
<img src="https://github.com/user-attachments/assets/3ded7747-cda9-48ec-ae2c-38e01035e94e" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
<img src="https://github.com/user-attachments/assets/7e5cdcd7-3e42-47d1-afb0-779342addc7d" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
<img src="https://github.com/user-attachments/assets/56103127-4d8a-4e87-9d79-0a71c5460200" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">

</div>

<div align="center">

### 🛞 status as of 04/03/25

<img src="https://github.com/user-attachments/assets/5736a210-2ebe-4cfc-a97d-3ada6ad23fc3" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">
<img src="https://github.com/user-attachments/assets/920bb215-e0b5-4365-a556-7b41b9e3cb0b" height="300" alt="Axle Installed" style="border-radius: 8px; margin: 8px;">

</div>


---

## 🤝 Collaboration & Feedback

This is an ongoing solo project — open to collaboration, ideas, or advice.  
Feel free to open an issue or contact directly with feedback.

---

## 📜 License

*To be determined.*

---

## 🔔 Stay Tuned

More updates coming soon — follow commits and README changes as the project evolves!
