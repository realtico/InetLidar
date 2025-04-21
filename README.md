# Lidar M1C1-mini TCP Viewer

This project provides a complete system for reading and visualizing data from the **M1C1-mini 2D LiDAR** sensor. It consists of:

- A **C-based serial reader** that reads LiDAR frames via UART and shares them over **TCP sockets**.
- A **Python-based graphical client** that connects to the TCP server and renders the point cloud using **pygame**, supporting full-screen display and dynamic scaling.

---

## 🚀 Device Overview

- **Model:** M1C1-mini 2D LiDAR
- **Range:** Up to 8 meters
- **Data Rate:** ~5 Hz (adjustable)
- **Connection:** UART at 115200 baud
- **Datasheet:** [Download PDF](M1C1_Mini_Datasheet.pdf) *(Chinese original, translated to Brazilian Portuguese via Google Translate)*

The LiDAR used in this project was acquired via AliExpress:  
🔗 [AliExpress Product Link](https://www.aliexpress.com/snapshot/0.html?spm=a2g0o.9042647.6.2.559e37a1jYP0hF&orderId=8133980903635463&productId=4000251359842)

---

## ⚙️ Components

### 1. `lidar_reader_tcp_bin.c`

A C program that:
- Continuously reads LiDAR data from a serial port.
- Parses the proprietary frame format of the M1C1-mini.
- Shares the latest frame via a TCP socket (default port: **9999**).

Use this to run on low-power devices like **Raspberry Pi Zero** or **ESP32 with Linux**.

### 2. `Lidar_Client_TCP_Bin.py`

A Python script that:
- Connects to the LiDAR server via TCP.
- Parses the binary point cloud data.
- Displays a **fullscreen or windowed polar visualization** using pygame.
- Shows overlay with delay and FPS.
- Includes hotkeys for scale adjustment, dot size, grid toggle, and theme switch.

---

## 🚀 Getting Started

### Prerequisites

- `gcc` (for compiling the C server)
- `Python 3.7+` with `pygame`

### Compilation (Server)

```bash
gcc -o lidar_reader_tcp lidar_reader_tcp_bin.c -lpthread -lm
```

### Running

#### Start the Server

```bash
./lidar_reader_tcp /dev/ttyUSB0
```

#### Run the Viewer

```bash
python3 Lidar_Client_TCP_Bin.py -f
```

Or in windowed mode with 800x800 resolution:

```bash
python3 Lidar_Client_TCP_Bin.py -w 800
```

---

## 🧠 Notes

> This is a personal development project based on the M1C1-mini datasheet.  
> The English documentation provided by sellers was incomplete, so the Chinese PDF was translated into Brazilian Portuguese using Google Translate.

---

## 📄 License

This project is distributed under an open-source license. Feel free to fork and adapt it for your own robotics or sensor processing applications.

---

## 💠 Author

Developed by Realtico – passionate about robotics, embedded systems, and real-time sensor interfaces.
... and several instances of ChatGPT, which helped me with the socket connections and debugging and optimizing code!

