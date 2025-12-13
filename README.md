
**Project**
- **Description**: Simple Raspberry Pi Pico W program that forwards lines received on a UART to a remote UDP server. The firmware reads serial data into a ring buffer and sends completed lines (terminated by CR/LF) over Wi‑Fi using lwIP/CYW43.

**Features**
- **UART → UDP**: Forwards newline-terminated serial lines to a configured UDP destination.
- **Ring buffer**: Simple interrupt-driven UART reception with a circular buffer to avoid data loss.
- **LED activity**: Pico W on-board LED toggles briefly on each outbound UDP packet.

**Hardware**
- **Board**: Raspberry Pi Pico W (RP2040 + CYW43)
- **Wiring**: Connect external serial device TX → Pico `GPIO0` (UART RX), external serial device RX ← Pico `GPIO1` (UART TX). Use a common ground.

**Configuration**
- **Source**: Edit `main.c` to change network and serial settings. Key constants at top of `main.c`:
	- `WIFI_SSID`, `WIFI_PASSWORD` — Wi‑Fi credentials
	- `UDP_SERVER_IP`, `UDP_PORT` — destination for forwarded messages
	- `UART_TX_PIN`, `UART_RX_PIN`, `BAUD_RATE` — UART settings