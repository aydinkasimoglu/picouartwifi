
# Pico W UART → UDP bridge (framed protocol)

This firmware for **Raspberry Pi Pico W** receives framed binary messages on **UART0**, validates them with **CRC‑16/CCITT**, and forwards supported payloads to a configured **UDP server** over Wi‑Fi (lwIP + CYW43).

It currently forwards **sensor frames** (message ID `0x01`) only.

## Features

- **UART frame parser (IRQ-driven)**: Consumes a byte stream and reconstructs frames.
- **CRC validation**: Drops frames with invalid CRC.
- **Sensor forwarder**: Only forwards `MSG_ID_SENSOR (0x01)` with the expected payload length.
- **UDP sender**: Sends a packed struct containing message type, sequence number, and sensor payload.
- **LED activity**: Brief LED pulse on each outbound UDP packet.

## Hardware

- **Board**: Raspberry Pi Pico W (RP2040 + CYW43)
- **UART level**: 3.3V TTL (do not feed 5V UART directly)

### Wiring (default pins)

The firmware uses **UART0** with these GPIO assignments (see `UART_TX_PIN` / `UART_RX_PIN` in `main.c`):

- Pico **GPIO0** = UART0 **TX**
- Pico **GPIO1** = UART0 **RX**

Connect your external serial device:

- External **TX** → Pico **GPIO1** (RX)
- External **RX** ← Pico **GPIO0** (TX)
- **GND ↔ GND**

## UART protocol

The UART byte stream is parsed into frames of the form:

```
{ 0xBC, 0x35, LEN, ID, PAYLOAD[LEN], CRC16_LE }
```

- `0xBC` and `0x35` are the two sync/header bytes.
- `LEN` is the payload length in bytes (0..32; frames larger than 32 are dropped).
- `ID` is the message type.
- `CRC16_LE` is the 16-bit CRC sent **little-endian** (low byte first).

### CRC details

CRC is computed over:

```
{ LEN, ID, PAYLOAD[0..LEN-1] }
```

Algorithm is CRC‑16/CCITT with polynomial `0x1021`, initial CRC `0x0000`.

## Configuration

Edit the constants near the top of `main.c`:

- `WIFI_SSID`, `WIFI_PASSWORD`, `WIFI_CONNECT_TIMEOUT_MS`
- `UDP_SERVER_IP`, `UDP_PORT`
- `UART_ID`, `BAUD_RATE`, `UART_TX_PIN`, `UART_RX_PIN` (and format bits)
- `HEADER_1`, `HEADER_2`, `MSG_ID_SENSOR`, `UART_MAX_PAYLOAD_LEN`

Security note: Wi‑Fi credentials are currently compiled into the firmware via `main.c`. Consider keeping local credentials out of version control.
