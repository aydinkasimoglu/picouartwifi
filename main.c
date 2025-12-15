/**
 * @file main.c
 * @author Aydın Kasımoğlu
 * @brief Pico W UART-to-UDP bridge.
 *
 * Reads framed messages from UART and forwards selected payloads as UDP packets.
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/uart.h"

/* ------------ CONFIGURATION --------------- */
#define WIFI_SSID     "---------------" /* Wi-Fi SSID to connect to. */
#define WIFI_PASSWORD "---------------" /* Wi-Fi password to connect to. */
#define WIFI_CONNECT_TIMEOUT_MS 10000U  /* Maximum time to wait for Wi-Fi connection attempt (milliseconds). */

#define UDP_SERVER_IP "---------------" /* Destination UDP server. */
#define UDP_PORT      4444U             /* Destination UDP port. */

#define UART_ID     uart0            /* UART peripheral to use. */
#define UART_IRQ    UART0_IRQ        /* IRQ line for the selected UART. */
#define BAUD_RATE   115200U          /* UART baud rate. */
#define DATA_BITS   8U               /* UART data bits. */
#define STOP_BITS   1U               /* UART stop bits. */
#define PARITY      UART_PARITY_NONE /* UART parity selection. */
#define UART_TX_PIN 0U               /* GPIO pin used for UART TX. */
#define UART_RX_PIN 1U               /* GPIO pin used for UART RX. */
#define UART_MAX_PAYLOAD_LEN 32U     /* Maximum accepted UART payload length (bytes). */

#define HEADER_1      0xBC /* First sync/header byte in the UART frame. */
#define HEADER_2      0x35 /* Second sync/header byte in the UART frame. */
#define MSG_ID_SENSOR 0x01 /* Message ID for sensor payload frames. */
/* ------------------------------------------ */

/* ------------- STRUCTS -------------------- */
/**
 * @brief UART receive parser states.
 *
 * Parser consumes a byte stream and reconstructs frames:
 * { HEADER_1, HEADER_2, LEN, ID, PAYLOAD[LEN], CRC16_LE }.
 */
typedef enum UART_State
{
    STATE_WAIT_SYNC_1,
    STATE_WAIT_SYNC_2,
    STATE_WAIT_LEN,
    STATE_WAIT_ID,
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CRC_L,
    STATE_WAIT_CRC_H
} UART_State_t;

/**
 * @brief Sensor payload transported inside UART frames.
 */
#pragma pack(push, 1)
typedef struct Sensor_Payload
{
    uint32_t pressure_pa;
    uint16_t distance_mm;
} Sensor_Payload_t;
#pragma pack(pop)

#pragma pack(push, 1)
/**
 * @brief UDP packet format sent to the server.
 */
typedef struct UDP_Packet
{
    uint8_t          msg_type;
    uint32_t         sequence_num;
    Sensor_Payload_t sensor_data;
} UDP_Packet_t;
#pragma pack(pop)
/* ------------------------------------------ */

/* ------------- GLOBALS -------------------- */
static UART_State_t rx_state = STATE_WAIT_SYNC_1;
static uint8_t      rx_buf[UART_MAX_PAYLOAD_LEN];
static uint8_t      rx_idx = 0;
static uint8_t      rx_len = 0;
static uint8_t      rx_id  = 0;
static uint8_t      rx_crc_l = 0;

static struct udp_pcb *udp_socket;
static ip_addr_t server_ip;
static uint32_t seq_counter = 0;

static volatile bool    sensor_payload_ready = false; /* Latest valid sensor payload. */
static volatile uint8_t sensor_payload_buf[sizeof(Sensor_Payload_t)];
/* ------------------------------------------ */

/* ------------ FUNCTIONS ------------------- */
static int  setup_wifi(void);
static void setup_uart(void);
static void uart_rx_isr(void);
static void send_udp_packet(uint8_t id, const uint8_t *raw_data);
static uint16_t calc_crc16_frame(uint8_t len, uint8_t id, const uint8_t *payload);
static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte);
/* ------------------------------------------ */

int main(void)
{
    stdio_init_all();

    if (setup_wifi()) {
        return 1;
    }

    setup_uart();

    while (1)
    {
        if (sensor_payload_ready)
        {
            uint8_t local_payload[sizeof(Sensor_Payload_t)];
            bool have_payload = false;

            const uint32_t irq_state = save_and_disable_interrupts();
            if (sensor_payload_ready)
            {
                for (size_t i = 0; i < sizeof(local_payload); ++i)
                {
                    local_payload[i] = sensor_payload_buf[i];
                }
                sensor_payload_ready = false;
                have_payload = true;
            }
            restore_interrupts(irq_state);

            if (have_payload)
            {
                send_udp_packet(MSG_ID_SENSOR, local_payload);
            }
        }
    }

    return 0;
}

/**
 * @brief Initializes and configures the Wi-Fi connection
 * 
 * Sets up the Wi-Fi interface with necessary configuration parameters
 * and establishes a connection to the configured network.
 * 
 * @retval int Returns 0 on successful Wi-Fi setup, 1 on failure
 */
static int setup_wifi(void)
{
    if (cyw43_arch_init())
    {
        return 1;
    }

    /* Indicate we're starting connection attempt (turn LED off) */
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    /* Enable Station mode */
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, WIFI_CONNECT_TIMEOUT_MS))
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        return 1;
    }

    /* Wait for DHCP */
    while (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP)
    {
        sleep_ms(100);
    }

    cyw43_arch_lwip_begin();

    /* Create UDP PCB (Protocol Control Buffer) */
    udp_socket = udp_new();
    if (!udp_socket)
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        cyw43_arch_lwip_end();
        return 1;
    }

    /* Prepare destination IP */
    if (!ipaddr_aton(UDP_SERVER_IP, &server_ip))
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        cyw43_arch_lwip_end();
        return 1;
    }

    cyw43_arch_lwip_end();

    /* Successful connection: turn LED on */
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    return 0;
}

/**
 * @brief Initializes and configures the UART peripheral
 * 
 * Sets up the UART interface with appropriate pins, baud rate, and other
 * configuration parameters required for serial communication.
 * 
 * @retval void
 */
static void setup_uart(void)
{
    /* Set up UART with the required speed */
    uart_init(UART_ID, BAUD_RATE);

    /* Set the TX and RX pins by using the function select on the GPIO */
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, true);

    /* Hook UART RX IRQ and enable RX interrupts. */
    irq_set_exclusive_handler(UART_IRQ, uart_rx_isr);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
}

/**
 * @brief UART receive interrupt service routine handler.
 * 
 * This function is called when data is received on the UART interface.
 * It handles the incoming data and processes it according to the application logic.
 *
 * @retval void
 */
static void uart_rx_isr(void)
{
    while (uart_is_readable(UART_ID))
    {
        const uint8_t byte = uart_getc(UART_ID);

        switch (rx_state)
        {
            case STATE_WAIT_SYNC_1:
            {
                if (byte == HEADER_1)
                {
                    rx_state = STATE_WAIT_SYNC_2;
                }
                break;
            }
            case STATE_WAIT_SYNC_2:
            {
                if (byte == HEADER_2)
                {
                    rx_state = STATE_WAIT_LEN;
                }
                else
                {
                    rx_state = STATE_WAIT_SYNC_1;
                }
                break;
            }
            case STATE_WAIT_LEN:
            {
                rx_len = byte;
                if (rx_len > (uint8_t)UART_MAX_PAYLOAD_LEN)
                {
                    rx_state = STATE_WAIT_SYNC_1;
                }
                else
                {
                    rx_state = STATE_WAIT_ID;
                }
                break;
            }
            case STATE_WAIT_ID:
            {
                rx_id = byte;
                rx_idx = 0;

                if (rx_len == 0)
                {
                    rx_state = STATE_WAIT_CRC_L;
                }
                else
                {
                    rx_state = STATE_WAIT_PAYLOAD;
                }
                break;
            }
            case STATE_WAIT_PAYLOAD:
            {
                if (rx_idx < (uint8_t)UART_MAX_PAYLOAD_LEN)
                {
                    rx_buf[rx_idx++] = byte;
                }

                if (rx_idx >= rx_len)
                {
                    rx_state = STATE_WAIT_CRC_L;
                }
                break;
            }
            case STATE_WAIT_CRC_L:
            {
                rx_crc_l = byte;
                rx_state = STATE_WAIT_CRC_H;
                break;
            }
            case STATE_WAIT_CRC_H:
            {
                const uint16_t recv_crc = (uint16_t)(((uint16_t)byte << 8) | (uint16_t)rx_crc_l);

                const uint16_t calc_crc = calc_crc16_frame(rx_len, rx_id, rx_buf);

                const uint8_t expected_payload_len = (uint8_t)sizeof(Sensor_Payload_t);

                /* Only forward expected sensor payload sizes. */
                if ((calc_crc == recv_crc) && (rx_id == MSG_ID_SENSOR) && (rx_len == expected_payload_len))
                {
                    for (size_t i = 0; i < sizeof(Sensor_Payload_t); ++i)
                    {
                        sensor_payload_buf[i] = rx_buf[i];
                    }
                    sensor_payload_ready = true;
                }

                rx_state = STATE_WAIT_SYNC_1;
                break;
            }

            default:
            {
                rx_state = STATE_WAIT_SYNC_1;
                break;
            }
        }
    }
}

/**
 * @brief Send one UDP packet containing sensor data.
 *
 * @param id Message ID/type
 * @param raw_data Pointer to raw sensor payload bytes.
 *
 * @retval void
 */
static void send_udp_packet(const uint8_t id, const uint8_t *raw_data)
{
    UDP_Packet_t packet;

    /* Prepare data */
    packet.msg_type = id;
    packet.sequence_num = seq_counter++;

    memcpy(&packet.sensor_data, raw_data, sizeof(Sensor_Payload_t));

    if (!udp_socket)
    {
        return;
    }

    /* Keep lwIP core locked only for lwIP operations. */
    cyw43_arch_lwip_begin();

    /* PBUF_TRANSPORT implies we are creating a packet for the transport layer.
       PBUF_RAM means we want it allocated in RAM. */
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(UDP_Packet_t), PBUF_RAM);
    if (p)
    {
        pbuf_take(p, &packet, sizeof(UDP_Packet_t));
        udp_sendto(udp_socket, p, &server_ip, UDP_PORT);
        pbuf_free(p);
    }

    cyw43_arch_lwip_end();

    /* Blink LED briefly to indicate outbound packet (Pico W CYW43 LED). */
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(30);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
}

/**
 * @brief Calculates the CRC16 checksum for a frame.
 * 
 * @param len The length of the payload in bytes.
 * @param id The frame identifier.
 * @param payload Pointer to the payload data buffer.
 * 
 * @retval uint16_t The calculated CRC16 checksum value.
 */
static uint16_t calc_crc16_frame(const uint8_t len, const uint8_t id, const uint8_t *payload)
{
    uint16_t crc = 0;
    crc = crc16_ccitt_update(crc, len);
    crc = crc16_ccitt_update(crc, id);

    for (uint8_t i = 0; i < len; ++i)
    {
        crc = crc16_ccitt_update(crc, payload[i]);
    }

    return crc;
}

/**
 * @brief Updates a CRC-16-CCITT checksum with a new byte.
 *
 * Performs an incremental CRC-16-CCITT calculation by processing one 
 * byte at a time. This function is typically called iteratively
 * over a data stream to compute the full checksum.
 *
 * @param crc The current CRC value (initially 0x0000 for a new calculation).
 * @param byte The next byte to process in the CRC calculation.
 *
 * @retval uint16_t The updated CRC-16-CCITT value.
 */
static uint16_t crc16_ccitt_update(uint16_t crc, const uint8_t byte)
{
    crc ^= (uint16_t)((uint16_t)byte << 8);

    for (uint8_t j = 0; j < 8U; ++j)
    {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }

    return crc;
}