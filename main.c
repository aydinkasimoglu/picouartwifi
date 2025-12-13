#include <string.h>

#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

/* ----------MACRO DEFINITIONS--------------- */
#define DEBUG 0

#if DEBUG
    #define DBG(...) do { (void) printf(__VA_ARGS__); (void) printf("\n"); } while(0)
#else
    #define DBG(...) ((void)0)
#endif
/* ------------------------------------------ */

/* -------------CONFIGURATION---------------- */
#define WIFI_SSID     "---------------"
#define WIFI_PASSWORD "---------------"
#define WIFI_CONNECT_TIMEOUT_MS 10000U

#define UDP_SERVER_IP "---------------"
#define UDP_PORT      4444U

#define UART_ID     uart0
#define BAUD_RATE   115200U
#define DATA_BITS   8U
#define STOP_BITS   1U
#define PARITY      UART_PARITY_NONE
#define UART_TX_PIN 0U
#define UART_RX_PIN 1U

#define MAX_MSG_SIZE 1024U

#define RING_BUFFER_SIZE 2048U
#define RING_BUFFER_MASK (RING_BUFFER_SIZE - 1U)
/* ------------------------------------------ */

/* --------------GLOBALS--------------------- */
static uint8_t ring_buf[RING_BUFFER_SIZE];
static volatile uint16_t head = 0U; /* Write index */
static volatile uint16_t tail = 0U; /* Read index */

struct udp_pcb *udp_socket;
ip_addr_t server_ip;
/* ------------------------------------------ */

/* --------------FUNCTIONS------------------- */
static void send_udp_packet(uint8_t *data, uint16_t len);
static void on_uart_rx(void);
static void setup_uart(void);
static int  setup_wifi(void);
/* ------------------------------------------ */

int main(void)
{
    stdio_init_all();

    if (setup_wifi()) {
        DBG("Couldn't set Wi-Fi connection.");
        return 1;
    }

    DBG("Wi-Fi is ready.");

    setup_uart();

    DBG("UART is ready.");

    static uint8_t msg_buf[MAX_MSG_SIZE];
    uint16_t msg_idx = 0;

    while (1)
    {
        while (tail != head)
        {
            /* Read one byte */
            const uint8_t ch = ring_buf[tail];

            tail = (tail + 1) & RING_BUFFER_MASK;

            /* Look for new line */
            if (ch == '\n' || ch == '\r')
            {
                /* Ignore empty lines */
                if (msg_idx > 0)
                {
                    msg_buf[msg_idx] = 0;

                    send_udp_packet(msg_buf, msg_idx);

                    /* Reset */
                    msg_idx = 0;
                }
            }
            else
            {
                /* Accumulate data */
                if (msg_idx < MAX_MSG_SIZE - 1)
                {
                    msg_buf[msg_idx++] = ch;
                }
                else
                {
                    /* Message too large */
                    /* Reset buffer to prevent memory corruption */
                    msg_idx = 0;
                }
            }
        }

        sleep_us(50);
    }

    return 0;
}

/**
 * @brief Sends a UDP packet with the provided data.
 * 
 * @param data Pointer to the buffer containing the data to be sent.
 * @param len Length of the data in bytes.
 * 
 * @retval void
 */
static void send_udp_packet(uint8_t *data, const uint16_t len)
{
    if (len == 0U || len > MAX_MSG_SIZE)
    {
        DBG("Invalid length of message.");
        return;
    }

    /* PBUF_TRANSPORT implies we are creating a packet for the transport layer */
    /* PBUF_RAM means we want it allocated in RAM */
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

    if (!p)
    {
        DBG("Couldn't allocate for pbuf.");
        return;
    }

    /* Copy data into the pbuf */
    memcpy(p->payload, data, len);

    /* Send the packet */
    /* lock lwIP core */
    cyw43_arch_lwip_begin();
    (void) udp_sendto(udp_socket, p, &server_ip, UDP_PORT);
    /* Blink LED briefly to indicate outbound packet (use CYW43 GPIO on Pico W) */
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    sleep_ms(30);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    cyw43_arch_lwip_end();

    /* udp_sendto doesn't free the pbuf, so we must free it. */
    (void) pbuf_free(p);
}

/**
 * @brief Handles UART receive interrupt.
 * 
 * This function is called when data is received on the UART interface.
 * It processes incoming serial data and handles any necessary UART RX operations.
 */
static void on_uart_rx(void)
{
    while (uart_is_readable(UART_ID))
    {
        const uint8_t ch = uart_getc(UART_ID);
        const uint16_t next_head = (head + 1) & RING_BUFFER_MASK;

        if (next_head != tail)
        {
            ring_buf[head] = ch;
            head = next_head;
        }
    }
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

    /* Set up UART Interrupts */
    const int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
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
        DBG("Couldn't initialize CYW43 driver.");
        return 1;
    }

    /* Indicate we're starting connection attempt (turn LED off) */
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    /* Enable Station mode */
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, WIFI_CONNECT_TIMEOUT_MS))
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        DBG("Couldn't connect to Wi-Fi.");
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
        DBG("Couldn't create new UDP socket.");
        return 1;
    }

    /* Prepare destination IP */
    if (!ipaddr_aton(UDP_SERVER_IP, &server_ip))
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        cyw43_arch_lwip_end();
        DBG("Couldn't convert the IP to binary address.");
        return 1;
    }

    cyw43_arch_lwip_end();

    /* Successful connection: turn LED on */
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    return 0;
}