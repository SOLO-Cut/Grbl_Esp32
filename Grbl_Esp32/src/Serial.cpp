/*
  Serial.cpp - UART0 (USB-serial console) and UART1 (master controller link).

  Part of Grbl_ESP32
  2018 - Bart Dring

  Grbl is free software: you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version. See <http://www.gnu.org/licenses/>.

  Both ports are driven by the ESP-IDF UART driver. Each port has a receive
  task that blocks on the driver's event queue (filled from the UART ISR), so
  nothing is polled. Realtime characters (? ! ~ ctrl-x and the 0x8x/0x9x
  overrides) are acted on immediately inside that task; everything else is
  queued into a per-port line buffer and the main loop is notified. The task
  never blocks, so realtime characters always get through.
*/

#include "Grbl.h"
#include <driver/uart.h>
#include <driver/gpio.h>

static const int UART_DRIVER_BUF = 1024;

typedef struct {
    uart_port_t   port;
    QueueHandle_t events;
    uint8_t       buf[RX_BUFFER_SIZE];
    volatile int  head;
    volatile int  tail;
    volatile bool overflow;  // Bytes were dropped because the host ignored flow control
    portMUX_TYPE  lock;
} serial_port_t;

static serial_port_t ports[CLIENT_COUNT];

static bool is_realtime_command(uint8_t data) {
    if (data >= 0x80) {
        return true;
    }
    auto cmd = static_cast<Cmd>(data);
    return cmd == Cmd::Reset || cmd == Cmd::StatusReport || cmd == Cmd::CycleStart || cmd == Cmd::FeedHold;
}

static void execute_realtime_command(Cmd command, uint8_t client) {
    switch (command) {
        case Cmd::Reset:
            mc_reset();
            break;
        case Cmd::StatusReport:
            report_realtime_status(client);  // Answered directly, even during long sequences
            break;
        case Cmd::CycleStart:
            rt_set(ExecFlag::CycleStart);
            break;
        case Cmd::FeedHold:
            rt_set(ExecFlag::FeedHold);
            break;
        case Cmd::JogCancel:
            if (sys.state == State::Jog) {
                rt_set(ExecFlag::MotionCancel);
            }
            break;
        case Cmd::FeedOvrReset:
            sys_rt_f_override = FEED_OVERRIDE_DEFAULT;
            break;
        case Cmd::FeedOvrCoarsePlus:
            sys_rt_f_override = MIN(FEED_OVERRIDE_MAX, sys_rt_f_override + FEED_OVERRIDE_COARSE);
            break;
        case Cmd::FeedOvrCoarseMinus:
            sys_rt_f_override = MAX(FEED_OVERRIDE_MIN, sys_rt_f_override - FEED_OVERRIDE_COARSE);
            break;
        case Cmd::FeedOvrFinePlus:
            sys_rt_f_override = MIN(FEED_OVERRIDE_MAX, sys_rt_f_override + FEED_OVERRIDE_FINE);
            break;
        case Cmd::FeedOvrFineMinus:
            sys_rt_f_override = MAX(FEED_OVERRIDE_MIN, sys_rt_f_override - FEED_OVERRIDE_FINE);
            break;
        case Cmd::RapidOvrReset:
            sys_rt_r_override = RAPID_OVERRIDE_DEFAULT;
            break;
        case Cmd::RapidOvrMedium:
            sys_rt_r_override = RAPID_OVERRIDE_MEDIUM;
            break;
        case Cmd::RapidOvrLow:
            sys_rt_r_override = RAPID_OVERRIDE_LOW;
            break;
        default:
            break;
    }
    protocol_notify();
}

static int buffer_free(serial_port_t* p) {
    int used = p->head - p->tail;
    if (used < 0) {
        used += RX_BUFFER_SIZE;
    }
    return RX_BUFFER_SIZE - 1 - used;
}

// Never blocks: realtime characters behind a full buffer must still get through.
// A host that respects Grbl flow control (Bf: / ok counting) never fills it.
static void push_char(uint8_t client, uint8_t c) {
    serial_port_t* p = &ports[client];
    portENTER_CRITICAL(&p->lock);
    if (buffer_free(p) == 0) {
        p->overflow = true;
    } else {
        p->buf[p->head] = c;
        p->head         = (p->head + 1) % RX_BUFFER_SIZE;
    }
    portEXIT_CRITICAL(&p->lock);
}

static void rx_task(void* arg) {
    uint8_t        client = (uint8_t)(uintptr_t)arg;
    serial_port_t* p      = &ports[client];
    uart_event_t   event;
    uint8_t        data[64];
    for (;;) {
        if (xQueueReceive(p->events, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (event.type) {
            case UART_DATA: {
                size_t avail = 0;
                uart_get_buffered_data_len(p->port, &avail);
                while (avail > 0) {
                    int n = uart_read_bytes(p->port, data, MIN(avail, sizeof(data)), 0);
                    if (n <= 0) {
                        break;
                    }
                    avail -= n;
                    bool queued = false;
                    for (int i = 0; i < n; i++) {
                        if (is_realtime_command(data[i])) {
                            execute_realtime_command(static_cast<Cmd>(data[i]), client);
                        } else {
                            push_char(client, data[i]);
                            queued = true;
                        }
                    }
                    if (queued) {
                        protocol_notify();
                    }
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(p->port);
                xQueueReset(p->events);
                break;
            default:
                break;
        }
    }
}

static void port_init(uint8_t client, int uart_num, int tx, int rx) {
    serial_port_t* p = &ports[client];
    p->port          = (uart_port_t)uart_num;
    p->head = 0;
    p->tail = 0;
    p->overflow       = false;
    p->lock           = portMUX_INITIALIZER_UNLOCKED;

    uart_config_t conf = {};
    conf.baud_rate     = BAUD_RATE;
    conf.data_bits     = UART_DATA_8_BITS;
    conf.parity        = UART_PARITY_DISABLE;
    conf.stop_bits     = UART_STOP_BITS_1;
    conf.flow_ctrl     = UART_HW_FLOWCTRL_DISABLE;
    conf.source_clk    = UART_SCLK_DEFAULT;
    if (uart_is_driver_installed(p->port)) {
        uart_driver_delete(p->port);
    }
    ESP_ERROR_CHECK(uart_driver_install(p->port, UART_DRIVER_BUF, UART_DRIVER_BUF, 20, &p->events, 0));
    ESP_ERROR_CHECK(uart_param_config(p->port, &conf));
    ESP_ERROR_CHECK(uart_set_pin(p->port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // The IDF driver leaves RX floating. An unconnected RX line (e.g. the master
    // link with no master attached) then receives noise, and any 0x18 byte is a
    // Ctrl-X soft reset. Keep the idle line high.
    gpio_pullup_en((gpio_num_t)rx);
    uart_set_rx_timeout(p->port, 2);  // Deliver UART_DATA events after 2 idle symbols

    xTaskCreatePinnedToCore(rx_task, client == CLIENT_MASTER ? "rxMaster" : "rxConsole", 4096, (void*)(uintptr_t)client,
                            SERIAL_TASK_PRIORITY, NULL, GRBL_CORE);
}

void serial_init() {
    port_init(CLIENT_CONSOLE, CONSOLE_UART_NUM, CONSOLE_TX_PIN, CONSOLE_RX_PIN);
    port_init(CLIENT_MASTER, MASTER_UART_NUM, MASTER_TX_PIN, MASTER_RX_PIN);
    serial_write(CLIENT_ALL, "\r\n");
}

int serial_read(uint8_t client) {
    serial_port_t* p = &ports[client];
    int            c = -1;
    portENTER_CRITICAL(&p->lock);
    if (p->tail != p->head) {
        c       = p->buf[p->tail];
        p->tail = (p->tail + 1) % RX_BUFFER_SIZE;
    }
    portEXIT_CRITICAL(&p->lock);
    return c;
}

bool serial_take_overflow(uint8_t client) {
    bool o                  = ports[client].overflow;
    ports[client].overflow  = false;
    return o;
}

int serial_rx_available(uint8_t client) {
    return buffer_free(&ports[client]);
}

void serial_reset_read_buffer(uint8_t client) {
    for (uint8_t c = 0; c < CLIENT_COUNT; c++) {
        if (client == c || client == CLIENT_ALL) {
            portENTER_CRITICAL(&ports[c].lock);
            ports[c].head = 0;
            ports[c].tail = 0;
            portEXIT_CRITICAL(&ports[c].lock);
        }
    }
}

void serial_write(uint8_t client, const char* text) {
    size_t len = strlen(text);
    for (uint8_t c = 0; c < CLIENT_COUNT; c++) {
        if (client == c || client == CLIENT_ALL) {
            if (ports[c].events) {
                uart_write_bytes(ports[c].port, text, len);
            }
        }
    }
}
