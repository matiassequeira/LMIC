/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#include <qm_interrupt.h>
#include <qm_pinmux.h>
#include <qm_gpio.h>
#include <qm_spi.h>
#include <clk.h>
#include <x86intrin.h>

#include "../lmic.h"
#include "hal.h"
#include <stdio.h>

#define static_assert(x, y) QM_ASSERT(x)

uint32_t micros() {
    uint32_t us = _rdtsc() / SYS_TICKS_PER_US_32MHZ;
    return us;
}

void delayMicroseconds(uint32_t us) {
    clk_sys_udelay(us);
}

void delay(uint32_t ms) {
    delayMicroseconds(ms * 1000);
}

typedef enum {
    INPUT,
    OUTPUT
} pinmode_t;

static qm_gpio_port_config_t gpio_cfg = { 0 };

void pinMode(uint8_t pin, pinmode_t mode) {
    qm_pmux_pullup_en(pin, false);
    qm_pmux_select(pin, QM_PMUX_FN_0);

    switch (mode) {
    case INPUT:
        qm_pmux_input_en(pin, true);
        gpio_cfg.direction &= ~BIT(pin);
        break;

    case OUTPUT:
    	gpio_cfg.direction |= BIT(pin);
        break;
    }

    qm_gpio_set_config(QM_GPIO_0, &gpio_cfg);
}

void digitalWrite(uint8_t pin, uint8_t val) {
    if (val > 0) {
	qm_gpio_set_pin(QM_GPIO_0, pin);
    } else {
    	qm_gpio_clear_pin(QM_GPIO_0, pin);
    }
}

int digitalRead(uint8_t pin) {
    qm_gpio_state_t state;
    qm_gpio_read_pin(QM_GPIO_0, pin, &state);
    return state;
}

// -----------------------------------------------------------------------------
// I/O
static bool dio_states[NUM_DIO];

static void hal_io_init () {
    // NSS and DIO0 are required, DIO1 is required for LoRa, DIO2 for FSK
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[0] != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio[1] != LMIC_UNUSED_PIN || lmic_pins.dio[2] != LMIC_UNUSED_PIN);

    pinMode(lmic_pins.nss, OUTPUT);
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rxtx, OUTPUT);
    if (lmic_pins.rst != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.rst, OUTPUT);

    pinMode(lmic_pins.dio[0], INPUT);
    if (lmic_pins.dio[1] != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.dio[1], INPUT);
    if (lmic_pins.dio[2] != LMIC_UNUSED_PIN)
        pinMode(lmic_pins.dio[2], INPUT);

    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
		if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
			continue;

		dio_states[i] = digitalRead(lmic_pins.dio[i]);
    }

    // LED
    pinMode(24, OUTPUT);
}

// val == 1  => tx 1
void hal_pin_rxtx (u1_t val) {
    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
        digitalWrite(lmic_pins.rxtx, val);
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if (lmic_pins.rst == LMIC_UNUSED_PIN)
        return;

    if(val == 0 || val == 1) { // drive pin
        pinMode(lmic_pins.rst, OUTPUT);
        digitalWrite(lmic_pins.rst, val);
    } else { // keep pin floating
        pinMode(lmic_pins.rst, INPUT);
    }
}

static void hal_io_check() {
#if USE_NO_DIO
	// Check IRQ flags in radio module
	if (radio_has_irq())
		radio_irq_handler(0);
#else
    uint8_t i, val;
    for (i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] == LMIC_UNUSED_PIN) {
        	continue;
        } else {
			val = digitalRead(lmic_pins.dio[i]);
			if (dio_states[i] != val) {
				dio_states[i] = val;
				if (dio_states[i]) {
					radio_irq_handler(i);
				}
			}
        }
    }
#endif
}

// -----------------------------------------------------------------------------
// SPI

typedef struct {
    void (*begin)();
    void (*beginTransaction)(const qm_spi_config_t *settings);
    uint8_t (*transfer)(uint8_t data);
    void (*endTransaction)();
} spi_t;

#define spi_HIGH 1

#define spi_SCK QM_PIN_ID_16
#define spi_MOSI QM_PIN_ID_17
#define spi_MISO QM_PIN_ID_18

#define SPI_CLOCK_1MHZ_DIV (32)

static void spi_begin() {
}

static void spi_beginTransaction(const qm_spi_config_t *settings) {
    /* Initialise SPI configuration. */
    int ret __attribute__ ((unused));
    ret = qm_spi_set_config(QM_SPI_MST_0, settings);
    QM_ASSERT(0 == ret);

    ret = qm_spi_slave_select(QM_SPI_MST_0, QM_SPI_SS_0);
    QM_ASSERT(0 == ret);
}

static uint8_t spi_transfer(uint8_t data) {
    uint8_t rxBuf;
    qm_spi_transfer_t xfer;
    qm_spi_status_t status;

    xfer.tx = &data;
    xfer.rx = &rxBuf;
    xfer.tx_len = 1;
    xfer.rx_len = 1;

    int ret = qm_spi_transfer(QM_SPI_MST_0, &xfer, &status);
    ASSERT(0 == ret);
    return rxBuf;
}

static void spi_endTransaction() {

}

spi_t SPI;
qm_spi_config_t settings;

static void hal_spi_init () {
    qm_pmux_select(spi_SCK, QM_PMUX_FN_2);   /* SCK */
    qm_pmux_select(spi_MOSI, QM_PMUX_FN_2);  /* MOSI */
    qm_pmux_select(spi_MISO, QM_PMUX_FN_2);  /* MISO */
    qm_pmux_input_en(spi_MISO, true);        /* MISO input */

#define MSBFIRST 1
    settings.frame_size = QM_SPI_FRAME_SIZE_8_BIT;
    settings.transfer_mode = QM_SPI_TMOD_TX_RX;
    settings.bus_mode = QM_SPI_BMODE_0;
    settings.clk_divider = SPI_CLOCK_1MHZ_DIV;

    SPI.begin = spi_begin;
    SPI.beginTransaction = spi_beginTransaction;
    SPI.transfer = spi_transfer;
    SPI.endTransaction = spi_endTransaction;

    SPI.begin();
    SPI.beginTransaction(&settings);
}

void hal_pin_nss (u1_t val) {
    digitalWrite(lmic_pins.nss, val);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {

    u1_t res = SPI.transfer(out);
    return res;
}

// -----------------------------------------------------------------------------
// TIME
static void hal_time_init () {
    // Nothing to do
}

u4_t hal_ticks () {
    // Because micros() is scaled down in this function, micros() will
    // overflow before the tick timer should, causing the tick timer to
    // miss a significant part of its values if not corrected. To fix
    // this, the "overflow" serves as an overflow area for the micros()
    // counter. It consists of three parts:
    //  - The US_PER_OSTICK upper bits are effectively an extension for
    //    the micros() counter and are added to the result of this
    //    function.
    //  - The next bit overlaps with the most significant bit of
    //    micros(). This is used to detect micros() overflows.
    //  - The remaining bits are always zero.
    //
    // By comparing the overlapping bit with the corresponding bit in
    // the micros() return value, overflows can be detected and the
    // upper bits are incremented. This is done using some clever
    // bitwise operations, to remove the need for comparisons and a
    // jumps, which should result in efficient code. By avoiding shifts
    // other than by multiples of 8 as much as possible, this is also
    // efficient on AVR (which only has 1-bit shifts).
    static uint8_t overflow = 0;

    // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
    // the others will be the lower bits of our return value.
    uint32_t scaled = micros() >> US_PER_OSTICK_EXPONENT;
    // Most significant byte of scaled
    uint8_t msb = scaled >> 24;
    // Mask pointing to the overlapping bit in msb and overflow.
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    // Update overflow. If the overlapping bit is different
    // between overflow and msb, it is added to the stored value,
    // so the overlapping bit becomes equal again and, if it changed
    // from 1 to 0, the upper bits are incremented.
    overflow += (msb ^ overflow) & mask;

    // Return the scaled value with the upper bits of stored added. The
    // overlapping bit will be equal and the lower bits will be 0, so
    // bitwise or is a no-op for them.
    return scaled | ((uint32_t)overflow << 24);

    // 0 leads to correct, but overly complex code (it could just return
    // micros() unmodified), 8 leaves no room for the overlapping bit.
    static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_waitUntil (u4_t time) {
    s4_t delta = delta_time(time);
    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383.
    while (delta > (16000 / US_PER_OSTICK)) {
        delay(16);
        delta -= (16000 / US_PER_OSTICK);
    }
    if (delta > 0)
        delayMicroseconds(delta * US_PER_OSTICK);
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    qm_irq_disable();
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        qm_irq_enable();

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}

void hal_sleep () {
    // Not implemented
}

// -----------------------------------------------------------------------------

#if defined(LMIC_PRINTF_TO)
static int uart_putchar (char c, FILE *)
{
    LMIC_PRINTF_TO.write(c) ;
    return 0 ;
}

void hal_printf_init() {
    // create a FILE structure to reference our UART output function
    static FILE uartout;
    memset(&uartout, 0, sizeof(uartout));

    // fill in the UART file descriptor with pointer to writer.
    fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

    // The uart is the standard output device STDOUT.
    stdout = &uartout ;
}
#endif // defined(LMIC_PRINTF_TO)

void hal_init () {
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
#if defined(LMIC_PRINTF_TO)
    // printf support
    hal_printf_init();
#endif
}

void hal_failed (const char *file, u2_t line) {
#if defined(LMIC_FAILURE_TO)
    LMIC_FAILURE_TO.println("FAILURE ");
    LMIC_FAILURE_TO.print(file);
    LMIC_FAILURE_TO.print(':');
    LMIC_FAILURE_TO.println(line);
    LMIC_FAILURE_TO.flush();
#endif
    hal_disableIRQs();
    while(1);
}
