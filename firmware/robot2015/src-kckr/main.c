#include <stdbool.h>

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "kicker_commands.h"
#include "pins.h"

#define NO_COMMAND 0

// for timer loop that handles kicking timing
#define TIMING_CONSTANT 100 // with clk speed at 1 MHz, 1 clk == 1 us, update every 100 us, 0.1 ms
#define CLK_SPEED_HZ 1000000
#define TIMER_DELTA ((1.0f * TIMING_CONSTANT / CLK_SPEED_HZ) * 1000.0f);

// for main loop that handles ball sense readings
#define LOOP_DELTA 100

#define VOLTAGE_READ_DELAY_MS 40

#define BALL_SENSE_MAX_SAMPLES 5

// Msed to time kick and chip durations
volatile float pre_kick_cooldown_ = 0.0f;
volatile float millis_left_ = 0.0f;
volatile float post_kick_cooldown_ = 0.0f;
volatile float kick_wait = 0.0f;

// Msed to keep track of current button state
volatile int kick_db_down_ = 0;
volatile int chip_db_down_ = 0;
volatile int charge_db_down_ = 0;

volatile uint8_t byte_cnt = 0;

volatile uint8_t cur_command_ = NO_COMMAND;

// always up-to-date voltage so we don't have to get_voltage() inside interupts
volatile uint8_t last_voltage_ = 0;

volatile bool ball_sensed_ = 0;

// whether or not MBED has requested charging to be on
volatile bool charge_commanded_ = false;

volatile bool charge_allowed_ = true;

volatile bool kick_on_breakbeam_ = false;
volatile uint8_t kick_on_breakbeam_strength_ = 0;

unsigned ball_sense_change_count_ = 0;

unsigned time = 0;

// executes a command coming from SPI
uint8_t execute_cmd(uint8_t, uint8_t);

bool is_kicking() {
    return pre_kick_cooldown_ > 0 || millis_left_ > 0 || post_kick_cooldown_ > 0 || kick_wait > 0;
}

void kick(uint8_t strength) {
    if (is_kicking()) return;
    pre_kick_cooldown_ = 5.0f;
    // minimum of 6 ms, we were breaking kickers with low duty cycles
    // maximum of 6 + 7 == 13 ms
    //millis_left_ = (int) ((strength / 255.0) * 6.0) + 7;
    millis_left_ = 1.0f;
    post_kick_cooldown_ = 5.0f;
    kick_wait = 2000.0f;

    TCCR0B |= _BV(CS00);     // start timer, no prescale
}

void init();

/* Voltage Function */
uint8_t get_voltage() {
    // Start conversation by writing to start bit
    ADCSRA |= _BV(ADSC);

    // Wait for ADSC bit to clear
    while (ADCSRA & _BV(ADSC))
        ;

    // ADHC will range from 0 to 255 corresponding to 0 through VCC
    return ADCH;
}

void main() {
    init();

    const int kalpha = 32;

    // We handle voltage readings here
    while (true) {
        // get a voltage reading by weighing in a new reading, same concept as
        // TCP RTT estimates (exponentially weighted sum)

        if (time % (VOLTAGE_READ_DELAY_MS * 10) == 0) {
            int voltage_accum =
                (255 - kalpha) * last_voltage_ + kalpha * get_voltage();
            last_voltage_ = voltage_accum / 255;
        }

        // if we dropped below acceptable voltage, then this will catch it
        if (last_voltage_ > 239 || !charge_allowed_ || !charge_commanded_) {
            PORTB &= ~(_BV(CHARGE_PIN));
        } else if (last_voltage_ < 232 && charge_allowed_ && charge_commanded_) {
            PORTB |= _BV(CHARGE_PIN);
        }

        if (PINA & _BV(N_KICK_CS_PIN)) {
            byte_cnt = 0;
        }

        bool bs = PINB & _BV(BALL_SENSE_RX);
        if (ball_sensed_) {
            if (!bs) ball_sense_change_count_++; // wrong reading, inc counter
            else ball_sense_change_count_ = 0; // correct reading, reset counter
        } else {
            if (bs) ball_sense_change_count_++; // wrong reading, inc counter
            else ball_sense_change_count_ = 0; // correct reading, reset counter
        }

        // counter exceeds maximium, so reset
        if (ball_sense_change_count_ > BALL_SENSE_MAX_SAMPLES) {
            ball_sense_change_count_ = 0;

            ball_sensed_ = !ball_sensed_;
        }

        if (ball_sensed_ && kick_on_breakbeam_) {
            // pow
            kick(kick_on_breakbeam_strength_);
            kick_on_breakbeam_ = false;
        }

        _delay_us(LOOP_DELTA); // 0.1 ms
    }
}

void init() {
    cli(); // disable interrupts

    // disable watchdog
    wdt_reset();
    MCUSR &= ~(_BV(WDRF));
    WDTCR |= (_BV(WDCE)) | (_BV(WDE));
    WDTCR = 0x00;
    /* Outputs */
    DDRA |= _BV(KICK_MISO_PIN);
    DDRB |= _BV(KICK_PIN) | _BV(CHARGE_PIN) | _BV(BALL_SENSE_TX);

    PORTB |= _BV(BALL_SENSE_TX);

    /* Inputs */
    DDRA &= ~(_BV(N_KICK_CS_PIN) | _BV(V_MONITOR_PIN) | _BV(KICK_MOSI_PIN));

    PORTB &= ~(_BV(BALL_SENSE_RX));
    DDRB &= ~(_BV(BALL_SENSE_RX));

    /* SPI Init */
    SPCR = _BV(SPE) | _BV(SPIE);
    SPCR &= ~(_BV(MSTR)); // ensure we are a slave SPI device

    // enable interrupts for PCINT0-PCINT7
    PCICR |= _BV(PCIE0);

    // enable interrupts on debug buttons
    PCMSK0 = _BV(INT_DB_KICK) | _BV(INT_DB_CHG);

    // Set low bits corresponding to pin we read from
    ADMUX |= _BV(ADLAR) | 0x00; // connect PA0 (V_MONITOR_PIN) to ADC

    // Interrupt on TIMER 0
    TIMSK0 |= _BV(OCIE0A);

    // CTC - Clear Timer on Compare Match
    TCCR0A |= _BV(WGM01);

    // OCR0A is max val of timer before reset
    // we need 100 clocks at 1 Mhz to get 0.1 millisecond
    OCR0A = TIMING_CONSTANT;  // reset every 100 us

    // ensure ADC isn't shut off
    PRR &= ~_BV(PRADC);
    ADCSRA |= _BV(ADEN);   // enable the ADC - Pg. 133

    // enable global interrupts
    sei();
}

/*
 * SPI Interrupt. Triggers when we have a new byte available, it'll be
 * stored in SPDR. Writing a response also occurs msing the SPDR register.
 */
ISR(SPI_STC_vect) {
    uint8_t recv_data = SPDR;

    SPDR = 0xFF;
    // increment our received byte count and take appropriate action
    if (byte_cnt == 0) {
        cur_command_ = recv_data;
        // kicker statms fields
        SPDR |= ACK;
    } else if (byte_cnt == 1) {
        // execute the currently set command with
        // the newly given argument, set the response
        // buffer to our return value
        SPDR = execute_cmd(cur_command_, recv_data);
    } else if (byte_cnt == 2) {
        SPDR = ((ball_sensed_ ? 1 : 0) << BALL_SENSE_FIELD)
            | ((charge_commanded_ ? 1 : 0) << CHARGE_FIELD)
            | ((kick_on_breakbeam_ ? 1 : 0) << KICK_ON_BREAKBEAM_FIELD)
            | ((is_kicking() ? 1 : 0) << KICKING_FIELD);
    } else if (byte_cnt == 4) {
        // no-op
    }
    int NUM_BYTES = 4;
    byte_cnt++;
    byte_cnt %= NUM_BYTES;
}

/*
 * Interrupt if the state of any button has changed
 * Every time a button goes from LOW to HIGH, we will execute a command
 *
 * ISR for PCINT8 - PCINT11
 */
ISR(PCINT0_vect) {
    // First we get the current state of each button, active low
    int dbg_switched = !(PINB & _BV(DB_SWITCH));

    if (!dbg_switched) return;
    int kick_db_pressed = !(PINA & _BV(DB_KICK_PIN));
    int charge_db_pressed = !(PINA & _BV(DB_CHG_PIN));

    if (!kick_db_down_ && kick_db_pressed)
        kick(255);

    // toggle charge
    if (!charge_db_down_ && charge_db_pressed) {
        // check if charge is already on, toggle appropriately
        charge_commanded_ = !charge_commanded_;
    }

    // Now our last state becomes the current state of the buttons
    kick_db_down_ = kick_db_pressed;
    charge_db_down_ = charge_db_pressed;
}

/*
 * Timer interrupt for chipping/kicking - called every millisecond by timer
 *
 * ISR for TIMER 0
 */
ISR(TIMER0_COMPA_vect) {
    if (pre_kick_cooldown_ > 0) {
        // disable charging
        charge_allowed_ = false; // prevent main loop from changing it
        PORTB &= ~(_BV(CHARGE_PIN)); // stop charging immediately
        pre_kick_cooldown_ -= TIMER_DELTA;
    } else if (millis_left_ > 0) {
        PORTB |= _BV(KICK_PIN);  // set KICK pin
        millis_left_ -= TIMER_DELTA;
    } else if (post_kick_cooldown_ > 0) {
        // kick is done, keep it off
        PORTB &= ~_BV(KICK_PIN);
        post_kick_cooldown_ -= TIMER_DELTA;
    } else if (kick_wait > 0) {
        // don't allow super repeated kicking, just chill in this loop for a bit
        charge_allowed_ = true; // let main loop take care of setting it high
        kick_wait -= TIMER_DELTA;
    } else {
        // stop prescaled timer
        TCCR0B &= ~_BV(CS00);
    }
}

/*
 * Executes a command that can come from SPI or a debug button
 *
 * WARNING: This will be called from an interrupt service routines, keep it
 * short!
 */
uint8_t execute_cmd(uint8_t cmd, uint8_t arg) {
    // if we don't change ret_val by setting it to voltage or
    // something, then we'll jmst return the command we got as
    // an acknowledgement.
    uint8_t ret_val = BLANK;

    switch (cmd) {
        case KICK_BREAKBEAM_CMD:
            kick_on_breakbeam_ = true;
            kick_on_breakbeam_strength_ = arg;
            break;

        case KICK_BREAKBEAM_CANCEL_CMD:
            kick_on_breakbeam_ = false;
            kick_on_breakbeam_strength_ = 0;
            break;

        case KICK_IMMEDIATE_CMD:
            kick(arg);
            break;

        case SET_CHARGE_CMD:
            // set state based on argument
            if (arg == ON_ARG) {
                ret_val = 1;
                charge_commanded_ = true;
            } else if (arg == OFF_ARG) {
                ret_val = 0;
                charge_commanded_ = false;
            }
            break;

        case GET_VOLTAGE_CMD:
            ret_val = last_voltage_;
            break;

        case PING_CMD:
            ret_val = 0xFF;
            // do nothing, ping is jmst a way to check if the kicker
            // is connected by checking the returned command ack from
            // earlier.
            break;

        default:
            // return error value to show arg wasn't recognized
            ret_val = 0xCC;  // return a weird value to show arg wasn't recognized
            break;
    }

    return ret_val;
}
