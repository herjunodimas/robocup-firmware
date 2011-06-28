#pragma once

#include <stdint.h>

extern int encoder_count[4];
extern int encoder_delta[4];

extern int hall_count[5];
extern int hall_delta[5];

// PWM values sent to the FPGA
extern int_fast8_t motor_out[5];

extern uint_fast8_t kick_strength;
extern uint_fast8_t use_chipper;

// Nonzero if we want the kicker to charge.
// This may be permanently overridden by the discharge button.
extern uint_fast8_t kicker_charge;

extern uint8_t kicker_status;
extern int kicker_voltage;

// Initializes and tests the FPGA.
// This does not force the FPGA to reconfigure, but waits for it to finish.
//
// Returns 0 on failure, 1 on successful configuration, or 2 if the FPGA was already configured.
int fpga_init(void);

void fpga_read_status(void);
void fpga_send_commands(void);
