/**********************************************************************
  R4A_SX1509.h

  Robots-For-All (R4A)
  Declare the SX1509 registers and constants
**********************************************************************/

#ifndef __R4A_SX1509_H__
#define __R4A_SX1509_H__

// Declare the register addresses
#define SX1509_INPUT_DISABLE_B      0x00
#define SX1509_INPUT_DISABLE_A      0x01

#define SX1509_LONG_SLEW_B          0x02
#define SX1509_LONG_SLEW_A          0x03

#define SX1509_LOW_DRIVE_B          0x04
#define SX1509_LOW_DRIVE_A          0x05

#define SX1509_PULL_UP_B            0x06
#define SX1509_PULL_UP_A            0x07

#define SX1509_PULL_DOWN_B          0x08
#define SX1509_PULL_DOWN_A          0x09

#define SX1509_OPEN_DRAIN_B         0x0a
#define SX1509_OPEN_DRAIN_A         0x0b

#define SX1509_POLARITY_B           0x0c
#define SX1509_POLARITY_A           0x0d

#define SX1509_DIR_B                0x0e
#define SX1509_DIR_A                0x0f

#define SX1509_DATA_B               0x10
#define SX1509_DATA_A               0x11

#define SX1509_INTERRUPT_MASK_B     0x12
#define SX1509_INTERRUPT_MASK_A     0x13

#define SX1509_SENSE_HIGH_B         0x14
#define SX1509_SENSE_LOW_B          0x15

#define SX1509_SENSE_HIGH_A         0x16
#define SX1509_SENSE_LOW_A          0x17

#define SX1509_INTERRUPT_SOURCE_B   0x18
#define SX1509_INTERRUPT_SOURCE_A   0x19

#define SX1509_EVENT_STATUS_B       0x1a
#define SX1509_EVENT_STATUS_A       0x1b

#define SX1509_LEVEL_SHIFTER_1      0x1c
#define SX1509_LEVEL_SHIFTER_2      0x1d

#define SX1509_CLOCK                0x1e
#define SX1509_MISC                 0x1f

#define SX1509_LED_DRIVER_ENABLE_B  0x20
#define SX1509_LED_DRIVER_ENABLE_A  0x21

#define SX1509_DEBOUNCE_CONFIG      0x22
#define SX1509_DEBOUNCE_ENABLE_B    0x23

#define SX1509_DEBOUNCE_ENABLE_A    0x24
#define SX1509_KEY_CONFIG_1         0x25

#define SX1509_KEY_CONFIG_2         0x26
#define SX1509_KEY_DATA_1           0x27

#define SX1509_KEY_DATA_2           0x28

#define SX1509_HIGH_INPUT_B         0x69
#define SX1509_HIGH_INPUT_A         0x6A

#define SX1509_RESET                0x7d

#define SX1509_HAS_TRISE_TFALL      0xf0f0

const uint8_t SX1509_TON_ADDR[16] =
{
    0x29, 0x2c, 0x2f, 0x33, 0x36, 0x3a, 0x3f, 0x44,
    0x49, 0x4c, 0x4f, 0x52, 0x55, 0x5a, 0x5f, 0x64
};

#endif  // __R4A_SX1509_H__
