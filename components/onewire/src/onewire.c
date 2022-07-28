/**
* \file     onewire.c
* \brief    This is the source code for the onewire driver
*
*	Created on: 13 feb. 2022
* 		Author: Gonzalo Rivera
*/
/******************************************************************************
        Interfaces
 ******************************************************************************/
#include "stdbool.h"
#include "string.h"
#include "stdint.h"

#include "onewire.h"
/******************************************************************************
        Defines and constants
 ******************************************************************************/
static const char * MODULE_NAME 	= "[onewire]";
static const char * DRIVER_VERSION 	= "onewire_v1.0.0";

/******************************************************************************
        Data types
 ******************************************************************************/

/******************************************************************************
        Local function prototypes
 ******************************************************************************/
static inline 	bool 	onewire_wait_for_bus	(one_wire_t *owire, int max_wait);
static 			bool 	onewire_write_bit		(one_wire_t *owire, bool value);
static 			int		onewire_read_bit		(one_wire_t *owire);
/******************************************************************************
        Local variables
 ******************************************************************************/

/******************************************************************************
        Public function definitions
 ******************************************************************************/
// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return false;
//
// Returns true if a device asserted a presence pulse, false otherwise.
//
bool onewire_reset(one_wire_t *owire)
{
	bool ret = false;

	// gpio config input
	owire->gpio_setup(owire->gpio_pin, GPIO_INPUT_OUTPUT_OPENDRAIN);	// set pin with pullup ever.

	owire->gpio_set_level(owire->gpio_pin, HIGH);

    // wait until the wire is high... just in case
    if (!onewire_wait_for_bus(owire, 250))
        return false;

    owire->gpio_set_level(owire->gpio_pin, LOW);
    owire->delay_us(480);

    // PORT_ENTER_CRITICAL;
    owire->gpio_set_level(owire->gpio_pin, HIGH); // allow it to float
    owire->delay_us(70);
    ret = !owire->gpio_get_level(owire->gpio_pin);
    // PORT_EXIT_CRITICAL;

    // Wait for all devices to finish pulling the bus low before returning
    if (!onewire_wait_for_bus(owire, 410))
        return false;

    return ret;
}

bool onewire_select(one_wire_t *owire)
{
    uint8_t i;
    uint64_t _addr = owire->onewire_addr;

    if (!onewire_write(owire, ONEWIRE_SELECT_ROM))
        return false;

    for (i = 0; i < 8; i++)
    {
        if (!onewire_write(owire, _addr & 0xff))
            return false;
        _addr >>= 8;
    }

    return true;
}

bool onewire_skip_rom(one_wire_t *owire)
{
    return onewire_write(owire, ONEWIRE_SKIP_ROM);
}

// Write a byte. The writing code uses open-drain mode and expects the pullup
// resistor to pull the line high when not driven low.  If you need strong
// power after the write (e.g. DS18B20 in parasite power mode) then call
// onewire_power() after this is complete to actively drive the line high.
//
bool onewire_write(one_wire_t *owire, uint8_t value)
{
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        if (!onewire_write_bit(owire, (bitMask & value)))
            return false;
    }

    return true;
}

bool onewire_write_bytes(one_wire_t *owire, const uint8_t *buf, uint16_t count)
{
    for (size_t i = 0; i < count; i++)
        if (!onewire_write(owire, buf[i]))
            return false;

    return true;
}

// Read a byte
//
uint8_t onewire_read(one_wire_t *owire)
{
    uint8_t read = 0;

    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        int bit = onewire_read_bit(owire);
        if (bit < 0)
            return -1;
        else if (bit)
        	read |= bitMask;
    }
    return read;
}

bool onewire_read_bytes(one_wire_t *owire, uint8_t *buf, uint16_t count)
{
    size_t i;
    int b;

    for (i = 0; i < count; i++)
    {
        b = onewire_read(owire);
        if (b < 0)
            return false;
        buf[i] = b;
    }
    return true;
}

bool onewire_power(one_wire_t *owire)
{
    // Make sure the bus is not being held low before driving it high, or we
    // may end up shorting ourselves out.
    if (!onewire_wait_for_bus(owire, 10))
        return false;

    owire->gpio_setup(owire->gpio_pin, GPIO_OUTPUT);
    owire->gpio_set_level(owire->gpio_pin, HIGH);

    return true;
}

void onewire_depower(one_wire_t *owire)
{
	owire->gpio_setup(owire->gpio_pin, GPIO_INPUT_OUTPUT_OPENDRAIN);
}

void onewire_search_start(one_wire_t *owire)
{
    // reset the search state
    memset(&owire->onewire_search, 0, sizeof(owire->onewire_search));
}

void onewire_search_prefix(one_wire_t *owire, uint8_t family_code)
{
    uint8_t i;

    owire->onewire_search.rom_no[0] = family_code;
    for (i = 1; i < 8; i++)
    {
    	owire->onewire_search.rom_no[i] = 0;
    }
    owire->onewire_search.last_discrepancy = 64;
    owire->onewire_search.last_device_found = false;
}

// Perform a search. If the next device has been successfully enumerated, its
// ROM address will be returned.  If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then ONEWIRE_NONE is returned.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return 1 : device found, ROM number in ROM_NO buffer
//        0 : device not found, end of search
//
bool onewire_search_next(one_wire_t *owire)
{
    //TODO: add more checking for read/write errors
    uint8_t 		id_bit_number;
    uint8_t 		last_zero;
    bool 			search_result = false;
    int 			rom_byte_number;
    int8_t 			id_bit, cmp_id_bit;
    uint64_t 		addr;
    unsigned char 	rom_byte_mask;
    bool 			search_direction;

    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = 0;

    // if the last call was not the last one
    if (!owire->onewire_search.last_device_found)
    {
        // 1-Wire reset
        if (!onewire_reset(owire))
        {
            // reset the search
        	owire->onewire_search.last_discrepancy = 0;
        	owire->onewire_search.last_device_found = false;
            return ONEWIRE_NONE;
        }

        // issue the search command
        onewire_write(owire, ONEWIRE_SEARCH);

        // loop to do the search
        do
        {
            // read a bit and its complement
            id_bit = onewire_read_bit(owire);
            cmp_id_bit = onewire_read_bit(owire);

            if ((id_bit == 1) && (cmp_id_bit == 1))
                break;
            else
            {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit)
                    search_direction = id_bit;  // bit write value for search
                else
                {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number <  owire->onewire_search.last_discrepancy)
                        search_direction = ((owire->onewire_search.rom_no[rom_byte_number] & rom_byte_mask) > 0);
                    else
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == owire->onewire_search.last_discrepancy);

                    // if 0 was picked then record its position in LastZero
                    if (!search_direction)
                        last_zero = id_bit_number;
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction)
                    owire->onewire_search.rom_no[rom_byte_number] |= rom_byte_mask;
                else
                	owire->onewire_search.rom_no[rom_byte_number] &= ~rom_byte_mask;

                // serial number search direction write bit
                onewire_write_bit(owire, search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0)
                {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!(id_bit_number < 65))
        {

            // search successful so set last_discrepancy,last_device_found,search_result
            owire->onewire_search.last_discrepancy = last_zero;

            // check for last device
            if (owire->onewire_search.last_discrepancy == 0)
                owire->onewire_search.last_device_found = true;

            search_result = true;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || !owire->onewire_search.rom_no[0])
    {
    	owire->onewire_search.last_discrepancy = 0;
    	owire->onewire_search.last_device_found = false;
        return ONEWIRE_NONE;
    }
    else
    {
        addr = 0;
        for (rom_byte_number = 7; rom_byte_number >= 0; rom_byte_number--)
        {
            addr = (addr << 8) | owire->onewire_search.rom_no[rom_byte_number];
        }
    }

    owire->onewire_addr = addr;
    return search_result;
}

// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#ifdef CONFIG_ONEWIRE_CRC8_TABLE
// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (c) 2000 Dallas Semiconductor Corporation
static const uint8_t dscrc_table[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

//
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (note: this might better be done without to
// table, it would probably be smaller and certainly fast enough
// compared to all those delayMicrosecond() calls.  But I got
// confused, so I use this table from the examples.)
//
uint8_t onewire_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    while (len--)
        crc = dscrc_table[crc ^ *data++];

    return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but much smaller, than the lookup table.
//
uint8_t onewire_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    while (len--)
    {
        uint8_t inbyte = *data++;
        for (int i = 8; i; i--)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}
#endif

// Compute the 1-Wire CRC16 and compare it against the received CRC.
// Example usage (reading a DS2408):
//    // Put everything in a buffer so we can compute the CRC easily.
//    uint8_t buf[13];
//    buf[0] = 0xF0;    // Read PIO Registers
//    buf[1] = 0x88;    // LSB address
//    buf[2] = 0x00;    // MSB address
//    WriteBytes(net, buf, 3);    // Write 3 cmd bytes
//    ReadBytes(net, buf+3, 10);  // Read 6 data bytes, 2 0xFF, 2 CRC16
//    if (!CheckCRC16(buf, 11, &buf[11])) {
//        // Handle error.
//    }
//
// @param input - Array of bytes to checksum.
// @param len - How many bytes to use.
// @param inverted_crc - The two CRC16 bytes in the received data.
//                       This should just point into the received data,
//                       *not* at a 16-bit integer.
// @param crc - The crc starting value (optional)
// @return 1, iff the CRC matches.
bool onewire_check_crc16(const uint8_t* input, size_t len, const uint8_t* inverted_crc, uint16_t crc_iv)
{
    uint16_t crc = ~onewire_crc16(input, len, crc_iv);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

// Compute a Dallas Semiconductor 16 bit CRC.  This is required to check
// the integrity of data received from many 1-Wire devices.  Note that the
// CRC computed here is *not* what you'll get from the 1-Wire network,
// for two reasons:
//   1) The CRC is transmitted bitwise inverted.
//   2) Depending on the endian-ness of your processor, the binary
//      representation of the two-byte return value may have a different
//      byte order than the two bytes you get from 1-Wire.
// @param input - Array of bytes to checksum.
// @param len - How many bytes to use.
// @param crc - The crc starting value (optional)
// @return The CRC16, as defined by Dallas Semiconductor.
uint16_t onewire_crc16(const uint8_t* input, size_t len, uint16_t crc_iv)
{
    uint16_t crc = crc_iv;
    static const uint8_t oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    uint16_t i;
    for (i = 0; i < len; i++)
    {
        // Even though we're just copying a byte from the input,
        // we'll be doing 16-bit computation with it.
        uint16_t cdata = input[i];
        cdata = (cdata ^ crc) & 0xff;
        crc >>= 8;

        if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
            crc ^= 0xC001;

        cdata <<= 6;
        crc ^= cdata;
        cdata <<= 1;
        crc ^= cdata;
    }
    return crc;
}
/******************************************************************************
        Private function definitions
 ******************************************************************************/
// Waits up to `max_wait` microseconds for the specified pin to go high.
// Returns true if successful, false if the bus never comes high (likely
// shorted).
static inline bool onewire_wait_for_bus(one_wire_t *owire, int max_wait)
{
    bool state;
    for (int i = 0; i < ((max_wait + 4) / 5); i++)
    {
        if (owire->gpio_get_level(owire->gpio_pin))
            break;
        owire->delay_us(5);
    }
    state = owire->gpio_get_level(owire->gpio_pin);
    // Wait an extra 1us to make sure the devices have an adequate recovery
    // time before we drive things low again.
    owire->delay_us(2);	//
    return state;
}

static bool onewire_write_bit(one_wire_t *owire, bool value)
{
    if (!onewire_wait_for_bus(owire, 10))
        return false;

    if (value)
    {
        owire->gpio_set_level(owire->gpio_pin, LOW);
        owire->delay_us(10);
        owire->gpio_set_level(owire->gpio_pin, HIGH);
        owire->delay_us(55);
    }
    else
    {
    	owire->gpio_set_level(owire->gpio_pin, LOW);
    	owire->delay_us(65);
    	owire->gpio_set_level(owire->gpio_pin, HIGH);
    }
    owire->delay_us(1);

    return true;
}

static int onewire_read_bit(one_wire_t *owire)
{
    int ret = 0;

	if (!onewire_wait_for_bus(owire, 10))
        return -1;

    owire->gpio_set_level(owire->gpio_pin, LOW);
    owire->delay_us(3); //2
    owire->gpio_set_level(owire->gpio_pin, HIGH);
    owire->delay_us(11);
    owire->gpio_get_level(owire->gpio_pin);

    ret = owire->gpio_get_level(owire->gpio_pin);
    owire->delay_us(48);

    return ret;
}

/* End of file ***************************************************************/

