
#ifndef COMPONENTS_MODULENAME_INCLUDE_MODULENAME_H_
#define COMPONENTS_MODULENAME_INCLUDE_MODULENAME_H_

#include <stdio.h>
#include <stdint.h>

#include <onewire.h>
/******************************************************************************
        Constants
 ******************************************************************************/
/** An address value which can be used to indicate "any device on the bus" */
#define DS18X20_ANY ONEWIRE_NONE

/** Family ID (lower address byte) of DS18B20 sensors */
#define DS18B20_FAMILY_ID 0x28

/** Family ID (lower address byte) of DS18S20 sensors */
#define DS18S20_FAMILY_ID 0x10
/******************************************************************************
        Data types
 ******************************************************************************/
typedef enum
{
	DS18x20_OK = 0,
	DS18x20_ERR = -1,
}ds18x20_sts_t;

typedef struct
{
	uint64_t 	address;
	uint16_t 	temp_raw;
	float		temp;

	one_wire_t one_wire;

	/* delay function pointer */
	void (*delay_ms)(uint32_t msec);
}ds18x20_t;
/******************************************************************************
        Public function prototypes
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Find the addresses of all ds18x20 devices on the bus.
 *
 * Scans the bus for all devices and places their addresses in the supplied
 * array. If there are more than `addr_count` devices on the bus, only the
 * first `addr_count` are recorded.
 *
 * @param pin         The GPIO pin connected to the ds18x20 bus
 * @param addr_list   A pointer to an array of ::ds18x20_addr_t values.
 *                    This will be populated with the addresses of the found
 *                    devices.
 * @param addr_count  Number of slots in the `addr_list` array. At most this
 *                    many addresses will be returned.
 * @param found       The number of devices found. Note that this may be less
 *                    than, equal to, or more than `addr_count`, depending on
 *                    how many ds18x20 devices are attached to the bus.
 *
 * @returns `ESP_OK` if the command was successfully issued
 */
int ds18x20_scan_devices(ds18x20_t *ds18x20, uint64_t *addr_list ,uint8_t addr_count, uint8_t *found);

/**
 * @brief Tell one or more sensors to perform a temperature measurement and
 * conversion (CONVERT_T) operation.
 *
 * This operation can take up to 750ms to complete.
 *
 * If `wait=true`, this routine will automatically drive the pin high for the
 * necessary 750ms after issuing the command to ensure parasitically-powered
 * devices have enough power to perform the conversion operation (for
 * non-parasitically-powered devices, this is not necessary but does not
 * hurt). If `wait=false`, this routine will drive the pin high, but will
 * then return immediately. It is up to the caller to wait the requisite time
 * and then depower the bus using onewire_depower() or by issuing another
 * command once conversion is done.
 *
 * @param pin   The GPIO pin connected to the ds18x20 device
 * @param addr  The 64-bit address of the device on the bus. This can be set
 *              to ::DS18X20_ANY to send the command to all devices on the bus
 *              at the same time.
 * @param wait  Whether to wait for the necessary 750ms for the ds18x20 to
 *              finish performing the conversion before returning to the
 *              caller (You will normally want to do this).
 *
 * @returns `ESP_OK` if the command was successfully issued
 */
int ds18x20_measure(ds18x20_t *ds18x20, bool wait);


/**
 * @brief Read the value from the last CONVERT_T operation.
 *
 * This should be called after ds18x20_measure() to fetch the result of the
 * temperature measurement.
 *
 * @param pin         The GPIO pin connected to the ds18x20 device
 * @param addr        The 64-bit address of the device to read. This can be set
 *                    to ::DS18X20_ANY to read any device on the bus (but note
 *                    that this will only work if there is exactly one device
 *                    connected, or they will corrupt each others' transmissions)
 * @param temperature The temperature in degrees Celsius
 *
 * @returns `ESP_OK` if the command was successfully issued
 */
int ds18x20_read_temperature(ds18x20_t *ds18x20);

/**
 * @brief Read the value from the last CONVERT_T operation (ds18b20 version).
 *
 * This should be called after ds18x20_measure() to fetch the result of the
 * temperature measurement.
 *
 * @param pin         The GPIO pin connected to the ds18x20 device
 * @param addr        The 64-bit address of the device to read. This can be set
 *                    to ::DS18X20_ANY to read any device on the bus (but note
 *                    that this will only work if there is exactly one device
 *                    connected, or they will corrupt each others' transmissions)
 * @param temperature The temperature in degrees Celsius
 *
 * @returns `ESP_OK` if the command was successfully issued
 */
int ds18b20_read_temperature(ds18x20_t *ds18x20);

/**
 * @brief Read the value from the last CONVERT_T operation (ds18s20 version).
 *
 * This should be called after ds18x20_measure() to fetch the result of the
 * temperature measurement.
 *
 * @param pin         The GPIO pin connected to the ds18x20 device
 * @param addr        The 64-bit address of the device to read. This can be set
 *                    to ::DS18X20_ANY to read any device on the bus (but note
 *                    that this will only work if there is exactly one device
 *                    connected, or they will corrupt each others' transmissions)
 * @param temperature The temperature in degrees Celsius
 *
 * @returns `ESP_OK` if the command was successfully issued
 */
int ds18s20_read_temperature(ds18x20_t *ds18x20);

/**
 * @brief Read the value from the last CONVERT_T operation for multiple devices.
 *
 * This should be called after ds18x20_measure() to fetch the result of the
 * temperature measurement.
 *
 * @param pin         The GPIO pin connected to the ds18x20 bus
 * @param addr_list   A list of addresses for devices to read.
 * @param addr_count  The number of entries in `addr_list`.
 * @param result_list An array of floats to hold the returned temperature
 *                     values. It should have at least `addr_count` entries.
 *
 * @returns `ESP_OK` if all temperatures were fetched successfully
 */
int ds18x20_read_temp_multi(ds18x20_t *ds18x20, uint64_t *addr_list, uint8_t addr_count, float *result_list);

/** Perform a ds18x20_measure() followed by ds18s20_read_temperature()
 *
 *  @param pin         The GPIO pin connected to the ds18s20 device
 *  @param addr        The 64-bit address of the device to read. This can be set
 *                     to ::DS18X20_ANY to read any device on the bus (but note
 *                     that this will only work if there is exactly one device
 *                     connected, or they will corrupt each others' transmissions)
 *  @param temperature The temperature in degrees Celsius
 */
int ds18s20_measure_and_read(ds18x20_t *ds18x20);

/** Perform a ds18x20_measure() followed by ds18b20_read_temperature()
 *
 *  @param pin         The GPIO pin connected to the ds18x20 device
 *  @param addr        The 64-bit address of the device to read. This can be set
 *                     to ::DS18X20_ANY to read any device on the bus (but note
 *                     that this will only work if there is exactly one device
 *                     connected, or they will corrupt each others' transmissions)
 *  @param temperature The temperature in degrees Celsius
 */
int ds18b20_measure_and_read(ds18x20_t *ds18x20);

/** Perform a ds18x20_measure() followed by ds18x20_read_temperature()
 *
 *  @param pin         The GPIO pin connected to the ds18x20 device
 *  @param addr        The 64-bit address of the device to read. This can be set
 *                     to ::DS18X20_ANY to read any device on the bus (but note
 *                     that this will only work if there is exactly one device
 *                     connected, or they will corrupt each others' transmissions)
 *  @param temperature The temperature in degrees Celsius
 */
int ds18x20_measure_and_read(ds18x20_t *ds18x20);

/**
 * @brief Perform a ds18x20_measure() followed by ds18x20_read_temp_multi()
 *
 * @param pin         The GPIO pin connected to the ds18x20 bus
 * @param addr_list   A list of addresses for devices to read.
 * @param addr_count  The number of entries in `addr_list`.
 * @param result_list An array of floats to hold the returned temperature
 *                    values. It should have at least `addr_count` entries.
 *
 * @returns `ESP_OK` if all temperatures were fetched successfully
 */

int ds18x20_measure_and_read_multi(ds18x20_t *ds18x20, uint64_t *addr_list, uint8_t addr_count, float *result_list);

/**
 * @brief Read the scratchpad data for a particular ds18x20 device.
 *
 * This is not generally necessary to do directly. It is done automatically
 * as part of ds18x20_read_temperature().
 *
 * @param pin     The GPIO pin connected to the ds18x20 device
 * @param addr    The 64-bit address of the device to read. This can be set
 *                to ::DS18X20_ANY to read any device on the bus (but note
 *                that this will only work if there is exactly one device
 *                connected, or they will corrupt each others' transmissions)
 * @param buffer  An 8-byte buffer to hold the read data.
 *
 * @returns `ESP_OK` if the command was successfully issued
 */

int ds18x20_read_scratchpad(ds18x20_t *ds18x20, uint8_t *buffer);

/**
 * @brief Write the scratchpad data for a particular ds18x20 device.
 *
 * @param pin     The GPIO pin connected to the ds18x20 device
 * @param addr    The 64-bit address of the device to write. This can be set
 *                to ::DS18X20_ANY to read any device on the bus (but note
 *                that this will only work if there is exactly one device
 *                connected, or they will corrupt each others' transmissions)
 * @param buffer  An 3-byte buffer to hold the data to write
 *
 * @returns `ESP_OK` if the command was successfully issued
 */
int ds18x20_write_scratchpad(ds18x20_t *ds18x20, uint8_t *buffer);

/**
 * @brief Issue the copy scratchpad command, copying current scratchpad to
 *        EEPROM.
 *
 * @param pin     The GPIO pin connected to the ds18x20 device
 * @param addr    The 64-bit address of the device to command. This can be set
 *                to ::DS18X20_ANY to read any device on the bus (but note
 *                that this will only work if there is exactly one device
 *                connected, or they will corrupt each others' transmissions)
 *
 * @returns `ESP_OK` if the command was successfully issued
 */
int ds18x20_copy_scratchpad(ds18x20_t *ds18x20);


#ifdef __cplusplus
} // extern "C"
#endif


#endif /* COMPONENTS_MODULENAME_INCLUDE_MODULENAME_H_ */
