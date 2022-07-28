
/******************************************************************************
        Interfaces
 ******************************************************************************/
#include "stdbool.h"
#include "string.h"
#include "stdint.h"

#include "ds18b20.h"
/******************************************************************************
        Defines and constants
 ******************************************************************************/
static const char * MODULE_NAME 	= "[DS18B20]";
static const char * DRIVER_VERSION 	= "ds18b20_v1.0.0";

#define ds18x20_WRITE_SCRATCHPAD 0x4E
#define ds18x20_READ_SCRATCHPAD  0xBE
#define ds18x20_COPY_SCRATCHPAD  0x48
#define ds18x20_READ_EEPROM      0xB8
#define ds18x20_READ_PWRSUPPLY   0xB4
#define ds18x20_SEARCHROM        0xF0
#define ds18x20_SKIP_ROM         0xCC
#define ds18x20_READROM          0x33
#define ds18x20_MATCHROM         0x55
#define ds18x20_ALARMSEARCH      0xEC
#define ds18x20_CONVERT_T        0x44

/******************************************************************************
        Data types
 ******************************************************************************/

/******************************************************************************
        Local function prototypes
 ******************************************************************************/

/******************************************************************************
        Local variables
 ******************************************************************************/

/******************************************************************************
        Public function definitions
 ******************************************************************************/
int ds18x20_scan_devices(ds18x20_t *ds18x20, uint64_t *addr_list ,uint8_t addr_count, uint8_t *found)
{
	int ret = DS18x20_OK;
	uint8_t count = 0;
    *found = 0;

    onewire_search_start(&ds18x20->one_wire);

    while (*found == 0)
    {
    	/* search devices */
    	(void)onewire_search_next(&ds18x20->one_wire);

    	ds18x20->address = ds18x20->one_wire.onewire_addr;

        uint8_t family_id = (uint8_t)ds18x20->address;
        if (family_id == DS18B20_FAMILY_ID || family_id == DS18S20_FAMILY_ID)
        {
            if (*found < addr_count)
                addr_list[*found] = ds18x20->address;
            *found += 1;
        }

        if(count++ >= 5)
        	ret = DS18x20_ERR;
    }

    return ret;
}

int ds18x20_measure(ds18x20_t *ds18x20, bool wait)
{
	int ret = DS18x20_OK;

	if (!onewire_reset(&ds18x20->one_wire))
        return DS18x20_ERR;

    if (ds18x20->address == DS18X20_ANY){
    	ret = onewire_skip_rom(&ds18x20->one_wire);
    }

    else{
    	ret = onewire_select(&ds18x20->one_wire);
    }

    onewire_write(&ds18x20->one_wire, ds18x20_CONVERT_T);
    // For parasitic devices, power must be applied within 10us after issuing
    // the convert command.
    onewire_power(&ds18x20->one_wire);

    if (wait)
    {
    	ds18x20->delay_ms(750);
        onewire_depower(&ds18x20->one_wire);
    }

    return DS18x20_OK;
}

int ds18x20_read_temperature(ds18x20_t *ds18x20)
{
    if ((uint8_t)ds18x20->address == DS18B20_FAMILY_ID) {
    	return ds18b20_read_temperature(ds18x20);
    }
    else
    {
    	return ds18s20_read_temperature(ds18x20);
    }
}

int ds18b20_read_temperature(ds18x20_t *ds18x20)
{
    uint8_t scratchpad[8];
    int16_t temp;

    if(ds18x20_read_scratchpad(ds18x20, scratchpad) != DS18x20_OK )
    	return DS18x20_ERR;

    temp = scratchpad[1] << 8 | scratchpad[0];
    ds18x20->temp = ((float)temp * 625.0) / 10000;

    return DS18x20_OK;
}

int ds18s20_read_temperature(ds18x20_t *ds18x20)
{
    uint8_t scratchpad[8];
    int16_t temp;

    if(ds18x20_read_scratchpad(ds18x20, scratchpad) != DS18x20_OK )
    	return DS18x20_ERR;

    temp = scratchpad[1] << 8 | scratchpad[0];

    temp = ((temp & 0xfffe) << 3) + (16 - scratchpad[6]) - 4;
    ds18x20->temp = ((float)temp * 625.0) / 10000;

    return DS18x20_OK;
}

int ds18x20_read_temp_multi(ds18x20_t *ds18x20, uint64_t *addr_list, uint8_t addr_count, float *result_list)
{
    int ret = DS18x20_OK;

    for (uint8_t i = 0; i < addr_count; i++)
    {
    	ds18x20->address = addr_list[i];
    	int tmp = ds18x20_read_temperature(ds18x20);
    	result_list[i] = ds18x20->temp;
    	if (tmp != DS18x20_OK)
        	ret = tmp;
    }
    return ret;
}

int ds18s20_measure_and_read(ds18x20_t *ds18x20)
{
    if(ds18x20_measure(ds18x20, true) != DS18x20_OK)
    	return DS18x20_ERR;

    return ds18s20_read_temperature(ds18x20);
}

int ds18b20_measure_and_read(ds18x20_t *ds18x20)
{
    if(ds18x20_measure(ds18x20, true) != DS18x20_OK)
    	return DS18x20_ERR;

    return ds18b20_read_temperature(ds18x20);
}

int ds18x20_measure_and_read(ds18x20_t *ds18x20)
{
    if(ds18x20_measure(ds18x20, true) != DS18x20_OK)
    	return DS18x20_ERR;

    return ds18x20_read_temperature(ds18x20);
}

int ds18x20_measure_and_read_multi(ds18x20_t *ds18x20, uint64_t *addr_list, uint8_t addr_count, float *result_list)
{
	ds18x20->address = DS18X20_ANY;
	if(ds18x20_measure(ds18x20, true) != DS18x20_OK)
		return DS18x20_ERR;

    return ds18x20_read_temp_multi(ds18x20, addr_list, addr_count, result_list);
}

int ds18x20_read_scratchpad(ds18x20_t *ds18x20, uint8_t *buffer)
{

    uint8_t crc;
    uint8_t expected_crc;

    if (!onewire_reset(&ds18x20->one_wire))
        return DS18x20_ERR;

    if (ds18x20->address == DS18X20_ANY)
        onewire_skip_rom(&ds18x20->one_wire);
    else
        onewire_select(&ds18x20->one_wire);

    onewire_write(&ds18x20->one_wire, ds18x20_READ_SCRATCHPAD);

    for (int i = 0; i < 8; i++)
        buffer[i] = onewire_read(&ds18x20->one_wire);

    crc = onewire_read(&ds18x20->one_wire);

    expected_crc = onewire_crc8(buffer, 8);

    if (crc != expected_crc)
    {
        /*ESP_LOGE(MODULE_NAME, "CRC check failed reading scratchpad: %02x %02x %02x %02x %02x %02x %02x %02x : %02x (expected %02x)", buffer[0], buffer[1],
                buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], crc, expected_crc);*/
        return DS18x20_ERR;
    }

    return DS18x20_OK;
}

int ds18x20_write_scratchpad(ds18x20_t *ds18x20, uint8_t *buffer)
{
    if (!onewire_reset(&ds18x20->one_wire))
        return DS18x20_ERR;

    if (ds18x20->address == DS18X20_ANY)
        onewire_skip_rom(&ds18x20->one_wire);
    else
        onewire_select(&ds18x20->one_wire);
    onewire_write(&ds18x20->one_wire, ds18x20_WRITE_SCRATCHPAD);

    for (int i = 0; i < 3; i++)
        onewire_write(&ds18x20->one_wire, buffer[i]);

    return DS18x20_OK;
}

int ds18x20_copy_scratchpad(ds18x20_t *ds18x20)
{
    if (!onewire_reset(&ds18x20->one_wire))
        return DS18x20_ERR;

    if (ds18x20->address == DS18X20_ANY)
        onewire_skip_rom(&ds18x20->one_wire);
    else
        onewire_select(&ds18x20->one_wire);

    onewire_write(&ds18x20->one_wire, ds18x20_COPY_SCRATCHPAD);
    // For parasitic devices, power must be applied within 10us after issuing
    // the convert command.
    onewire_power(&ds18x20->one_wire);

    // And then it needs to keep that power up for 10ms.
    ds18x20->delay_ms(10);
    onewire_depower(&ds18x20->one_wire);

    return DS18x20_OK;
}
/******************************************************************************
        Private function definitions
 ******************************************************************************/

/* End of file ***************************************************************/

