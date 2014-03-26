
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "I2CDriver.h"

/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	1
#define PX4_I2C_BUS_LED		2

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_LED	0x55
#define PX4_I2C_OBDEV_HMC5883	0x1e

#define SUB_ADDR_SETTINGS	0x84
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */

/* Pulsed Light */
// i2c address
#define AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR   0x42

// registers
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MEASURE_REG           0x00
#define AP_RANGEFINDER_PULSEDLIGHTLRF_DISTHIGH_REG          0x0f // high byte of distance measurement
#define AP_RANGEFINDER_PULSEDLIGHTLRF_DISTLOW_REG           0x10 // low byte of distance measurement

// Measurement
#define AP_RANGEFINDER_PULSEDLIGHTLRF_MSRREG_ACQUIRE        0x61

using namespace PX4;

extern const AP_HAL::HAL& hal;

PX4I2CDriver::PX4I2CDriver(AP_HAL::Semaphore* semaphore) : 
    _semaphore(semaphore),
    _bus(PX4_I2C_BUS_EXPANSION),
    _frequency(100000),
    _retries(3)
{
}
void PX4I2CDriver::begin() {
	/* attach to the i2c bus */
	_dev = up_i2cinitialize(_bus);    
}

void PX4I2CDriver::end() {
    // I'm just using this for testing for now
    uint8_t buff[2];
    int16_t ret_value = 0;
    
    if (hal.i2c->writeRegister(AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR, AP_RANGEFINDER_PULSEDLIGHTLRF_MEASURE_REG, AP_RANGEFINDER_PULSEDLIGHTLRF_MSRREG_ACQUIRE) == 0) {
        hal.console->print_P(PSTR("\n PX4I2CDriver pulsedlight write successful \n"));
    }
    else {
        hal.console->print_P(PSTR("\n PX4I2CDriver pulsedlight write failed \n"));
    }
    hal.scheduler->delay(5);
    
    if (hal.i2c->readRegisters(AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR, AP_RANGEFINDER_PULSEDLIGHTLRF_DISTHIGH_REG, 1, &buff[0]) == 0) {
        hal.scheduler->delay(5);
        // read the low byte
        if (hal.i2c->readRegisters(AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR, AP_RANGEFINDER_PULSEDLIGHTLRF_DISTLOW_REG, 1, &buff[1]) == 0) {            
            // combine results into distance
            ret_value = buff[0] << 8 | buff[1];
            hal.console->printf_P(PSTR("distance: %d high: %x low: %x\n"),(int)ret_value,(int)buff[0], (int)buff[1]);
        }
    }
}
void PX4I2CDriver::setTimeout(uint16_t ms) {}
void PX4I2CDriver::setHighSpeed(bool active) {}

uint8_t PX4I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data) {
    struct i2c_msg_s msgv[1];
    
    msgv[0].addr = addr;
	msgv[0].flags = 0; // Write
	msgv[0].buffer = data;
	msgv[0].length = len;
    
    if (_transfer(msgv, 1) < 0)
    {
        return 1;
    }
    
    return 0;
} 

uint8_t PX4I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
    return writeRegisters(addr, reg, 1, &val);
}

uint8_t PX4I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data) {
    uint8_t buffer[100];
    buffer[0] = reg;
    memcpy(buffer+1, data, len);

    struct i2c_msg_s msgv[1];    
    msgv[0].addr = addr;
	msgv[0].flags = 0; // Write
	msgv[0].buffer = buffer;
	msgv[0].length = len + 1;
    
    if (_transfer(msgv, 1) < 0)
    {
        return 1;
    }
    
    return 0;
}

uint8_t PX4I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data) {
    struct i2c_msg_s msgv[1];
    
    msgv[0].addr = addr;
    msgv[0].flags = I2C_M_READ;
    msgv[0].buffer = data;
    msgv[0].length = len;
    
    if (_transfer(msgv, 1) < 0)
    {
        return 1;
    }
    
    return 0;
}
uint8_t PX4I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data) {
    return readRegisters(addr, reg, 1, data);
}

uint8_t PX4I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data) {
    // We conduct a write of the register number we want followed by a read
    data[0] = reg; // Temporarily steal this for the write
    struct i2c_msg_s msgv[2];
    msgv[0].addr = addr;
	msgv[0].flags = 0; // Write
	msgv[0].buffer = data;
	msgv[0].length = 1;
    
    // Second transaction is a read
    msgv[1].addr = addr;
    msgv[1].flags = I2C_M_READ;
    msgv[1].buffer = data;
    msgv[1].length = len;
    
    if (_transfer(msgv, 2) < 0)
    {
        return 1;
    }
    
    return 0;
}

uint8_t PX4I2CDriver::lockup_count() {return 0;}

int PX4I2CDriver::_transfer(i2c_msg_s *msgv, unsigned msgs)
{
	int ret = 0;
	unsigned retry_count = 0;

	do {
		I2C_SETFREQUENCY(_dev, _frequency);
		ret = I2C_TRANSFER(_dev, msgv, msgs);

        hal.console->printf_P(PSTR("\n ret evaluates to %d \n"), ret);
        
		/* success */
		if (ret >= OK)
			break;

		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries))
			up_i2creset(_dev);

	} while (retry_count++ < _retries);

	return ret;
}

#endif // CONFIG_HAL_BOARD
