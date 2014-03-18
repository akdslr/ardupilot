
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "I2CDriver.h"

//PX4_I2C_BUS_EXPANSION

using namespace PX4;

PX4I2CDriver::PX4I2CDriver(AP_HAL::Semaphore* semaphore) : 
    _semaphore(semaphore)
{
}
void PX4I2CDriver::begin() {}
void PX4I2CDriver::end() {}
void PX4I2CDriver::setTimeout(uint16_t ms) {}
void PX4I2CDriver::setHighSpeed(bool active) {}

uint8_t PX4I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{return 0;} 
uint8_t PX4I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{return 0;}
uint8_t PX4I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 0;}

uint8_t PX4I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    memset(data, 0, len);
    return 0;
}
uint8_t PX4I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    *data = 0;
    return 0;
}

uint8_t PX4I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
    memset(data, 0, len);    
    return 0;
}

uint8_t PX4I2CDriver::lockup_count() {return 0;}
#endif // CONFIG_HAL_BOARD