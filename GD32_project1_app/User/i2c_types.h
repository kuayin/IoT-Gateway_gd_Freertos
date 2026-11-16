/* i2c_types.h */
#ifndef I2C_TYPES_H
#define I2C_TYPES_H

typedef enum {
    I2C_OK = 0,
    I2C_ERR_TIMEOUT,
    I2C_ERR_NACK,
    I2C_ERR_BUS_BUSY,
    I2C_ERR_MUTEX,
    I2C_ERR_RECOVER_FAIL,
    I2C_ERR_PARAM,
} i2c_status_t;

#endif