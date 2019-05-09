#ifndef SERVER_I2C_H
#define SERVER_I2C_H
#include "classes.h"
#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int init_slave(bsc_xfer_t *, int );

int close_slave(bsc_xfer_t *);

int I2C(nodes*);

#endif //SERVER_I2C_H
