#include "I2C.h"
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <mutex>
#define SLAVE_ADDR 0

extern std::mutex all_nodes_mutex;

int init_slave(bsc_xfer_t &xfer, int addr)
{
  gpioSetMode(18, PI_ALT3);
  gpioSetMode(19, PI_ALT3);
  xfer.control = (addr << 16) | /* Slave address */
                 (0x00 << 13) | /* invert transmit status flags */
                 (0x00 << 12) | /* enable host control */
                 (0x00 << 10) | /* invert receive status flags */
                 (0x01 << 9) | /* enable receive */
                 (0x00 << 8) | /* enable transmit */
                 (0x00 << 7) | /* abort and clear FIFOs */
                 (0x00 << 6) | /* send control reg as 1st I2C byte */
                 (0x00 << 5) | /* send status regr as 1st I2C byte */
                 (0x00 << 4) | /* set SPI polarity high */
                 (0x00 << 3) | /* set SPI phase high */
                 (0x01 << 2) | /* enable I2C mode */
                 (0x00 << 1) | /* enable SPI mode */
                 0x01 ; /* enable BSC peripheral */
  return bscXfer(&xfer);
}

int close_slave(bsc_xfer_t &xfer)
{
  xfer.control = 0;
  return bscXfer(&xfer);
}


int I2C(nodes *all_nodes)
{
  int status, j, key = 0, address, pwm;
  if (gpioInitialise() < 0) {
    printf("Erro 1\n"); return 1;
  }
  bsc_xfer_t xfer;
  float lux;
  char msg_type, token, buffer[4];

  std::cout << "I2C" << std::endl;

  status = init_slave(xfer, SLAVE_ADDR);

  while (1)
  {
    //usleep(1000000);
    xfer.txCnt = 0;
    status = bscXfer(&xfer);

    if (xfer.rxCnt > 0)
    {
      //std::cout<<"while"<<std::endl;
      //std::cout<<"Received bytes"<< xfer.rxCnt<<std::endl;
      //std::cout<<"o que esta no buffer "<< xfer.rxBuf<<std::endl;
      //printf("buffer: %x\n",xfer.rxBuf);

      msg_type = (char)xfer.rxBuf[0];

      if (msg_type == 'T')
      {
        address = (int)xfer.rxBuf[1];

        pwm = (int)xfer.rxBuf[2];

        for (j = 0; j < 4; j++)
          buffer[j] = xfer.rxBuf[3 + j];

        lux = *((float*)buffer);

        if (xfer.rxBuf[7] == 'N')
        {
          all_nodes_mutex.lock();
          (*all_nodes).node_i[address - 4].insertDATA(lux, pwm);
          all_nodes_mutex.unlock();
        }

      }

      else if (msg_type == 'S')
      {
        address = (int)xfer.rxBuf[1];

        token = xfer.rxBuf[2];

        for (j = 0; j < 4; j++)
            buffer[j] = xfer.rxBuf[3 + j];

        lux = *((float*)buffer);

        if (xfer.rxBuf[7] == 'N')
        {
          all_nodes_mutex.lock();
          (*all_nodes).node_i[address - 4].change_occup(token);
          (*all_nodes).node_i[address - 4].change_ref(lux);
          all_nodes_mutex.unlock();
        }

      }


    }
  }
  status = close_slave(xfer);
  gpioTerminate();
}
