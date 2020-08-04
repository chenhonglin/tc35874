#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/poll.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
 
#include <linux/i2c.h>
#include <linux/i2c-dev.h>


int main(){
	
	int fd;
	fd=open("/dev/i2c-6",O_RDWR);
	
	if(fd<0){
		printf("open fail\n");
	}
	/*write 함수*/
     struct i2c_rdwr_ioctl_data msg_rdwr;
     struct i2c_msg i2cmsg;
     char my_buf[128];

     my_buf[0] = 0x0002;
     my_buf[1] = 0x0001;

     for(i= 0; i < len; i++)
     {
        my_buf[2+i] = buf[i];
     }
     msg_rdwr.msgs = &i2cmsg;
     msg_rdwr.nmsgs = 1;
     i2cmsg.addr  = I2C_ADDR;
     i2cmsg.flags = 0;
     i2cmsg.len   = 2+len;
     i2cmsg.buf   = my_buf;

    if(ioctl(fd,I2C_RDWR,&msg_rdwr)<0)
    {
          printf("write: %s\n", strerror(errno));
          return -1;
    }

/*read함수*/
    struct i2c_rdwr_ioctl_data msg_rdwr;
    struct i2c_msg             i2cmsg;
    
    //위 write함수 후 (addr전송) 아래 read처리로 값 read

    msg_rdwr.msgs = &i2cmsg;
    msg_rdwr.nmsgs = 1;

    i2cmsg.addr  = I2C_ADDR;
    i2cmsg.flags = I2C_M_RD;
    i2cmsg.len   = len;
    i2cmsg.buf   = buf;

    if(ioctl(fd,I2C_RDWR,&msg_rdwr)<0)
    {
        printf("read: %s\n", strerror(errno));
        return -1;
    }
	return 0;
}
