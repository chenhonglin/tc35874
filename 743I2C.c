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
	unsigned char buf[32];
	int wr;
	fd=open("/dev/i2c-6", O_RDWR);
	
	if(fd<0){
		printf("open fail\n");
		exit(1);
	}

	ioctl(fd,I2C_SLAVE, 0x0f);
	
	buf[0]=0x0002;
	buf[1]=0x0001;

	wr=write(fd,buf,2);
	if(wr<0){
		printf("write fail\n");
		exit(1);
	}

	return 0;
}
