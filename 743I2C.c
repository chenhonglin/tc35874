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
	int err;
	u8 *data;
	data[0]=0x0002;
	data[1]=0x0001;
	struct i2c_msg i2c_msgs={
		.addr = 0x0f,
		.flags = 0,
		.len = 2,
		.buf = data,
	};
	
	fd=open("/dev/i2c-6", O_RDWR);
	
	if(fd<0){
		printf("open fail\n");
		exit(1);
	}
	
	err=ioctl(fd,I2C_RDWR,(struct i2c_rdwr_ioctl_data *) msgs);
	
	if(err<0){
		printf("ioctl fail\n");
	}
	

	

	return 0;
}
