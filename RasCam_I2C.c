

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define SLAVE_ADDR 0b00100000
#define TEST_PATT_MODE 0x0601
#define COLOR_BAR16 0x0005

void Test_Pattern() {
	int i2c_fd;
	
	//Open the I2C Bus number 2 or other number
	if ((i2c_fd = open("dev/i2c-0", O_RDWR)) < 0) {
		printf("Failed to open\n");
		exit(1);
	}
	else {
		printf("Open the bus");
	}

	//Connect to the SLAVE
	if (ioctl(i2c_fd, I2C_SLAVE_FORCE, SLAVE_ADDR) < 0) {
		printf("Failed to acquire bus access\n");
		exit(1);
	}

	char buff[5] = { 0, };
	buff[0] = TEST_PATT_MODE; //Address for Test Pattern Mode
	buff[1] = COLOR_BAR16; // For Split Color Bar
	buff[2] = COLOR_BAR16;
	int cnt = 0;
	while (1) {
		if (write(i2c_fd, buff, 3) != 3) {
			printf("Failed to write to the bus!!\n");
		}
		else {
			printf("Test Pattern : 16 Split Color Bar\n");
		}
	}
}
int main()
{	
	
	Test_Pattern();
}

