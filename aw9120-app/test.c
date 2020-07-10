#include "stdio.h"

int main(){
	
	unsigned int data = 0xffff;
	printf("-----read data: [%x]\r\n",data);
	data &= ~(1 << 4);
	printf("-----change data: [%x]\r\n",data);
	return 0;
}
