#include <stdint.h>
#include <stdbool.h>
#include "glue.h"

extern uint32_t _estack;
extern uint32_t _srelocate, _erelocate, _etext;
extern uint32_t _sbss, _ebss;

extern uint32_t Image$$STACK$$Base;
extern uint32_t Image$$STACK$$Limit;

int main(void);

int htoi(char x) {
	unsigned int v=x;
	v-='0';
	if (v<10)	return v;
	v|=0x20;	//A->a
	v-='a'-'0';
	if (v<6)	return v+10;
	return -1;
}

__attribute__ ((section (".vectors"),used))
const struct {
	uint32_t* SP;
	void*	  Entry;
#if defined ( __CC_ARM )
}vectors = {&Image$$STACK$$Limit, main};
#else
}vectors = {&_estack, main};
#endif
