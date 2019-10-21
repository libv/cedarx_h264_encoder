#ifndef _CEDAR_VE_H_
#define _CEDAR_VE_H_

enum IOCTL_CMD {
	IOCTL_GET_ENV_INFO = 0x101,
	IOCTL_ENCODE = 0x600,
	IOCTL_CONFIG = 0x601,
};

struct cedarv_env_infomation {
	unsigned int phymem_start;
	int phymem_total_size;
	unsigned int address_macc;
};

#endif
