#ifndef _CEDAR_IOCTL_H_
#define _CEDAR_IOCTL_H_

enum cedar_ioctl_cmd {
	CEDAR_IOCTL_GET_ENV_INFO = 0x101,
	CEDAR_IOCTL_ENCODE = 0x600,
	CEDAR_IOCTL_CONFIG = 0x601,
};

struct cedarv_env_infomation {
	unsigned int phymem_start;
	int phymem_total_size;
	unsigned int address_macc;
};

#endif /* _CEDAR_IOCTL_H_ */
