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

struct cedar_ioctl_config {
	int src_width;
	int src_height;
#define CEDAR_IOCTL_CONFIG_FORMAT_NV12 0
#define CEDAR_IOCTL_CONFIG_FORMAT_NV16 1
	int src_format;

	int dst_width;
	int dst_height;

	int profile;
	int level;
	int qp;
	int keyframe_interval;

#define CEDAR_IOCTL_ENTROPY_CODING_CAVLC 0
#define CEDAR_IOCTL_ENTROPY_CODING_CABAC 1
	int entropy_coding_mode;
};

#endif /* _CEDAR_IOCTL_H_ */
