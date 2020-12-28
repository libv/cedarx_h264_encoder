#ifndef _CEDAR_IOCTL_H_
#define _CEDAR_IOCTL_H_

#define CEDAR_DEVICE "cedar_dev"
#define CEDAR_DEVICE_PATH "/dev/" CEDAR_DEVICE

enum cedar_ioctl_cmd {
	CEDAR_IOCTL_ENCODE = 0x600,
	CEDAR_IOCTL_CONFIG = 0x601,
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

	uint32_t input_luma_dma_addr;
	int input_luma_size;
	uint32_t input_chroma_dma_addr;
	int input_chroma_size;

	uint32_t bytestream_dma_addr;
	int bytestream_size;
};

#endif /* _CEDAR_IOCTL_H_ */
