#ifndef _V4L2CAMERA_H_
#define _V4L2CAMERA_H_

#include <unistd.h>

enum VC_ErrNum_e
{
	VC_SUCCESS = 0,
	VC_ERR_ALREADLY_INIT,
	VC_ERR_NOT_INIT,
	VC_ERR_PTHREAD_CREATE,
	VC_ERR_OUTOFMEM,
	VC_ERR_SELECT,
	VC_ERR_V4L2_OPEN,
	VC_ERR_V4L2_CLOSE,
	VC_ERR_V4L2_IOCTL,
	VC_ERR_V4L2_NOCAPDEV,
	VC_ERR_V4L2_NOSTREAMING,
	VC_ERR_V4L2_INSUFFCIENT,
	VC_ERR_V4L2_MMAP,
	VC_ERR_V4L2_MUNMMAP,
	VC_ERR_MAX
};

// **修改**: 重命名结构体以避免冲突
struct camera_plane_info {
    void* start;
    size_t length;
	int fd;
};

struct camera_buffer {
    struct camera_plane_info* planes;
    unsigned int n_planes;
};

// **修改**: 回调函数使用新的结构体类型
typedef void (*v4l2camera_callback_t)(struct camera_buffer* buffer, int width, int height, int ch);

typedef struct V4l2Camera_s
{
	int (*init)(const char* dev_name, unsigned int width, unsigned int height, unsigned int fps, unsigned int format);
	int (*uninit)(void);
	void (*SetCallback)(v4l2camera_callback_t callback);
	int (*start)(void);
	int (*restart)(void);
	int (*stop)(void);
}V4l2Camera_t;

V4l2Camera_t* GetV4l2CameraInstance(void);
#endif