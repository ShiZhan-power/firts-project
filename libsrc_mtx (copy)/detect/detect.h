#ifndef __DETECT_H__
#define __DETECT_H__

enum DP_ErrNum_e
{
	DP_SUCCESS = 0,
	DP_ERR_ALREADLY_INIT,
	DP_ERR_NOT_INIT,
	DP_ERR_PTHREAD_CREATE,
	DP_ERR_OUTOFMEM,
	DP_ERR_FOEPN,
	DP_ERR_FREAD,
	DP_ERR_RKNN_INIT,
	DP_ERR_MAX
};

typedef struct _SSDRECT
{
    int left;
    int top;
    int right;
    int bottom;
} SSDRECT;

struct ssd_object
{
  char name[20]; // 确保有足够的空间存放类别名称
  int id; // 用于存放追踪ID
  SSDRECT select;
};

struct ssd_group
{
    int count;
    struct ssd_object objects[100];
};

typedef void (*detect_callback_t)(int fd, int w, int h);

struct camera_buffer;

typedef struct DetectProcess_s
{
	// 修改：增加参数以接收配置
    int (*init)(int enable_detect, int src_w, int src_h);
	int (*uninit)(void);
	int (*start)(void);
	int (*stop)(void);
    void (*camera_handle)(struct camera_buffer* cam_buf, int w, int h, int ch);
	void (*SetCallback)(detect_callback_t callback);
}DetectProcess_t;

#ifdef __cplusplus
extern "C"{
#endif
	DetectProcess_t* GetDetectProcessInstance(void);
#ifdef __cplusplus
}
#endif

#endif
