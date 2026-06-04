#ifndef _ENCODE_H_
#define _ENCODE_H_

#include "rk_mpi.h" // 包含 MppPacket 的定义

enum EP_ErrNum_e
{
	EP_SUCCESS = 0,
	EP_ERR_ALREADLY_INIT,
	EP_ERR_NOT_INIT,
	EP_ERR_PTHREAD_CREATE,
	EP_ERR_OUTOFMEM,
	EP_ERR_SELECT,
	EP_ERR_MAX
};

typedef void (*encode_callback_t)(MppPacket packet);

typedef struct EncodeProcess_s
{
	int (*init)(int width, int height, int format, int type, int fps, int gop);
	int (*uninit)(void);
	void (*SetCallback)(encode_callback_t callback);
	int (*start)(void);
	int (*stop)(void);
	// **修改**: camera_handle 现在接收 dma-buf 文件描述符
	void (*camera_handle)(int dma_fd, int w, int h);
    int (*get_header_data)(void* buf, size_t* size);
	int (*set_output_file)(const char* path);
}EncodeProcess_t;

EncodeProcess_t* GetEncodeProcessInstance(void);
#endif
