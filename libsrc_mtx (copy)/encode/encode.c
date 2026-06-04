#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include "rk_mpi.h"

#include "mpp_env.h"
#include "mpp_mem.h"
#include "mpp_time.h"
#include "mpp_common.h"
#include "mpp_debug.h"
#include "mpp_buffer.h"

#include "encode.h"

static struct encodePara_s
{
	int is_init;
    MppCtx ctx;
    MppApi *mpi;
	MppEncCfg cfg;
	MppBufferGroup buf_grp;
    MppBuffer pkt_buf;
	MppEncHeaderMode header_mode;
    MppPacket sps_pps_packet;
    RK_U32 width;
    RK_U32 height;
    RK_U32 hor_stride;
    RK_U32 ver_stride;
    MppFrameFormat fmt;
    MppCodingType type;
	MppEncRcMode rc_mode;
    size_t frame_size;
    RK_U64 frame_count;
    RK_S32 gop;
    RK_S32 fps;
    RK_S32 bps;
	pthread_t encode_pid;
	int encode_pid_on;
	encode_callback_t callback;
	int fflag;
	FILE *fp_output;
}g_para =
{
	.is_init = 0,
	.ctx = NULL,
    .sps_pps_packet = NULL,
	.gop = 0,
	.fps = 0,
	.bps = 0,
    .frame_count = 0,
	.encode_pid_on = 0,
	.callback = NULL,
	.fflag = 0,
	.fp_output = NULL,
};

static MPP_RET mpp_enc_cfg_setup(void)
{
    MPP_RET ret;
    MppApi *mpi = g_para.mpi;
    MppCtx ctx = g_para.ctx;
	MppEncCfg cfg = g_para.cfg;

	mpp_enc_cfg_set_s32(cfg, "prep:width", g_para.width);
	mpp_enc_cfg_set_s32(cfg, "prep:height", g_para.height);
	mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", g_para.hor_stride);
	mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", g_para.ver_stride);
	mpp_enc_cfg_set_s32(cfg, "prep:format", g_para.fmt);

	g_para.rc_mode = MPP_ENC_RC_MODE_CBR;//MPP_ENC_RC_MODE_AVBR;
	mpp_enc_cfg_set_s32(cfg, "rc:mode", g_para.rc_mode);

	mpp_enc_cfg_set_s32(cfg, "rc:fps_in_flex", 0);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", g_para.fps);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", 1);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_out_flex", 0);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", g_para.fps);
	mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm", 1);
	mpp_enc_cfg_set_s32(cfg, "rc:gop", g_para.gop);

	mpp_enc_cfg_set_u32(cfg, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
	mpp_enc_cfg_set_u32(cfg, "rc:drop_thd", 20);
	mpp_enc_cfg_set_u32(cfg, "rc:drop_gap", 1);

    mpp_enc_cfg_set_s32(cfg, "rc:bps_target", g_para.bps);
	switch (g_para.rc_mode)
	{
		case MPP_ENC_RC_MODE_AVBR :
			mpp_enc_cfg_set_s32(cfg, "rc:bps_max", g_para.bps * 3 / 2);
			mpp_enc_cfg_set_s32(cfg, "rc:bps_min", g_para.bps * 1 / 2);
			break;
		default :
			mpp_enc_cfg_set_s32(cfg, "rc:bps_max", g_para.bps * 17 / 16);
			mpp_enc_cfg_set_s32(cfg, "rc:bps_min", g_para.bps * 15 / 16);
			break;
	}

	switch (g_para.type)
	{
		case MPP_VIDEO_CodingAVC :
			switch (g_para.rc_mode)
			{
				case MPP_ENC_RC_MODE_AVBR :
					mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 26);
					mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 51);
					mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 22);
					mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 51);
					mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 22);
					mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);
                    mpp_enc_cfg_set_s32(cfg, "rc:qp_step", 4);
					break;
				default :
					break;
			}
			break;
		default :
			break;
	}

    mpp_enc_cfg_set_s32(cfg, "codec:type", g_para.type);
    switch (g_para.type)
	{
		case MPP_VIDEO_CodingAVC :
            if (g_para.width > 1920) {
			    mpp_enc_cfg_set_s32(cfg, "h264:level", 51);
            } else {
			    mpp_enc_cfg_set_s32(cfg, "h264:level", 41);
            }
            mpp_enc_cfg_set_s32(cfg, "h264:profile", 66);//100
			mpp_enc_cfg_set_s32(cfg, "h264:cabac_en", 1);
			break;
		default :
			break;
	}

	ret = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
    if (ret)
	{
        printf("mpi control enc set cfg failed ret %d\n", ret);
        return ret;
    }

    if (g_para.type == MPP_VIDEO_CodingAVC || g_para.type == MPP_VIDEO_CodingHEVC)
	{
        g_para.header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
        ret = mpi->control(ctx, MPP_ENC_SET_HEADER_MODE, &g_para.header_mode);
        if (ret)
		{
            printf("mpi control enc set header mode failed ret %d\n", ret);
            return ret;
        }
    }
    return ret;
}

static int EncodeProcessSetOutputFile(const char* path)
{
    if (!g_para.is_init) {
        printf("ERROR: set_output_file must be called after init.\n");
        return -1;
    }

    if (g_para.fp_output) {
        fclose(g_para.fp_output);
        g_para.fp_output = NULL;
    }

    if (!path) {
        return 0;
    }

    g_para.fp_output = fopen(path, "wb");
    if (!g_para.fp_output) {
        printf("ERROR: Failed to open output file %s\n", path);
        return -1;
    }

    return 0;
}

static void EncodeProcessCameraHandle(int dma_fd, int w, int h)
{
    MPP_RET ret = MPP_OK;
    MppCtx ctx = g_para.ctx;
    MppApi *mpi = g_para.mpi;
	MppFrame frame = NULL;
    MppBuffer imported_buf = NULL;

	if(g_para.is_init == 0)
		return;

    MppBufferInfo info;
    memset(&info, 0, sizeof(MppBufferInfo));
    info.type = MPP_BUFFER_TYPE_DRM;
    info.fd = dma_fd;
    info.size = g_para.frame_size;
    info.index = dma_fd;
    ret = mpp_buffer_import(&imported_buf, &info);
    if (ret) {
        printf("mpp_buffer_import failed, fd=%d, ret=%d\n", dma_fd, ret);
        return;
    }

    if(g_para.fp_output && !g_para.fflag) {
		mpp_packet_init_with_buffer(&g_para.sps_pps_packet, imported_buf);
		mpp_packet_set_length(g_para.sps_pps_packet, 0);

        ret = mpi->control(ctx, MPP_ENC_GET_HDR_SYNC, g_para.sps_pps_packet);
        if (ret) {
            printf("mpi control enc get extra info failed\n");
        } else {
            void *ptr   = mpp_packet_get_pos(g_para.sps_pps_packet);
            size_t len  = mpp_packet_get_length(g_para.sps_pps_packet);
            if (g_para.fp_output)
                fwrite(ptr, 1, len, g_para.fp_output);
            g_para.fflag = 1;
        }
	}

	ret = mpp_frame_init(&frame);
	if (ret)
	{
		printf("mpp_frame_init failed\n");
        mpp_buffer_put(imported_buf);
		return;
	}

	mpp_frame_set_width(frame, g_para.width);
	mpp_frame_set_height(frame,g_para.height);
	mpp_frame_set_hor_stride(frame, g_para.hor_stride);
	mpp_frame_set_ver_stride(frame, g_para.ver_stride);
	mpp_frame_set_fmt(frame, g_para.fmt);
	mpp_frame_set_buffer(frame, imported_buf);
    mpp_frame_set_pts(frame, g_para.frame_count * (1000000 / g_para.fps));
    g_para.frame_count++;

	ret = mpi->encode_put_frame(ctx, frame);
	if (ret)
		printf("mpp encode put frame failed\n");

	mpp_frame_deinit(&frame);
    mpp_buffer_put(imported_buf);
}

void* EncodeLoopProcess(void* args)
{
    MPP_RET ret;
    MppApi *mpi = g_para.mpi;
    MppCtx ctx = g_para.ctx;
	MppPacket packet = NULL;

	while(g_para.encode_pid_on)
	{
        ret = mpi->encode_get_packet(ctx, &packet);
        if (ret)
		{
            if (ret == MPP_ERR_TIMEOUT) {
                continue;
            }
            printf("mpp encode get packet failed, ret=%d\n", ret);
			usleep(5000);
            continue;
        }

        if (packet)
		{
			if (g_para.fp_output) {
                void* ptr = mpp_packet_get_pos(packet);
                size_t len = mpp_packet_get_length(packet);
                if (ptr && len > 0) {
                    fwrite(ptr, 1, len, g_para.fp_output);
                }
            }
			if(g_para.callback)
				g_para.callback(packet);
			mpp_packet_deinit(&packet);
		}
		else
        {
			usleep(1000);
        }
	}
	printf("encode pthread exit\n");
	pthread_exit(0);
}

static int EncodeProcessStart(void)
{
	int ret = EP_ERR_MAX;
	if(!g_para.is_init) return EP_ERR_NOT_INIT;
	g_para.encode_pid_on = 1;
	ret = pthread_create(&g_para.encode_pid, NULL, EncodeLoopProcess, NULL);
	if(ret)
	{
		printf("pthread_create failed(%d)\n", ret);
		g_para.encode_pid_on = 0;
		return EP_ERR_PTHREAD_CREATE;
	}
	return EP_SUCCESS;
}

static int EncodeProcessStop(void)
{
	if(g_para.encode_pid_on)
	{
		g_para.encode_pid_on = 0;
		pthread_join(g_para.encode_pid, NULL);
	}
	return EP_SUCCESS;
}

static int EncodeProcessUninit(void)
{
    MPP_RET ret = MPP_OK;
	EncodeProcessStop();
	if(g_para.is_init)
	{
		g_para.is_init = 0;
		ret = g_para.mpi->reset(g_para.ctx);
		if (ret) printf("mpi->reset failed\n");
		if (g_para.ctx) { mpp_destroy(g_para.ctx); g_para.ctx = NULL; }
		if(g_para.cfg) { mpp_enc_cfg_deinit(g_para.cfg); g_para.cfg = NULL; }
		if (g_para.pkt_buf) { mpp_buffer_put(g_para.pkt_buf); g_para.pkt_buf = NULL; }
		if (g_para.buf_grp) { mpp_buffer_group_put(g_para.buf_grp); g_para.buf_grp = NULL; }
        if (g_para.sps_pps_packet) { mpp_packet_deinit(&g_para.sps_pps_packet); g_para.sps_pps_packet = NULL; }
        if (g_para.fp_output) { fclose(g_para.fp_output); g_para.fp_output = NULL; }
	}
	return EP_SUCCESS;
}

static int EncodeProcessGetHeaderData(void* buf, size_t* size)
{
	if (!g_para.sps_pps_packet) {
        return -1;
    }

    void *ptr = mpp_packet_get_pos(g_para.sps_pps_packet);
    size_t len = mpp_packet_get_length(g_para.sps_pps_packet);

    if (len > 0 && ptr) {
        if (len <= *size) {
            memcpy(buf, ptr, len);
            *size = len;
            return 0;
        } else {
            printf("EncodeGetHeader: Buffer too small (needed %zu, got %zu)\n", len, *size);
            *size = len;
            return -1;
        }
    }
    return -1;
}

static int EncodeProcessInit(int width, int height, int format, int type, int fps, int gop)
{
    MPP_RET ret = MPP_OK;
	RK_S32 timeout = 100;

	if(g_para.is_init) return EP_ERR_ALREADLY_INIT;

	g_para.width = width;
	g_para.height = height;
	g_para.fmt = format;
	g_para.type = type;

	g_para.ver_stride = MPP_ALIGN(g_para.height, 16);

    if (g_para.fmt == MPP_FMT_RGBA8888 || g_para.fmt == MPP_FMT_BGRA8888) {
        // 对于 RGBA 这样的打包格式, 水平步长是以字节为单位 (宽度 * 每像素字节数), 然后对齐。
        g_para.hor_stride = MPP_ALIGN(g_para.width * 4, 16);
        // 总大小 = 水平步长(字节) * 垂直对齐高度
        g_para.frame_size = g_para.hor_stride * g_para.ver_stride;
    } else {
        // 对于 YUV 等平面或半平面格式, 水平步长是Y分量的像素宽度对齐。
        g_para.hor_stride = MPP_ALIGN(g_para.width, 64);
        // 总大小包含所有分量。
        g_para.frame_size = g_para.hor_stride * g_para.ver_stride * 3 / 2;
    }
	
    g_para.fps = fps;
    g_para.gop = gop > 0 ? gop : fps * 2;

    if (width > 1920) {
        g_para.bps = 12 * 1024 * 1024;
    } else {
        g_para.bps = 4 * 1024 * 1024;
    }
    // printf("Encoder Bitrate set to: %d bps for format %d\n", g_para.bps, g_para.fmt);

	ret = mpp_buffer_group_get_internal(&g_para.buf_grp, MPP_BUFFER_TYPE_DRM);
    if (ret) { printf("failed to get mpp buffer group ret %d\n", ret); return ret; }

	ret = mpp_buffer_get(g_para.buf_grp, &g_para.pkt_buf, g_para.frame_size);
    if (ret) { printf("failed to get buffer for output packet ret %d\n", ret); return ret; }

    ret = mpp_create(&g_para.ctx, &g_para.mpi);
    if (ret) { printf("mpp_create failed ret %d\n", ret); g_para.is_init = 1; EncodeProcessUninit(); return ret; }

	ret = g_para.mpi->control(g_para.ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
	if (MPP_OK != ret) { printf("mpi control set output timeout %d ret %d\n",timeout, ret); return ret; }

    ret = mpp_init(g_para.ctx, MPP_CTX_ENC, g_para.type);
    if (ret) { printf("mpp_init failed ret %d\n", ret); g_para.is_init = 1; EncodeProcessUninit(); return ret; }

	ret = mpp_enc_cfg_init(&g_para.cfg);
    if (ret) { printf("mpp_enc_cfg_init failed ret %d\n", ret); g_para.is_init = 1; EncodeProcessUninit(); return ret; }

	ret = g_para.mpi->control(g_para.ctx, MPP_ENC_GET_CFG, g_para.cfg);
    if (ret) { printf("get enc cfg failed ret %d\n", ret); g_para.is_init = 1; EncodeProcessUninit(); return ret; }

    ret = mpp_enc_cfg_setup();
    if (ret) { printf("test mpp setup failed ret %d\n", ret); g_para.is_init = 1; EncodeProcessUninit(); return ret; }

    if (g_para.sps_pps_packet == NULL) {
            void *header_buf = malloc(1024);
            if (header_buf == NULL) {
                 printf("malloc header buffer failed\n");
                 return -1;
            }

            ret = mpp_packet_init(&g_para.sps_pps_packet, header_buf, 1024);
            if (ret) {
                printf("mpp_packet_init failed ret %d\n", ret);
                free(header_buf);
                return ret;
            }

            mpp_packet_set_length(g_para.sps_pps_packet, 0);
        }

    ret = g_para.mpi->control(g_para.ctx, MPP_ENC_GET_HDR_SYNC, g_para.sps_pps_packet);
    if (ret) {
        printf("mpi control get header failed ret %d\n", ret);
    } else {
        printf("EncodeProcessInit: Successfully got SPS/PPS header, size: %zu\n",
               mpp_packet_get_length(g_para.sps_pps_packet));
    }
    // free(header_buf);
    
	g_para.is_init = 1;
	return EP_SUCCESS;
}

static void EncodeProcessSetCallback(encode_callback_t callback)
{
	g_para.callback = callback;
}

static EncodeProcess_t g_EncodeProcess =
{
	.init = EncodeProcessInit,
	.uninit = EncodeProcessUninit,
	.start = EncodeProcessStart,
	.stop = EncodeProcessStop,
	.SetCallback = EncodeProcessSetCallback,
	.camera_handle = EncodeProcessCameraHandle,
    .get_header_data = EncodeProcessGetHeaderData,
	.set_output_file = EncodeProcessSetOutputFile,
};

EncodeProcess_t* GetEncodeProcessInstance(void)
{
	return &g_EncodeProcess;
}
