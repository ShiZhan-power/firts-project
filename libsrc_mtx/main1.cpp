#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sched.h>
#include <pthread.h>

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <signal.h>
#include <linux/videodev2.h>
#include <sys/types.h>
#include <dirent.h>

#include "cserialmngr.h"

extern "C" {
    #include <libavformat/avformat.h>
    #include <libavcodec/avcodec.h>
    #include <libavutil/opt.h>
    #include <libavutil/timestamp.h>
    #include "encode.h"
    #include "v4l2camera.h"
}

#include "detect.h"
#include "mpp_frame.h"
#include "rk_mpi.h"
#include "mpp_packet.h"

// 开关命令行参数
#define ENABLE_CMD_LINE_ARGS 1

#define CAMERA_DEVICE_NAME  ("/dev/video22")
#define IR_V4L2_FMT		V4L2_PIX_FMT_NV12

static int ir_fps = 30;

AVFormatContext *ofmt_ctx = NULL;

AVStream *out_stream = NULL;
AVDictionary *avdic = NULL;
//const char *rtsp_url = "rtsp://127.0.0.1:8554/1";
const char *rtsp_url = "rtsp://192.168.1.30:8554/1";

int64_t frame_index = 0;

static int init_ffmpeg_streamer(int width, int height) {
    int ret;

    av_dict_set(&avdic, "rtsp_transport", "tcp", 0);
    av_dict_set(&avdic, "muxdelay", "0", 0);
    av_dict_set(&avdic, "tune", "zerolatency", 0);

    ret = avformat_alloc_output_context2(&ofmt_ctx, NULL, "rtsp", rtsp_url);
    if (ret < 0 || !ofmt_ctx) {
        fprintf(stderr, "Could not create output context\n");
        return -1;
    }

    ofmt_ctx->max_delay = 0;
    ofmt_ctx->flags |= AVFMT_FLAG_NOBUFFER | AVFMT_FLAG_FLUSH_PACKETS;

    out_stream = avformat_new_stream(ofmt_ctx, NULL);
    if (!out_stream) {
        fprintf(stderr, "Failed allocating output stream\n");
        return -1;
    }

    out_stream->codecpar->codec_id = AV_CODEC_ID_H264;
    out_stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    out_stream->codecpar->width = width;
    out_stream->codecpar->height = height;

    out_stream->time_base = (AVRational){1, ir_fps};

    EncodeProcess_t* ec = GetEncodeProcessInstance();
    size_t header_size = 256;
    char* header_buf = (char*)malloc(header_size);
    if (header_buf) {
        usleep(100000);
        if (ec->get_header_data(header_buf, &header_size) == 0 && header_size > 0) {
             printf("Successfully got encoder header data, size: %zu\n", header_size);
             out_stream->codecpar->extradata = (uint8_t*)av_mallocz(header_size + AV_INPUT_BUFFER_PADDING_SIZE);
            if (out_stream->codecpar->extradata) {
                memcpy(out_stream->codecpar->extradata, header_buf, header_size);
                out_stream->codecpar->extradata_size = header_size;
             }
        } else {
            fprintf(stderr, "WARN: Failed to get encoder header data. RTSP stream may not be playable.\n");
        }
        free(header_buf);
    }

    if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open(&ofmt_ctx->pb, rtsp_url, AVIO_FLAG_WRITE);
        if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE]; // 1. 定义一个缓冲区
            av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
            fprintf(stderr, "Could not open output URL '%s': %s\n", rtsp_url, errbuf);
            return -1;
        }
    }

    ret = avformat_write_header(ofmt_ctx, &avdic);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE]; // 1. 定义一个缓冲区
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        fprintf(stderr, "Error occurred when opening output URL: %s\n", errbuf);
        return -1;
    }

    printf("FFmpeg streamer initialized, pushing to %s\n", rtsp_url);
    return 0;
}


static void cleanup_ffmpeg_streamer() {
    if (avdic) {
        av_dict_free(&avdic);
    }
    if (ofmt_ctx) {
        av_write_trailer(ofmt_ctx);
        if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
            avio_closep(&ofmt_ctx->pb);
        }
        avformat_free_context(ofmt_ctx);
        ofmt_ctx = NULL;
        printf("FFmpeg streamer cleaned up.\n");
    }
}

void enccallBack(MppPacket packet)
{
    if (!packet || !ofmt_ctx) return;
    void* data = mpp_packet_get_pos(packet);
    int len = mpp_packet_get_length(packet);
    if (len <= 0) return;

    AVPacket pkt = { 0 };
    // av_init_packet(&pkt); // It's good practice to initialize the packet

    pkt.data = (uint8_t*)data;
    pkt.size = len;
    pkt.stream_index = out_stream->index;

    // --- TIMESTAMP FIX ---
    // Get the timestamp directly from the MppPacket
    int64_t pts_from_mpp = mpp_packet_get_pts(packet);

    // If the MPP timestamp is valid, use it. Otherwise, fall back to our index-based calculation.
    if (pts_from_mpp > 0) {
        pkt.pts = av_rescale_q(pts_from_mpp, (AVRational){1, 1000000}, out_stream->time_base);
    } else {
        // Fallback for safety, though we expect MPP to provide PTS
        int64_t pts_us = frame_index * (1000000 / ir_fps);
        pkt.pts = av_rescale_q(pts_us, (AVRational){1, 1000000}, out_stream->time_base);
    }
    
    pkt.dts = pkt.pts; // For H.264, DTS and PTS are usually the same
    pkt.duration = av_rescale_q(1000000 / ir_fps, (AVRational){1, 1000000}, out_stream->time_base);
    pkt.pos = -1;
    
    frame_index++; // Keep incrementing frame_index for the fallback

    MppMeta meta = mpp_packet_get_meta(packet);
    RK_S32 is_keyframe = 0;
    if (meta) {
        mpp_meta_get_s32(meta, KEY_OUTPUT_INTRA, &is_keyframe);
    }
    if (is_keyframe) {
        pkt.flags |= AV_PKT_FLAG_KEY;
    }

//    int ret = av_interleaved_write_frame(ofmt_ctx, &pkt);
    int ret = av_write_frame(ofmt_ctx, &pkt);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE]; // 1. 定义一个缓冲区
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        fprintf(stderr, "Error writing frame to RTSP stream: %s\n", errbuf);
    }
    
    av_packet_unref(&pkt); // Clean up the packet
}

void DetectToEncodeCallbackAdapter(int fd, int w, int h)
{
    EncodeProcess_t* ec = GetEncodeProcessInstance();
    if (ec && ec->camera_handle) {
        ec->camera_handle(fd, w, h);
    }
}

int app_deinit()
{
    printf("De-initializing application...\n");
    V4l2Camera_t *vc = GetV4l2CameraInstance();
    DetectProcess_t *dp = GetDetectProcessInstance();
    EncodeProcess_t* ec = GetEncodeProcessInstance();

    printf("Stopping camera...\n");
    vc->stop();
    printf("Stopping detect process...\n");
    dp->stop();
    printf("Stopping encode process...\n");
    ec->stop();
    printf("Cleaning up FFmpeg...\n");
    cleanup_ffmpeg_streamer();
    printf("Un-initializing modules...\n");
    ec->uninit();
    dp->uninit();
    vc->uninit();
    printf("Application de-initialized.\n");
    return 0;
}

void SigHandle(int sig_num)
{
    printf("rknn_demo receive sig num=%d\n", sig_num);
    app_deinit();
    exit(0);
}

int main(int argc, char* argv[])
{
    int ir_w = 3840;
    int ir_h = 2160;
    int enable_detect = 1;
    int enable_file = 0;

    if (!SerialManager::GetInstance()->init("/dev/ttyS3")) {
        printf("串口初始化失败\n");
        return -1;
    }

    SerialManager::GetInstance()->addMirrorPort("/dev/ttyUSB0");
    
#if ENABLE_CMD_LINE_ARGS
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-r") == 0) {
                ir_w = 1920;
                ir_h = 1080;
        } else if (strcmp(argv[i], "--no-detect") == 0) {
            enable_detect = 0;
        } else if (strcmp(argv[i], "-f") == 0) {
            enable_file = 1;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            return -1;
        }
    }
#endif
    printf("###########################################################\n");
    printf("Application Configuration:\n");
    printf("  Resolution: %dx%d\n", ir_w, ir_h);
    printf("  Detection: %s\n", enable_detect ? "Enabled" : "Disabled");
    printf("  Output.h264 file: %s\n", enable_file ? "Enabled" : "Disabled");
    printf("###########################################################\n");
    
    V4l2Camera_t *vc = GetV4l2CameraInstance();
    DetectProcess_t *dp = GetDetectProcessInstance();
    EncodeProcess_t* ec = GetEncodeProcessInstance();

    signal(SIGINT, SigHandle);
    signal(SIGTERM, SigHandle);

    // printf("Initializing V4L2 camera...\n");
    if (vc->init(CAMERA_DEVICE_NAME, ir_w, ir_h, ir_fps, IR_V4L2_FMT) != 0) {
        fprintf(stderr, "Failed to initialize V4L2 camera.\n");
        return -1;
    }
    // printf("Initializing detect process...\n");
    if (dp->init(enable_detect, ir_w, ir_h) != 0) {
        fprintf(stderr, "Failed to initialize detect process.\n");
        vc->uninit();
        return -1;
    }
    // printf("Initializing encode process...\n");
    if (ec->init(ir_w, ir_h, MPP_FMT_YUV420SP, MPP_VIDEO_CodingAVC, ir_fps, ir_fps) != 0) {
        fprintf(stderr, "Failed to initialize encode process.\n");
        dp->uninit();
        vc->uninit();
        return -1;
    }

    // printf("Setting up module callbacks...\n");
    vc->SetCallback(dp->camera_handle);
    dp->SetCallback(DetectToEncodeCallbackAdapter);
    ec->SetCallback(enccallBack);

    // printf("Starting encode thread...\n");
    ec->start();

    if(enable_file) ec->set_output_file("output.h264");
//    printf("Saving H.264 stream to output.h264\n");
    
    avformat_network_init();
    if (init_ffmpeg_streamer(ir_w, ir_h) != 0) {
        fprintf(stderr, "Failed to initialize FFmpeg streamer.\n");
        app_deinit();
        return -1;
    }

    // printf("Starting remaining processing threads...\n");
    dp->start();
    vc->start();

    // std::vector<uint8_t> payload = {0x03, 0x00};
    // SerialManager::GetInstance()->sendCommand(0x0002, payload);

    while(1) {
        usleep(5 * 1000);
    }

    printf("Application started. Press Ctrl+C to exit.\n");
    while(1)sleep(10);

    return 0;
}
