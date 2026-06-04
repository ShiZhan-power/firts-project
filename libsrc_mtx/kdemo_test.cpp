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
#include <sys/types.h>
#include <dirent.h>

#include "cserialmngr.h"

extern "C" {
    #include <libavformat/avformat.h>
    #include <libavcodec/avcodec.h>
    #include <libavutil/opt.h>
    #include <libavutil/timestamp.h>
    #include <libavutil/imgutils.h>
    #include <libswscale/swscale.h>
    #include "encode.h"
    #include "v4l2camera.h" // Needed for camera_buffer struct definition
}

#include "detect.h"
#include "mpp_frame.h"
#include "rk_mpi.h"
#include "mpp_packet.h"
#include "dma_alloc.h"

// Global variables for cleanup
static int g_running = 1;
static AVFormatContext *g_fmt_ctx = NULL;
static AVCodecContext *g_dec_ctx = NULL;
static AVStream *g_video_stream = NULL;
static int g_video_stream_idx = -1;
static AVFrame *g_frame = NULL;
static AVPacket *g_pkt = NULL;
static struct SwsContext *g_sws_ctx = NULL;
static AVFrame *g_frame_nv12 = NULL;

// DMA Buffer for DetectProcess
struct DmaBuffer_s {
    int fd;
    void* ptr;
    size_t size;
};
static DmaBuffer_s g_dma_buf = { -1, NULL, 0 };

// Output RTSP/File variables (copied from main.cpp)
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
        }
        free(header_buf);
    }

    if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open(&ofmt_ctx->pb, rtsp_url, AVIO_FLAG_WRITE);
        if (ret < 0) {
            char errbuf[AV_ERROR_MAX_STRING_SIZE];
            av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
            fprintf(stderr, "Could not open output URL '%s': %s\n", rtsp_url, errbuf);
            return -1;
        }
    }

    ret = avformat_write_header(ofmt_ctx, &avdic);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
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

    pkt.data = (uint8_t*)data;
    pkt.size = len;
    pkt.stream_index = out_stream->index;

    int64_t pts_from_mpp = mpp_packet_get_pts(packet);

    if (pts_from_mpp > 0) {
        pkt.pts = av_rescale_q(pts_from_mpp, (AVRational){1, 1000000}, out_stream->time_base);
    } else {
        int64_t pts_us = frame_index * (1000000 / ir_fps);
        pkt.pts = av_rescale_q(pts_us, (AVRational){1, 1000000}, out_stream->time_base);
    }
    
    pkt.dts = pkt.pts;
    pkt.duration = av_rescale_q(1000000 / ir_fps, (AVRational){1, 1000000}, out_stream->time_base);
    pkt.pos = -1;
    
    frame_index++;

    MppMeta meta = mpp_packet_get_meta(packet);
    RK_S32 is_keyframe = 0;
    if (meta) {
        mpp_meta_get_s32(meta, KEY_OUTPUT_INTRA, &is_keyframe);
    }
    if (is_keyframe) {
        pkt.flags |= AV_PKT_FLAG_KEY;
    }

    int ret = av_write_frame(ofmt_ctx, &pkt);
    if (ret < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE];
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, ret);
        fprintf(stderr, "Error writing frame to RTSP stream: %s\n", errbuf);
    }
    
    av_packet_unref(&pkt);
}

void DetectToEncodeCallbackAdapter(int fd, int w, int h)
{
    EncodeProcess_t* ec = GetEncodeProcessInstance();
    if (ec && ec->camera_handle) {
        ec->camera_handle(fd, w, h);
    }
}

void SigHandle(int sig_num)
{
    printf("Receive sig num=%d\n", sig_num);
    g_running = 0;
}

static int open_input_file(const char *filename)
{
    int ret;

    if ((ret = avformat_open_input(&g_fmt_ctx, filename, NULL, NULL)) < 0) {
        fprintf(stderr, "Cannot open input file '%s'\n", filename);
        return ret;
    }

    if ((ret = avformat_find_stream_info(g_fmt_ctx, NULL)) < 0) {
        fprintf(stderr, "Cannot find stream information\n");
        return ret;
    }

    ret = av_find_best_stream(g_fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    if (ret < 0) {
        fprintf(stderr, "Cannot find a video stream in the input file\n");
        return ret;
    }
    g_video_stream_idx = ret;
    g_video_stream = g_fmt_ctx->streams[g_video_stream_idx];

    const AVCodec *dec = avcodec_find_decoder(g_video_stream->codecpar->codec_id);
    if (!dec) {
        fprintf(stderr, "Failed to find codec\n");
        return AVERROR(EINVAL);
    }

    g_dec_ctx = avcodec_alloc_context3(dec);
    if (!g_dec_ctx) {
        fprintf(stderr, "Failed to allocate the codec context\n");
        return AVERROR(ENOMEM);
    }

    if ((ret = avcodec_parameters_to_context(g_dec_ctx, g_video_stream->codecpar)) < 0) {
        fprintf(stderr, "Failed to copy codec parameters to decoder context\n");
        return ret;
    }

    if ((ret = avcodec_open2(g_dec_ctx, dec, NULL)) < 0) {
        fprintf(stderr, "Failed to open codec\n");
        return ret;
    }

    return 0;
}

int main(int argc, char* argv[])
{
    int ir_w = 0;
    int ir_h = 0;
    int enable_detect = 1;
    int enable_file = 0;
    char *input_filename = NULL;

    if (!SerialManager::GetInstance()->init("/dev/ttyS3")) {
        printf("串口初始化失败\n");
        return -1;
    }
    SerialManager::GetInstance()->addMirrorPort("/dev/ttyUSB0");

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-detect") == 0) {
            enable_detect = 0;
        } else if (strcmp(argv[i], "-f") == 0) {
            enable_file = 1;
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_filename = argv[++i];
        }
    }

    if (!input_filename) {
        fprintf(stderr, "Usage: %s -i <video_file> [--no-detect] [-f]\n", argv[0]);
        return -1;
    }

    // Initialize FFmpeg Input FIRST to get resolution
    if (open_input_file(input_filename) < 0) {
        return -1;
    }

    // Set resolution from video stream
    ir_w = g_video_stream->codecpar->width;
    ir_h = g_video_stream->codecpar->height;

    printf("###########################################################\n");
    printf("Test Configuration:\n");
    printf("  Input File: %s\n", input_filename);
    printf("  Resolution: %dx%d (From Video)\n", ir_w, ir_h);
    printf("  Detection: %s\n", enable_detect ? "Enabled" : "Disabled");
    printf("  Output.h264 file: %s\n", enable_file ? "Enabled" : "Disabled");
    printf("###########################################################\n");

    DetectProcess_t *dp = GetDetectProcessInstance();
    EncodeProcess_t* ec = GetEncodeProcessInstance();

    signal(SIGINT, SigHandle);
    signal(SIGTERM, SigHandle);

    // Allocate DMA buffer for NV12 frames
    // NV12 size = w * h * 1.5
    size_t frame_size = ir_w * ir_h * 3 / 2;
    if (dma_buf_alloc(DMA_HEAP_DMA32_UNCACHE_PATCH, frame_size, &g_dma_buf.fd, &g_dma_buf.ptr) < 0) {
        fprintf(stderr, "Failed to allocate DMA buffer\n");
        return -1;
    }
    printf("Allocated DMA buffer fd=%d, size=%zu\n", g_dma_buf.fd, frame_size);

    // Initialize Detect Process
    if (dp->init(enable_detect, ir_w, ir_h) != 0) {
        fprintf(stderr, "Failed to initialize detect process.\n");
        return -1;
    }

    // Initialize Encode Process
    if (ec->init(ir_w, ir_h, MPP_FMT_YUV420SP, MPP_VIDEO_CodingAVC, ir_fps, ir_fps) != 0) {
        fprintf(stderr, "Failed to initialize encode process.\n");
        dp->uninit();
        return -1;
    }

    dp->SetCallback(DetectToEncodeCallbackAdapter);
    ec->SetCallback(enccallBack);

    ec->start();
    if(enable_file) ec->set_output_file("output.h264");

    avformat_network_init();
    if (init_ffmpeg_streamer(ir_w, ir_h) != 0) {
        fprintf(stderr, "Failed to initialize FFmpeg streamer.\n");
        return -1;
    }

    dp->start();

    g_frame = av_frame_alloc();
    g_pkt = av_packet_alloc();
    g_frame_nv12 = av_frame_alloc();
    
    // Setup g_frame_nv12 to point to our DMA buffer
    // We need to fill the linesize and data pointers manually or use av_image_fill_arrays
    // NV12: Y plane, then UV plane.
    // Y plane size: w * h
    // UV plane size: w * h / 2
    // Stride (linesize): w
    
    // Note: DetectProcess expects aligned strides, usually. 
    // detect.cpp: g_para.hor_stride = MPP_ALIGN(g_para.src_w, 64);
    // Let's check if we need to align. For simplicity, let's assume ir_w is aligned or we handle it.
    // If DetectProcess uses MPP_ALIGN, we should probably respect that.
    // In detect.cpp: 
    // g_para.hor_stride = MPP_ALIGN(g_para.src_w, 64);
    // g_para.ver_stride = MPP_ALIGN(g_para.src_h, 16);
    // We should probably match this stride in our DMA buffer layout if we want to be safe.
    
    // Re-calculating frame size based on alignment to be safe
    #define MPP_ALIGN(x, a)         (((x)+(a)-1)&~((a)-1))
    int hor_stride = MPP_ALIGN(ir_w, 64);
    int ver_stride = MPP_ALIGN(ir_h, 16);
    size_t aligned_frame_size = hor_stride * ver_stride * 3 / 2;
    
    if (aligned_frame_size > frame_size) {
        // Re-allocate if our initial guess was too small (though ir_w*ir_h*1.5 is usually smaller than aligned)
        // Actually, let's just use the aligned size for the DMA alloc to be correct.
        dma_buf_free(frame_size, &g_dma_buf.fd, &g_dma_buf.ptr);
        frame_size = aligned_frame_size;
        if (dma_buf_alloc(DMA_HEAP_DMA32_UNCACHE_PATCH, frame_size, &g_dma_buf.fd, &g_dma_buf.ptr) < 0) {
            fprintf(stderr, "Failed to re-allocate DMA buffer\n");
            return -1;
        }
        printf("Re-allocated DMA buffer fd=%d, size=%zu (aligned)\n", g_dma_buf.fd, frame_size);
    }

    // Assign DMA buffer pointers to AVFrame
    g_frame_nv12->width = ir_w;
    g_frame_nv12->height = ir_h;
    g_frame_nv12->format = AV_PIX_FMT_NV12;
    
    // Manual assignment for NV12 in a single buffer
    g_frame_nv12->data[0] = (uint8_t*)g_dma_buf.ptr;
    g_frame_nv12->data[1] = (uint8_t*)g_dma_buf.ptr + hor_stride * ver_stride;
    g_frame_nv12->linesize[0] = hor_stride;
    g_frame_nv12->linesize[1] = hor_stride; // UV plane stride is same as Y for NV12 usually

    // Camera buffer struct for DetectProcess
    struct camera_plane_info plane = {0};
    plane.fd = g_dma_buf.fd;
    plane.start = g_dma_buf.ptr;
    plane.length = frame_size;
    
    struct camera_buffer cam_buf = {0};
    cam_buf.planes = &plane;
    cam_buf.n_planes = 1;

    printf("Starting video processing loop...\n");
    
    double frame_delay = 1.0 / ir_fps; 
    // If video has FPS, use it
    if (g_video_stream->avg_frame_rate.num > 0) {
        frame_delay = av_q2d(av_inv_q(g_video_stream->avg_frame_rate));
    }

    while (g_running && av_read_frame(g_fmt_ctx, g_pkt) >= 0) {
        if (g_pkt->stream_index == g_video_stream_idx) {
            int ret = avcodec_send_packet(g_dec_ctx, g_pkt);
            if (ret < 0) {
                fprintf(stderr, "Error sending packet for decoding\n");
                break;
            }

            while (ret >= 0) {
                ret = avcodec_receive_frame(g_dec_ctx, g_frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    break;
                else if (ret < 0) {
                    fprintf(stderr, "Error during decoding\n");
                    goto end;
                }

                // Check if we can do a direct copy (Fast Path)
                // Condition: Input is already NV12 and resolution matches target
                if (g_frame->format == AV_PIX_FMT_NV12 && 
                    g_frame->width == ir_w && 
                    g_frame->height == ir_h) {
                    
                    // Direct copy Y plane
                    uint8_t* src_y = g_frame->data[0];
                    uint8_t* dst_y = (uint8_t*)g_dma_buf.ptr;
                    for (int i = 0; i < ir_h; i++) {
                        memcpy(dst_y + i * hor_stride, src_y + i * g_frame->linesize[0], ir_w);
                    }

                    // Direct copy UV plane
                    uint8_t* src_uv = g_frame->data[1];
                    uint8_t* dst_uv = (uint8_t*)g_dma_buf.ptr + hor_stride * ver_stride;
                    for (int i = 0; i < ir_h / 2; i++) {
                        memcpy(dst_uv + i * hor_stride, src_uv + i * g_frame->linesize[1], ir_w);
                    }
                } else {
                    // Slow Path: Format conversion or resizing needed
                    if (!g_sws_ctx) {
                         g_sws_ctx = sws_getContext(g_dec_ctx->width, g_dec_ctx->height, g_dec_ctx->pix_fmt,
                                                   ir_w, ir_h, AV_PIX_FMT_NV12,
                                                   SWS_BILINEAR, NULL, NULL, NULL);
                        if (!g_sws_ctx) {
                            fprintf(stderr, "Impossible to create scale context for the conversion\n");
                            goto end;
                        }
                    }
                    
                    // Note: sws_scale might re-init if input format changes, but here we assume constant stream
                    sws_scale(g_sws_ctx, (const uint8_t * const*)g_frame->data, g_frame->linesize,
                              0, g_frame->height, g_frame_nv12->data, g_frame_nv12->linesize);
                }

                if (dp->camera_handle) {
                    dp->camera_handle(&cam_buf, ir_w, ir_h, 0);
                }

                // Simple rate control
                usleep((useconds_t)(frame_delay * 1000000));
            }
        }
        av_packet_unref(g_pkt);
    }

end:
    printf("Cleaning up...\n");
    dp->stop();
    ec->stop();
    cleanup_ffmpeg_streamer();
    
    ec->uninit();
    dp->uninit();
    
    if (g_dma_buf.fd >= 0) {
        dma_buf_free(frame_size, &g_dma_buf.fd, &g_dma_buf.ptr);
    }
    
    av_frame_free(&g_frame);
    av_frame_free(&g_frame_nv12);
    av_packet_free(&g_pkt);
    avcodec_free_context(&g_dec_ctx);
    avformat_close_input(&g_fmt_ctx);
    sws_freeContext(g_sws_ctx);

    return 0;
}
