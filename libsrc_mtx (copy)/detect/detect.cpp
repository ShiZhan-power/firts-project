#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>

#include <vector>
#include <map>
#include <string>
#include <set>
#include <cmath>

#include "cserialmngr.h"
#include "rknn_api.h"
#include "detect.h"
#include "yolov8.h"
#include "rgamap.h"
#include "postprocess.h"
#include "v4l2camera.h"
#include "BYTETracker.h"
//#include "PositionFilter.hpp"

#include "rga/RgaApi.h"
#include "rga/im2d.h"
//#include "rga/RgaUtils.h"

#include "dma_alloc.h"

#include "mpp_common.h"

#include "eis.h"

#define MODEL_NAME    "/yolov8.rknn"
#define DST_W         640
#define DST_H         640
#define DST_BPP       24

const double FOV_W = 18.2;//86.0;
const double FOV_H = 10.3;//48.7867495;

#define FONT_SCALE 9
const int FONT_WIDTH = 8;
const int FONT_HEIGHT = 10;

// PositionFilter pf(20);

//           a b g r           r g b  a
//(代码中)0XFFFFFF00 = (屏幕)0X00FFFF FF-> (蓝色)
//(代码中)0xFF00FFFF = (屏幕)0XFFFF00 FF-> (黄色)
//(代码中)0xFFFF00FF = (屏幕)0XFF00FF FF-> (洋红色)
#define COLOR_GREEN 0xFFFF00FF
#define COLOR_BG 0x00000000

static std::map<char, im_rect> g_glyph_cache;

struct DmaBuffer_s {
    int fd;
    void* ptr;
    size_t size;
};

static bool g_enable_detect = true;
static bool g_enable_sendpos = true;

static EisProcess g_eis;


static struct DetectProcessPara_s
{
    int is_init;
    rknn_app_context_t app_ctx;

    DmaBuffer_s rknn_in_buf;

    int hor_stride;
    int ver_stride;

    pthread_t ai_thread_pid;
    int ai_thread_on;
    
    pthread_t send_thread_pid;
    int send_thread_on;

    detect_callback_t callback;
    long long frame_counter;

    int src_w;
    int src_h;

    pthread_mutex_t mtx;
    pthread_cond_t cond;

    int latest_frame_fd;
    long long latest_frame_counter;
    bool new_frame_available;
    int latest_frame_w;      // 新增：当前帧的实际宽度
    int latest_frame_h;      // 新增：当前帧的实际高度

    struct ssd_group objects_to_draw;

    int primary_target_id;
    bool stable_coord_valid;

    float ref_x;                   // 中心 X
    float ref_y;                   // 中心 Y
    float ref_vx;                  // X 轴速度 (像素/帧)
    float ref_vy;                  // Y 轴速度 (像素/帧)
    //long long ref_frame_counter;   // 产生上述数据的帧号
  
    double ref_timestamp;

    rknn_output rknn_outputs[16];
    void* rknn_output_bufs[16];

    BYTETracker tracker;

    // Font Atlas
    rga_buffer_t font_atlas_buf;
    void* font_atlas_ptr;
    int font_atlas_fd;
    int font_atlas_w;
    int font_atlas_h;

}g_para =
{
    .is_init = 0,
    .app_ctx = {0},
    .rknn_in_buf = {0},
    .hor_stride = 0,
    .ver_stride = 0,
    .ai_thread_pid = 0,
    .ai_thread_on = 0,
    .callback = NULL,
    .frame_counter = 0,
    .src_w = 0,
    .src_h = 0,
    .latest_frame_fd = -1,
    .latest_frame_counter = 0,
    .new_frame_available = false,
    .latest_frame_w = 0,
    .latest_frame_h = 0,
    .objects_to_draw = {0},
    .primary_target_id = -1,
    .stable_coord_valid = false,
    
    .ref_timestamp = 0,
    
    
    .font_atlas_buf = {0},
    .font_atlas_ptr = NULL,
    .font_atlas_fd = -1,
    .font_atlas_w = 0,
    .font_atlas_h = 0,
};



static double get_now_time()
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return ts.tv_sec +
           ts.tv_nsec / 1000000000.0;
}


void sendAngleData(uint16_t cmd, int pos_x, int pos_y, int src_w, int src_h)
{
    double angle_x = (double)pos_x * (FOV_W / (double)src_w);
    double angle_y = (double)pos_y * (FOV_H / (double)src_h);

    int16_t x_data = (int16_t)round(angle_x * 10.0);
    int16_t y_data = (int16_t)round(angle_y * 10.0);

    std::vector<uint8_t> payload;
    payload.reserve(4);

    // payload { x_low, x_high, y_low, y_high }
    payload.push_back(x_data & 0xFF);
    payload.push_back((x_data >> 8) & 0xFF);
    payload.push_back(y_data & 0xFF);
    payload.push_back((y_data >> 8) & 0xFF);

    // SerialManager::GetInstance()->sendCommand(cmd, payload);
    SerialManager::GetInstance()->sendCommandToMirror(cmd, payload);

    // printf("X=%d, Y=%d\n", pos_x, pos_y);
    // printf("------%d,%d\n",(g_para.src_w/2)+pos_x, (g_para.src_h/2)-pos_y);
    // printf("Angle: X=%.1f (Int:%d), Y=%.1f (Int:%d)\n", angle_x, x_data, angle_y, y_data);
   printf("Angle: X=%.1f , Y=%.1f \n", angle_x, angle_y);
}

static int rknn_run_and_get_outputs(void* in_data)
{
    int status = 0;
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = DST_W * DST_H * DST_BPP / 8;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = in_data;

    status = rknn_inputs_set(g_para.app_ctx.rknn_ctx, 1, inputs);
    if(status < 0) {
        printf("ERROR: rknn_inputs_set fail! ret=%d\n", status);
        return -1;
    }

    status = rknn_run(g_para.app_ctx.rknn_ctx, NULL);
    if(status < 0) {
        printf("ERROR: rknn_run fail! ret=%d\n", status);
        return -1;
    }

    status = rknn_outputs_get(g_para.app_ctx.rknn_ctx, g_para.app_ctx.io_num.n_output, g_para.rknn_outputs, NULL);
    if(status < 0) {
        printf("ERROR: rknn_outputs_get fail! ret=%d\n", status);
        return -1;
    }
    return 0;
}


static int rga_blit_wrapper(int src_fd, int src_w, int src_h, int src_fmt,
                            int dst_fd, int dst_w, int dst_h, int dst_fmt)
{
    int src_hor_stride = MPP_ALIGN(src_w, 64);
    int src_ver_stride = MPP_ALIGN(src_h, 16);
    int dst_hor_stride = MPP_ALIGN(dst_w, 64);
    //int dst_ver_stride = dst_h;
    int dst_ver_stride = MPP_ALIGN(dst_h, 16);


    rga_info_t src, dst;
    memset(&src, 0, sizeof(rga_info_t));
    src.fd = src_fd; src.mmuFlag = 1;
    rga_set_rect(&src.rect, 0, 0, src_w, src_h, src_hor_stride, src_ver_stride, src_fmt);

    memset(&dst, 0, sizeof(rga_info_t));
    dst.fd = dst_fd; dst.mmuFlag = 1;
    rga_set_rect(&dst.rect, 0, 0, dst_w, dst_h, dst_hor_stride, dst_ver_stride, dst_fmt);

    int ret = c_RkRgaBlit(&src, &dst, NULL);
    if (ret) {
        printf("ERROR: c_RkRgaBlit for scale/conversion failed with return code %d: %s\n", ret, strerror(errno));
    }
    return ret;
}

inline static int clamp(float val, int min, int max) { return val > min ? (val < max ? val : max) : min; }

static int draw_rects_on_yuv_with_rga(int yuv_fd, struct ssd_group *group, int w, int h) {
    if (group == NULL || group->count == 0) return 1;

    rga_buffer_t dst_buf_base = wrapbuffer_fd(yuv_fd, w, h, RK_FORMAT_YCbCr_420_SP, g_para.hor_stride, g_para.ver_stride);
    int usage = IM_SYNC | IM_ALPHA_BLEND_SRC_OVER;// | RK_ALPHA_PREMUL;

    std::vector<im_rect> all_rects;

    for (int i = 0; i < group->count; i++) {
        im_rect box_rect = {};
        box_rect.x      = (clamp(group->objects[i].select.left               ,0 ,w             ) / 2) * 2;
        box_rect.y      = (clamp(group->objects[i].select.top                ,0 ,h             ) / 2) * 2;
        box_rect.width  = (clamp(group->objects[i].select.right - box_rect.x ,0 ,w - box_rect.x) / 2) * 2;
        box_rect.height = (clamp(group->objects[i].select.bottom - box_rect.y,0 ,h - box_rect.y) / 2) * 2;

        all_rects.push_back({box_rect.x, box_rect.y, box_rect.width, box_rect.height});

        bool is_primary_target = (group->objects[i].id == g_para.primary_target_id);

        // Draw '+' for primary target
        if(is_primary_target && g_glyph_cache.count('+')){
            im_rect glyph_src = g_glyph_cache.at('+');

            int tx = (((group->objects[i].select.left + group->objects[i].select.right) / 2 - 3 * FONT_SCALE) / 2) * 2;
            int ty = (((group->objects[i].select.top + group->objects[i].select.bottom) / 2 - 3 * FONT_SCALE) / 2) * 2;

            rga_buffer_t src_buf = g_para.font_atlas_buf;
            rga_buffer_t dst_buf = dst_buf_base;
            
            im_rect src_rect = glyph_src;
            im_rect dst_rect = {tx, ty, glyph_src.width, glyph_src.height};

            rga_buffer_t pat_buf = {};
            im_rect pat_rect = {};
            improcess(src_buf, dst_buf, pat_buf, src_rect, dst_rect, pat_rect, -1, NULL, NULL, usage);
        }

        char text_to_draw[16];
        // if(is_primary_target) {
        //     // snprintf(text_to_draw, sizeof(text_to_draw), "%d", group->objects[i].id);
        //     snprintf(text_to_draw, sizeof(text_to_draw), "%s", "abcdefghijklmnopqrstuvwxyz");
        // }
        // else {
        snprintf(text_to_draw, sizeof(text_to_draw), "%d", group->objects[i].id);
        // }
        int text_y = box_rect.y - FONT_HEIGHT * FONT_SCALE;
        if (text_y < 0) text_y = box_rect.y + box_rect.height;
        
        int current_x = box_rect.x;
        for (int j = 0; text_to_draw[j] != '\0'; ++j) {
            char c = text_to_draw[j];
            if (g_glyph_cache.count(c)) {
                im_rect glyph_src = g_glyph_cache.at(c);
                
                rga_buffer_t src_buf = g_para.font_atlas_buf;
                rga_buffer_t dst_buf = dst_buf_base;

                im_rect src_rect = glyph_src;
                im_rect dst_rect = {current_x, text_y, glyph_src.width, glyph_src.height};

                rga_buffer_t pat_buf = {};
                im_rect pat_rect = {};
                improcess(src_buf, dst_buf, pat_buf, src_rect, dst_rect, pat_rect, -1, NULL, NULL, usage);
            }
            current_x += FONT_WIDTH * FONT_SCALE;
        }
    }

    if (!all_rects.empty()) {
        imrectangleArray(dst_buf_base, all_rects.data(), all_rects.size(), 0x00FF00FF, 4, 1, NULL);
    }

    return 1;
}

static void DetectProcessCameraHandle(struct camera_buffer* cam_buf, int w, int h, int ch)
{
    if (!g_para.is_init) return;
    if (!cam_buf || !cam_buf->planes || cam_buf->planes[0].fd < 0) {
        printf("ERROR: Invalid camera buffer or FD from V4L2.\n");
        return;
    }

    g_para.frame_counter++;
    int frame_fd = cam_buf->planes[0].fd;
    
    //g_eis.process(frame_fd);
    // 1. 执行防抖（裁剪到 eis_buf_）
    if (g_eis.process(frame_fd) != 0) {
        // 防抖失败时，可以回退到原始帧（或者直接跳过）
        g_para.latest_frame_fd = frame_fd;
        g_para.latest_frame_w = w;   // 你需要添加这两个成员变量，或者直接用全局的 src_w_
        g_para.latest_frame_h = h;
        // 这里简单返回，或者设置 stable_fd = frame_fd
    }else{
    
       g_para.latest_frame_fd = g_eis.getOutputFd();
       g_para.latest_frame_w = g_eis.getCropW();
       g_para.latest_frame_h = g_eis.getCropH();
    
    }
    int stable_fd = g_eis.getOutputFd();   // 获得稳定后的帧 fd

    // 2. 后续所有操作都使用 stable_fd
    //    注意：stable_fd 对应的图像尺寸是 crop_w_ x crop_h_ (1280x720)
    //    而原来的 w, h 是原始相机尺寸 (3840x2160)
    //    所以需要更新宽高参数，否则绘制和检测会出错。

    // 如果你需要绘制检测框，必须传入正确的宽高（1280,720）
    

    
    
    /***
    if (g_enable_sendpos)
    //使用卡尔曼滤波的参考坐标进行线性预测，然后调用sendAngleData发送预测坐标
    {
        int pred_x_to_send = 0;
        int pred_y_to_send = 0;
        bool should_send = false;

        pthread_mutex_lock(&g_para.mtx);
        if (g_para.stable_coord_valid)
        {
            // 计算时间差 (当前帧 - AI算出那一帧)
            long long delta_frames = g_para.frame_counter - g_para.ref_frame_counter;

            // 限制预测范围：如果 AI 超过 10 帧(约300ms)没更新，就停止预测，防止飞出屏幕
            if (delta_frames >= 0 && delta_frames < 10)
            {
                // 线性预测公式: Pos = RefPos + Velocity * Time
                float pred_cx = g_para.ref_x + g_para.ref_vx * (float)delta_frames;
                float pred_cy = g_para.ref_y + g_para.ref_vy * (float)delta_frames;

                // 边界检查
                if (pred_cx < 0) pred_cx = 0;
                if (pred_cx > g_para.src_w) pred_cx = g_para.src_w;
                if (pred_cy < 0) pred_cy = 0;
                if (pred_cy > g_para.src_h) pred_cy = g_para.src_h;

                // 转为相对坐标 (以中心为0)
                pred_x_to_send = (int)round(pred_cx - (g_para.src_w / 2));
                pred_y_to_send = (int)round((g_para.src_h / 2) - pred_cy);
                should_send = true;
            }
        }
        pthread_mutex_unlock(&g_para.mtx);

        if (should_send) {
            // 正常发送
            sendAngleData(0x0001, pred_x_to_send, pred_y_to_send, g_para.src_w, g_para.src_h);

            // 横纵坐标交换，纵坐标取反
            // sendAngleData(0x0001, g_para.stable_coord_y_to_send*(-1), g_para.stable_coord_x_to_send, g_para.src_w, g_para.src_h);

            // 均值滤波
            // pf.add(g_para.stable_coord_y_to_send*(-1),g_para.stable_coord_x_to_send);
            // sendAngleData(0x0001, pf.get_filtered_x(), pf.get_filtered_y(), g_para.src_w, g_para.src_h);
        }
    }***/
    
    
    
    
    // 注意：draw_rects_on_yuv_with_rga 内部使用的 g_para.hor_stride/ver_stride 是基于原始分辨率的，
        // 如果你要绘制在 1280x720 上，需要重新计算 stride，或者修改该函数。简单起见，可以先注释掉绘制，
        // 先验证画面是否在移动。
        

    if (g_enable_detect) {
        struct ssd_group objects_display;

        pthread_mutex_lock(&g_para.mtx);
        objects_display = g_para.objects_to_draw;
        pthread_mutex_unlock(&g_para.mtx);
        draw_rects_on_yuv_with_rga(stable_fd, &objects_display, g_eis.getCropW(), g_eis.getCropH());
        //draw_rects_on_yuv_with_rga(frame_fd, &objects_display, w, h);
    }

    if (g_para.callback) {
        g_para.callback(g_para.latest_frame_fd,
                        g_para.latest_frame_w,
                        g_para.latest_frame_h);
    }

    if (g_enable_detect)
    {
        pthread_mutex_lock(&g_para.mtx);
        g_para.latest_frame_fd = stable_fd;
        g_para.latest_frame_counter = g_para.frame_counter;
        g_para.new_frame_available = true;
        pthread_cond_signal(&g_para.cond);
        pthread_mutex_unlock(&g_para.mtx);
    }
}




void* SendLoopProcess(void* args)
{

 
 
    const long PERIOD_NS = 10 * 1000 * 1000;

    struct timespec next_time;

  
   

    clock_gettime(
        CLOCK_MONOTONIC,
        &next_time);

    while(g_para.send_thread_on)
    {
      
       
       
        next_time.tv_nsec += PERIOD_NS;

        
        while(next_time.tv_nsec >= 1000000000)
        {
            next_time.tv_sec += 1;
            next_time.tv_nsec -= 1000000000;
        }

       
        clock_nanosleep(
            CLOCK_MONOTONIC,
            TIMER_ABSTIME,
            &next_time,
            NULL);
            
       
	// 
	/***
	
	static double last_send_time = 0;

	double now_test = get_now_time();

	if(last_send_time > 0)
	{
    	  double diff_ms =
             (now_test - last_send_time) * 1000.0;

          printf(
              "[SEND_INTERVAL] %.3f ms\n",
              diff_ms);
        }

        last_send_time = now_test;    
        ***/
        
        
        
        
        

        if(!g_enable_sendpos)
            continue;

        int pred_x_to_send = 0;
        int pred_y_to_send = 0;

        bool should_send = false;

        pthread_mutex_lock(&g_para.mtx);

        if(g_para.stable_coord_valid)
        {
            
            double now = get_now_time();

          
            double dt =
                now - g_para.ref_timestamp;

            
            if(dt >= 0 && dt < 0.3)
            {
              
                float pred_cx =
                    g_para.ref_x +
                    g_para.ref_vx * dt;

                float pred_cy =
                    g_para.ref_y +
                    g_para.ref_vy * dt;

              
                if(pred_cx < 0)
                    pred_cx = 0;

                if(pred_cx > g_para.src_w)
                    pred_cx = g_para.src_w;

                if(pred_cy < 0)
                    pred_cy = 0;

                if(pred_cy > g_para.src_h)
                    pred_cy = g_para.src_h;

               
                pred_x_to_send =
                    (int)round(
                    pred_cx -
                    (g_para.src_w / 2));

                pred_y_to_send =
                    (int)round(
                    (g_para.src_h / 2)
                    - pred_cy);

                should_send = true;
            }
        }

        pthread_mutex_unlock(&g_para.mtx);

        
        if(should_send)
        {
            sendAngleData(
                0x0001,
                pred_x_to_send,
                pred_y_to_send,
                g_para.src_w,
                g_para.src_h);
        }
    }

    printf("Send thread exited.\n");

    return NULL;
}


void* DetectLoopProcess(void* args)
{
    std::vector<Object> od_results;
    const std::vector<Object> empty_objects;
    std::vector<STrack> output_stracks;
    int current_process_fd = -1;
    long long current_process_frame_idx = 0;
    int current_frame_w = 0, current_frame_h = 0;   // 新增

    while(g_para.ai_thread_on)
    {
        pthread_mutex_lock(&g_para.mtx);
        while(!g_para.new_frame_available && g_para.ai_thread_on) {
            pthread_cond_wait(&g_para.cond, &g_para.mtx);
        }
        if (!g_para.ai_thread_on) {
            pthread_mutex_unlock(&g_para.mtx);
            break;
        }

        current_process_fd = g_para.latest_frame_fd;
        current_process_frame_idx = g_para.latest_frame_counter;
        current_frame_w = g_para.latest_frame_w;   // 获取实际宽度
        current_frame_h = g_para.latest_frame_h;   // 获取实际高度
        g_para.new_frame_available = false;

        pthread_mutex_unlock(&g_para.mtx);

        rga_blit_wrapper(current_process_fd, current_frame_w, current_frame_h, RK_FORMAT_YCbCr_420_SP,
                         g_para.rknn_in_buf.fd, DST_W, DST_H, RK_FORMAT_RGB_888);
        rknn_run_and_get_outputs(g_para.rknn_in_buf.ptr);

        post_process(&g_para.app_ctx, g_para.rknn_outputs,
                     current_frame_w, current_frame_h, 
                     BOX_THRESH, NMS_THRESH,
                     od_results);

        for (auto it = od_results.begin(); it != od_results.end(); ) {
            if (it->box.width * it->box.height < 20) {
                it = od_results.erase(it);
            } else {
                ++it;
            }
        }

        output_stracks = g_para.tracker.update(od_results);

        struct ssd_group objects_temp = {0};
        for (int i = 0; i < output_stracks.size(); i++) {
            if (objects_temp.count >= 100) break;

            std::vector<float> tlwh = output_stracks[i].tlwh;
            if(output_stracks[i].is_activated)
            {
                int idx = objects_temp.count;
                objects_temp.objects[idx].id = output_stracks[i].track_id;
                objects_temp.objects[idx].select.left = tlwh[0];
                objects_temp.objects[idx].select.top = tlwh[1];
                objects_temp.objects[idx].select.right = tlwh[0] + tlwh[2];
                objects_temp.objects[idx].select.bottom = tlwh[1] + tlwh[3];

                const char* name = coco_cls_to_name(output_stracks[i].classId);
                snprintf(objects_temp.objects[idx].name, 20, "%s", name);

                objects_temp.count++;
            }
        }

        pthread_mutex_lock(&g_para.mtx);
        g_para.objects_to_draw = objects_temp;

        pthread_mutex_unlock(&g_para.mtx);

        if(g_enable_sendpos) {
            int best_person_index = -1;
            int highest_score_index = -1;
            bool primary_found = false;
            float max_person_score = 0.0f;

            for (int i = 0; i < output_stracks.size(); i++) {
                if (!output_stracks[i].is_activated) continue;

                if (g_para.primary_target_id != -1 && output_stracks[i].track_id == g_para.primary_target_id) {
                    best_person_index = i;
                    break;
                }

                if (output_stracks[i].classId == 0 && output_stracks[i].score > max_person_score) {
                    max_person_score = output_stracks[i].score;
                    highest_score_index = i;
                }
            }

            if (best_person_index == -1) best_person_index = highest_score_index;

            pthread_mutex_lock(&g_para.mtx);
            if (best_person_index != -1)
            {
                const STrack& best_track = output_stracks[best_person_index];
                g_para.primary_target_id = best_track.track_id;

                /***
                // 获取卡尔曼滤波后的状态
                // mean[0]=cx, mean[1]=cy, mean[4]=vx, mean[5]=vy 
                // 注意：这里直接使用了跟踪器的状态作为参考坐标和速度，实际应用中可能需要根据具体情况进行调整或平滑处理
                // 在DetectLoopProcessAI线程中，每次处理新帧时都会更新这个参考坐标和速度，DetectProcessCameraHandle线程则使用这些数据进行线性预测并发送位置数据

                // 在DetectLoopProcessAI线程中，当检测到最优目标时，直接将跟踪器的状态作为参考坐标和速度进行保存。这样可以利用跟踪器的卡尔曼滤波结果，获得相对稳定的坐标和速度估计。
                // 什么是最优目标：在所有检测到的目标中，具有最高置信度且符合特定条件（如尺寸、位置等）的目标。
                g_para.ref_x = best_track.mean[0];// X坐标：跟踪器状态中的中心X坐标，经过卡尔曼滤波后的估计值，通常比原始检测结果更稳定。
                g_para.ref_y = best_track.mean[1];
                g_para.ref_vx = best_track.mean[4];
                g_para.ref_vy = best_track.mean[5];

                g_para.ref_frame_counter = current_process_frame_idx;
                g_para.stable_coord_valid = true;
                ***/
                
		g_para.ref_x = best_track.mean[0];
		g_para.ref_y = best_track.mean[1];
		
		float fps = 30.0f;
		g_para.ref_vx = best_track.mean[4] * fps;
		g_para.ref_vy = best_track.mean[5] * fps;
		g_para.ref_timestamp = get_now_time();
		g_para.stable_coord_valid = true;       
            }
            else
            {
                g_para.primary_target_id = -1;
                g_para.stable_coord_valid = false;
            }
            pthread_mutex_unlock(&g_para.mtx);
        }
    }
    printf("AI processing thread exited.\n");
    pthread_exit(0);
}

static int DetectProcessStart(void)
{
    if(!g_para.is_init) return DP_ERR_NOT_INIT;

    /***
    if(g_enable_detect) {
        g_para.ai_thread_on = 1;
        if(pthread_create(&g_para.ai_thread_pid, NULL, DetectLoopProcess, NULL))
        {
            g_para.ai_thread_on = 0;
            return DP_ERR_PTHREAD_CREATE;
        }
    }
    ***/
    
    if(g_enable_detect)
    {
       
       g_para.ai_thread_on = 1;
       if(pthread_create(&g_para.ai_thread_pid, NULL, DetectLoopProcess, NULL))
       {
           g_para.ai_thread_on = 0;
           return DP_ERR_PTHREAD_CREATE;
       }
       
       //发送线程
       g_para.send_thread_on = 1;
       
       if(pthread_create(&g_para.send_thread_pid, NULL, SendLoopProcess, NULL))
       {
         g_para.send_thread_on = 0;
         return DP_ERR_PTHREAD_CREATE; 
       }
           
    }
    
    
    
    
    return DP_SUCCESS;
}

static int DetectProcessStop(void)
{
    if(g_para.ai_thread_on) {
        g_para.ai_thread_on = 0;
        pthread_mutex_lock(&g_para.mtx);
        pthread_cond_broadcast(&g_para.cond);
        pthread_mutex_unlock(&g_para.mtx);
        pthread_join(g_para.ai_thread_pid, NULL);
    }
    
    
    //关闭发送线程
   if(g_para.send_thread_on)
   {
      g_para.send_thread_on = 0;

      pthread_join(g_para.send_thread_pid,NULL);
   }
    
    
    
    return DP_SUCCESS;
}

static int DetectProcessUninit(void)
{
    DetectProcessStop();
    if(g_para.is_init)
    {
        if(g_enable_detect) {
            deinit_post_process();

            if (g_para.app_ctx.input_attrs) {
                free(g_para.app_ctx.input_attrs);
                g_para.app_ctx.input_attrs = NULL;
            }
            if (g_para.app_ctx.output_attrs) {
                free(g_para.app_ctx.output_attrs);
                g_para.app_ctx.output_attrs = NULL;
            }

            for(uint32_t j = 0; j < g_para.app_ctx.io_num.n_output; j++) {
                if(g_para.rknn_output_bufs[j]) {
                    free(g_para.rknn_output_bufs[j]);
                    g_para.rknn_output_bufs[j] = NULL;
                }
            }

            if (g_para.rknn_in_buf.fd >= 0) {
                dma_buf_free(g_para.rknn_in_buf.size, &g_para.rknn_in_buf.fd, (void**)&g_para.rknn_in_buf.ptr);
            }

            if(g_para.app_ctx.rknn_ctx > 0) { rknn_destroy(g_para.app_ctx.rknn_ctx); g_para.app_ctx.rknn_ctx = 0; }
            c_RkRgaDeInit();
            g_glyph_cache.clear();
            pthread_mutex_destroy(&g_para.mtx);
            pthread_cond_destroy(&g_para.cond);

            if (g_para.font_atlas_fd >= 0) {
                // Size calculation must match alloc
                int atlas_size = g_para.font_atlas_w * g_para.font_atlas_h * 4;
                dma_buf_free(atlas_size, &g_para.font_atlas_fd, &g_para.font_atlas_ptr);
            }
        }

        g_para.is_init = 0;
    }
    
    g_eis.uninit();
    return DP_SUCCESS;
}

void handle_frame(uint16_t cmd, const std::vector<uint8_t>& data) {
    // printf("收到完整帧! 命令码: 0x%04X, 数据长度: %zu\n", cmd, data.size());

    // printf("数据内容: ");
    // for(int i = 0; i < data.size(); i++) {
    //     printf("%02X ", data[i]);
    // }
    // printf("\n");

    // if(cmd == 0x0002 && data.size() == 2 && data[0] == 0x03 && data[1] == 0x00) {
    //     if(!g_enable_sendpos) {
    //         g_enable_sendpos = true;
    //         printf("开启跟踪模式：启用位置发送功能。\n");
    //     }
    // }

    if(cmd == 0x0002 && data.size() == 2) {
        if(data[0] == 0x03 && data[1] == 0x00) {
            if(!g_enable_sendpos) {
                g_enable_sendpos = true;
                printf("开启跟踪模式：启用位置发送功能。\n");
            }
        } else {
            if(g_enable_sendpos) {
                g_enable_sendpos = false;
                printf("关闭跟踪模式：停止位置发送。\n");
            }
        }
    }
}

static int DetectProcessInit(int enable_detect, int src_w, int src_h)
{
    int status = 0;
    if(g_para.is_init) return DP_ERR_ALREADLY_INIT;

    g_enable_detect = (bool)enable_detect;

    g_para.src_w = src_w;
    g_para.src_h = src_h;

    g_para.hor_stride = MPP_ALIGN(g_para.src_w, 64);
    g_para.ver_stride = MPP_ALIGN(g_para.src_h, 16);
    size_t frame_size = g_para.hor_stride * g_para.ver_stride * 3 / 2;

    // printf("[DETECT_INIT] Calculated strides (hor: %u, ver: %u) -> frame_size: %zu\n",
    //        g_para.hor_stride, g_para.ver_stride, frame_size);

    if (!g_enable_detect) {
        printf("Detection disabled, skip some init.\n");
        g_enable_sendpos = false;
        g_para.is_init = 1;
        return DP_SUCCESS;
    }

    pthread_mutex_init(&g_para.mtx, NULL);
    pthread_cond_init(&g_para.cond, NULL);

    FILE *fp = fopen(MODEL_NAME, "rb");
    if(fp == NULL) { printf("fopen %s fail!\n", MODEL_NAME); return DP_ERR_FOEPN; }
    fseek(fp, 0, SEEK_END);
    int model_len = ftell(fp);
    unsigned char* model = (unsigned char*)malloc(model_len);
    if(!model) { printf("malloc model error\n"); fclose(fp); return DP_ERR_OUTOFMEM; }
    fseek(fp, 0, SEEK_SET);
    if(model_len != fread(model, 1, model_len, fp)) {
        printf("fread %s fail!\n", MODEL_NAME);
        free(model); fclose(fp); return DP_ERR_FREAD;
    }
    fclose(fp);

    status = rknn_init(&g_para.app_ctx.rknn_ctx, model, model_len, 0, NULL);
    free(model);
    if(status < 0) { printf("rknn_init fail! ret=%d\n", status); g_para.app_ctx.rknn_ctx = 0; return DP_ERR_RKNN_INIT; }

    status = rknn_query(g_para.app_ctx.rknn_ctx, RKNN_QUERY_IN_OUT_NUM, &g_para.app_ctx.io_num, sizeof(g_para.app_ctx.io_num));
    if (status != RKNN_SUCC) {
        printf("rknn_query fail! ret=%d\n", status);
        DetectProcessUninit(); return DP_ERR_RKNN_INIT;
    }
    // printf("model input num: %d, output num: %d\n", g_para.app_ctx.io_num.n_input, g_para.app_ctx.io_num.n_output);

    g_para.app_ctx.input_attrs = (rknn_tensor_attr *)malloc(g_para.app_ctx.io_num.n_input * sizeof(rknn_tensor_attr));
    g_para.app_ctx.output_attrs = (rknn_tensor_attr *)malloc(g_para.app_ctx.io_num.n_output * sizeof(rknn_tensor_attr));
    memset(g_para.app_ctx.input_attrs, 0, g_para.app_ctx.io_num.n_input * sizeof(rknn_tensor_attr));

    // 查询输入属性 (为了获取 model_width/height)
    for (int i = 0; i < g_para.app_ctx.io_num.n_input; i++) {
        g_para.app_ctx.input_attrs[i].index = i;
        status = rknn_query(g_para.app_ctx.rknn_ctx, RKNN_QUERY_INPUT_ATTR, &(g_para.app_ctx.input_attrs[i]), sizeof(rknn_tensor_attr));
        if (status != RKNN_SUCC) {
             printf("rknn_query input attr fail! ret=%d\n", status);
             DetectProcessUninit(); return DP_ERR_RKNN_INIT;
        }
    }
    // 设置模型维度
    if (g_para.app_ctx.input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        g_para.app_ctx.model_channel = g_para.app_ctx.input_attrs[0].dims[1];
        g_para.app_ctx.model_height = g_para.app_ctx.input_attrs[0].dims[2];
        g_para.app_ctx.model_width = g_para.app_ctx.input_attrs[0].dims[3];
    } else {
        g_para.app_ctx.model_height = g_para.app_ctx.input_attrs[0].dims[1];
        g_para.app_ctx.model_width = g_para.app_ctx.input_attrs[0].dims[2];
        g_para.app_ctx.model_channel = g_para.app_ctx.input_attrs[0].dims[3];
    }
    // printf("model input height=%d, width=%d, channel=%d\n",
    //    g_para.app_ctx.model_height, g_para.app_ctx.model_width, g_para.app_ctx.model_channel);

    for(uint32_t i = 0; i < g_para.app_ctx.io_num.n_output; i++) {
        g_para.app_ctx.output_attrs[i].index = i;
        status = rknn_query(g_para.app_ctx.rknn_ctx, RKNN_QUERY_OUTPUT_ATTR, &(g_para.app_ctx.output_attrs[i]), sizeof(rknn_tensor_attr));
        if (status != RKNN_SUCC) {
             printf("rknn_query output attr fail! ret=%d\n", status);
             DetectProcessUninit(); return DP_ERR_RKNN_INIT;
        }

        g_para.rknn_output_bufs[i] = malloc(g_para.app_ctx.output_attrs[i].size);
        g_para.rknn_outputs[i].want_float = 0;
        g_para.rknn_outputs[i].is_prealloc = 1;
        g_para.rknn_outputs[i].index = i;
        g_para.rknn_outputs[i].buf = g_para.rknn_output_bufs[i];
        g_para.rknn_outputs[i].size = g_para.app_ctx.output_attrs[i].size;
    }

    if (g_para.app_ctx.output_attrs[0].type == RKNN_TENSOR_INT8) {
        g_para.app_ctx.is_quant = true;
        // printf("Model is INT8 quantized.\n");
    } else {
        g_para.app_ctx.is_quant = false;
        // printf("Model is FP32.\n");
        for(uint32_t i = 0; i < g_para.app_ctx.io_num.n_output; i++) {
            g_para.rknn_outputs[i].want_float = 1;
        }
    }

    g_para.rknn_in_buf.size = DST_W * DST_H * DST_BPP / 8;
    status = dma_buf_alloc(DMA_HEAP_DMA32_UNCACHE_PATCH,
                           g_para.rknn_in_buf.size,
                           &g_para.rknn_in_buf.fd,
                           (void**)&g_para.rknn_in_buf.ptr);
    if (status < 0) {
        printf("ERROR: dma_buf_alloc for rknn_in_buf failed!\n");
        DetectProcessUninit(); return DP_ERR_OUTOFMEM;
    }

    c_RkRgaInit();

    // Initialize Font Atlas
    int char_count = FONT_8X16.size();
    printf("char_count=%d\n", char_count);

    int vir_scale = 0;
    if(FONT_SCALE < 9) vir_scale = 10;
    else vir_scale = FONT_SCALE;

    g_para.font_atlas_w = char_count * FONT_WIDTH * vir_scale;
    g_para.font_atlas_h = FONT_HEIGHT * vir_scale;
    int atlas_size = g_para.font_atlas_w * g_para.font_atlas_h * 4; // RGBA8888

    status = dma_buf_alloc(DMA_HEAP_DMA32_UNCACHE_PATCH, atlas_size, &g_para.font_atlas_fd, &g_para.font_atlas_ptr);
    if (status < 0) {
        printf("ERROR: dma_buf_alloc for font atlas failed!\n");
        DetectProcessUninit(); return DP_ERR_OUTOFMEM;
    }
    memset(g_para.font_atlas_ptr, 0, atlas_size);

    uint32_t* atlas_pixels = (uint32_t*)g_para.font_atlas_ptr;
    int char_idx = 0;

    for (const auto& pair : FONT_8X16) {
        char c = pair.first;
        const auto& char_bitmap = pair.second;
        
        // Store location in cache
        im_rect char_rect = {char_idx * FONT_WIDTH * vir_scale, 0, FONT_WIDTH * FONT_SCALE, FONT_HEIGHT * FONT_SCALE};
        g_glyph_cache[c] = char_rect;

        // Draw character into atlas
        const int atlas_w = g_para.font_atlas_w;
        for (int y = 0; y < FONT_HEIGHT; y++) {
            // 1. 预计算当前行在 atlas 中的垂直起始偏移
            int base_py = char_rect.y + y * FONT_SCALE;
            uint8_t row_bits = char_bitmap[y];
        
            for (int x = 0; x < FONT_WIDTH; x++) {
                // 2. 确定颜色（只计算一次）
                uint32_t color = ((row_bits >> (7 - x)) & 1) ? COLOR_GREEN : COLOR_BG;
            
                // 3. 预计算当前块在 atlas 中的水平起始坐标
                int base_px = char_rect.x + x * FONT_SCALE;
            
                for (int dy = 0; dy < FONT_SCALE; dy++) {
                    uint32_t* p_row = &atlas_pixels[(base_py + dy) * atlas_w + base_px];
                    for (int dx = 0; dx < FONT_SCALE; dx++) {
                        p_row[dx] = color;
                    }
                }

            }
        }
        char_idx++;
    }
    
    g_para.font_atlas_buf = wrapbuffer_fd(g_para.font_atlas_fd, g_para.font_atlas_w, g_para.font_atlas_h, RK_FORMAT_RGBA_8888);
    printf("Font Atlas initialized: %dx%d\n", g_para.font_atlas_w, g_para.font_atlas_h);

    status = init_post_process();
    if (status < 0) {
        printf("ERROR: init_post_process failed!\n");
        DetectProcessUninit(); return DP_ERR_RKNN_INIT;
    }

    g_para.tracker = BYTETracker(30, 60);
    
    SerialManager::GetInstance()->registerCallback(handle_frame);

    // printf("DetectProcess initialized successfully.\n");
    g_eis.init(src_w, src_h);
    g_para.is_init = 1;
    return 0;
}

static void DetectProcessSetCallback(detect_callback_t callback)
{
    g_para.callback = callback;
}

static DetectProcess_t g_DetectProcess =
{
    .init = DetectProcessInit,
    .uninit = DetectProcessUninit,
    .start = DetectProcessStart,
    .stop = DetectProcessStop,
    .camera_handle = DetectProcessCameraHandle,
    .SetCallback = DetectProcessSetCallback,
};

DetectProcess_t* GetDetectProcessInstance(void)
{
    return &g_DetectProcess;
}
