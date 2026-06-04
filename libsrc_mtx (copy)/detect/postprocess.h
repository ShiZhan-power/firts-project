#ifndef _RKNN_YOLOV8_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
//#include "common.h"
//#include "image_utils.h"
#include "opencv2/core/types.hpp"
#include "BYTETracker.h"

#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_NUM 5
#define NMS_THRESH 0.45
#define BOX_THRESH 0.6//0.1

// class rknn_app_context_t;

int init_post_process();
void deinit_post_process();
const char *coco_cls_to_name(int cls_id);
// int post_process(rknn_app_context_t *app_ctx, void *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, object_detect_result_list *od_results);
int post_process(rknn_app_context_t *app_ctx, void *outputs, int src_w, int src_h, float conf_threshold, float nms_threshold, std::vector<Object>& od_results);

void deinitPostProcess();
#endif //_RKNN_YOLOV8_DEMO_POSTPROCESS_H_
