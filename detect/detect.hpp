#ifndef DETECT_HPP
#define DETECT_HPP

// Detection wrapper for rk3588 / RKNN YOLOv8.
// Mirrors the aka-sg2002/detect.hpp interface so tennis.cpp compiles unchanged.

#include <vector>
#include "yolov8.h"        // rknn_app_context_t, inference_yolov8_model
#include "postprocess.h"   // object_detect_result_list
#include "image_utils.h"   // image_buffer_t

// Detection box (same layout as sg2002 detect.hpp)
typedef struct {
    float x, y, w, h;
} box;

// Detection result (single object)
typedef struct {
    box   bbox;
    int   cls;
    float score;
    int   batch_idx;
} detection;

// ── Init / deinit ─────────────────────────────────────────────────────────────

// Load RKNN model.  Returns 0 on success.
int detect_init(const char* model_path, rknn_app_context_t* ctx);

// Release model resources.
void detect_deinit(rknn_app_context_t* ctx);

// ── Inference ─────────────────────────────────────────────────────────────────

// Run YOLOv8 inference on an RGB888 image already letterboxed to model_w×model_h.
// pad_x, pad_y, lbox_scale: letterbox parameters from decode_mjpeg,
//   used to map bbox coords back to original camera frame (orig_w × orig_h).
// Returns number of detections.
int detect_run(rknn_app_context_t* ctx,
               const uint8_t* rgb_data, int model_w, int model_h,
               int orig_w, int orig_h,
               int pad_x, int pad_y, float lbox_scale,
               float conf_thresh, float iou_thresh,
               std::vector<detection>& dets);

#endif // DETECT_HPP
