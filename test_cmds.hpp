#pragma once
#include <stdint.h>
// Sub-commands for debugging: test-uvc, test-yolo, test-motor, test-arm, test-bucket

int cmd_test_uvc(int uvc_index);
int cmd_test_yolo(const char* model_path, int uvc_index);
int cmd_test_motor(const char* uart_dev, int argc, char** argv);
int cmd_test_arm(const char* uart_dev, int argc, char** argv);
int cmd_test_bucket(int uvc_index);

// ── Bucket detection helper for main loop ────────────────────────────────────
// Runs HSV red detection on an already-decoded RGB frame.
// Returns true if a bucket was found; fills out fields.
struct BucketResult {
    bool  found;
    float area_ratio;  // red pixel area / frame area
    int   cx, cy;      // bounding-box center (frame coords)
    int   x, y, w, h;  // bounding box
};

bool detect_bucket_frame(const uint8_t* rgb, int width, int height,
                         BucketResult& out, int min_area = 1000);
