// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka-sg2002/tennis.cpp - ported to rk3588 / RKNN YOLOv8
// Description: 追球抓取状态机 (两状态: chase / grab)
//   - UVC camera (MJPEG) → libjpeg-turbo decode → RKNN YOLOv8 inference
//   - Motor driver abstraction: UART (ESP32-C3) or PWM
//   - 差速脉冲转向 + 动态脉冲 + 太近后退
//   - Ctrl-C 安全退出

/*
cd ../aka-rk3588/ && ./build_rk3588.sh && cd - && mv ../aka-rk3588/build/tennis  .

*/

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <algorithm>
#include <vector>

// JPEG decode
#include <turbojpeg.h>

#include "logger.hpp"
#include "motor/motor.hpp"
#include "capture/uvc_capture.hpp"
#include "detect/detect.hpp"

// ── Build-time tunables ───────────────────────────────────────────────────────
#define ENABLE_DRAW_BBOX  0
#define ENABLE_SAVE_IMAGE 0
#define SKIP_FRAMES       0  // Process every Nth frame (0=process all)

// ── Segmented turn parameters ─────────────────────────────────────────────────
static const int TURN_SEGMENTS       = 3;  // Number of segments per turn
static const int TURN_SEGMENT_DELAY  = 25 * 1000;  // 25ms pause between segments

// ── Camera / model parameters ─────────────────────────────────────────────────
// Camera capture resolution: use a standard MJPEG mode most UVC cameras support.
// The JPEG is then decoded and nearest-neighbour scaled to MODEL_W × MODEL_H.
static const int FRAME_WIDTH  = 640;
static const int FRAME_HEIGHT = 480;
static const int MODEL_W      = 640;
static const int MODEL_H      = 640;

// ── Control parameters (tuned for rk3588 + UVC camera) ───────────────────────
static const float GRAB_AREA            = 0.40f;
static const float GRAB_AREA_MAX        = 0.55f;
static const int   CENTER_MARGIN        = 35;
static const float K_TURN_PULSE         = 1000.0f;  // Further reduced for close-range
static const int   TURN_PULSE_MAX       = 100 * 1000;  // Max 100ms for close targets
static const int   TURN_PULSE_MIN       = 20  * 1000;  // Min 20ms
// Speed unit is Motor [-100,100] → PWM via speed_scale=150.
// ESP32 PWM range is [-255,255], so 100%% → PWM 150 (≈60%% duty cycle).
// Minimum usable speed is 16 (PWM 24).
static const int   CHASE_SPEED          = 60;  // Forward speed (PWM 90) - reduced by 40%
static const int   TURN_SPEED           = 30;  // Turn in place (PWM 45)
static const int   IDLE_SPEED           = 17;  // Search spin (PWM 25.5)
static const int   GRAB_CONFIRM_THRESHOLD = 5;
static const int   BACKWARD_SPEED       = 40;  // Back up (PWM 60)
static const int   BACKWARD_PULSE_US    = 200 * 1000;
static const int   GRAB_LEFT_TURN_SPEED = 25;  // Alignment turns (PWM 38)
static const int   GRAB_LEFT_TURN_US    = 150 * 1000;
static const int   GRAB_LEFT_TURN_COUNT = 4;

// ── State machine ─────────────────────────────────────────────────────────────
enum RobotStatus {
    STATUS_CHASE_TENNIS,
    STATUS_GRAB_TENNIS,
};

static const char* status_name(RobotStatus s) {
    return s == STATUS_CHASE_TENNIS ? "chase" : "grab";
}

struct RobotState {
    RobotStatus status           = STATUS_CHASE_TENNIS;
    float       area_ratio       = 0;
    int         ball_cx          = 0;
    int         grab_confirm_count = 0;
};

// ── Globals for signal handler ────────────────────────────────────────────────
static Motor*      g_motor   = nullptr;
static UvcCapture* g_capture = nullptr;
static rknn_app_context_t* g_rknn_ctx = nullptr;
static int         g_saved_stderr = -1;
static int         g_devnull = -1;

static void cleanup_and_exit() {
    if (g_motor)   g_motor->standby();
    if (g_capture) g_capture->close();
    if (g_rknn_ctx) detect_deinit(g_rknn_ctx);
    // Restore stderr
    if (g_saved_stderr >= 0 && g_devnull >= 0) {
        dup2(g_saved_stderr, STDERR_FILENO);
    }
}

static void signal_handler(int /*sig*/) {
    cleanup_and_exit();
    exit(0);
}

// ── Segmented turn helper ─────────────────────────────────────────────────────
// Split a long turn into multiple short segments to reduce overshoot
// NON-BLOCKING: sends pulse command and returns immediately
static void segmented_turn(Motor& motor, int left_speed, int right_speed, int total_pulse_us)
{
    // Send turn command without blocking
    motor.drive(left_speed, right_speed);

    // Schedule stop after total_pulse_us using a timer (simplified: just drive)
    // For non-blocking, we let the motor run until next command updates it
    // TODO: implement proper timer-based stop if needed
}

// ── Timing helper ───────────────────────────────────────────────────────────────
static long elapsed_us(const struct timeval& start)
{
    struct timeval now;
    gettimeofday(&now, nullptr);
    return (now.tv_sec - start.tv_sec) * 1000000L + (now.tv_usec - start.tv_usec);
}

// ── Dynamic speed based on distance ────────────────────────────────────────────
// Calculate forward speed based on ball area ratio (closer = slower)
// area_ratio: 0.01 (far) to 0.55 (very close)
// Returns speed in range [MIN_APPROACH_SPEED, CHASE_SPEED]
static const int MIN_APPROACH_SPEED = 20;  // Minimum speed when very close (PWM 30)
static int get_approach_speed(float area_ratio)
{
    // Linear interpolation: speed decreases as area_ratio increases
    // At area_ratio = 0.01: return CHASE_SPEED
    // At area_ratio = GRAB_AREA (0.40): return MIN_APPROACH_SPEED
    float t = (area_ratio - 0.01f) / (GRAB_AREA - 0.01f);  // 0 to 1
    t = std::max(0.0f, std::min(1.0f, t));
    int speed = CHASE_SPEED - t * (CHASE_SPEED - MIN_APPROACH_SPEED);
    return std::max(MIN_APPROACH_SPEED, speed);
}

// ── JPEG decode helper ────────────────────────────────────────────────────────
// Optimized: Use libjpeg-turbo scaled decode + direct fill
// Timing outputs (optional, pass nullptr to skip)
static int decode_mjpeg(const uint8_t* jpeg_data, size_t jpeg_len,
                        uint8_t* rgb_out, int out_w, int out_h,
                        int* pad_x = nullptr, int* pad_y = nullptr,
                        float* scale_out = nullptr,
                        long* t_header = nullptr, long* t_decomp = nullptr, long* t_copy = nullptr)
{
    struct timeval t0, t1, t2;
    long header_us = 0, decomp_us = 0, copy_us = 0;

    tjhandle tj = tjInitDecompress();
    if (!tj) return -1;

    gettimeofday(&t0, nullptr);
    int w, h, subsamp, colorspace;
    if (tjDecompressHeader3(tj, jpeg_data, jpeg_len,
                            &w, &h, &subsamp, &colorspace) < 0) {
        tjDestroy(tj); return -1;
    }
    gettimeofday(&t1, nullptr);
    header_us = elapsed_us(t0);

    // Letterbox: scale uniformly so image fits in out_w × out_h
    float scale = std::min((float)out_w / w, (float)out_h / h);
    int new_w = (int)(w * scale + 0.5f);
    int new_h = (int)(h * scale + 0.5f);
    int off_x = (out_w - new_w) / 2;
    int off_y = (out_h - new_h) / 2;
    if (pad_x)     *pad_x = off_x;
    if (pad_y)     *pad_y = off_y;
    if (scale_out) *scale_out = scale;

    // Fill background with 114
    memset(rgb_out, 114, out_w * out_h * 3);

    // Use libjpeg-turbo scaled decode (fast) + direct fill to letterbox position
    // Decode to temporary scaled buffer
    uint8_t* tmp = static_cast<uint8_t*>(malloc(new_w * new_h * 3));
    if (!tmp) { tjDestroy(tj); return -1; }

    gettimeofday(&t0, nullptr);
    int ret = tjDecompress2(tj, jpeg_data, jpeg_len,
                            tmp, new_w, 0, new_h, TJPF_RGB, TJFLAG_FASTDCT);
    tjDestroy(tj);
    gettimeofday(&t1, nullptr);
    decomp_us = elapsed_us(t0);

    if (ret < 0) { free(tmp); return -1; }

    // Copy scaled image to letterbox position (single pass)
    gettimeofday(&t0, nullptr);
    for (int y = 0; y < new_h; y++) {
        const uint8_t* src = tmp + y * new_w * 3;
        uint8_t* dst = rgb_out + ((y + off_y) * out_w + off_x) * 3;
        memcpy(dst, src, new_w * 3);
    }
    gettimeofday(&t1, nullptr);
    copy_us = elapsed_us(t0);

    free(tmp);

    if (t_header) *t_header = header_us;
    if (t_decomp) *t_decomp = decomp_us;
    if (t_copy)   *t_copy   = copy_us;
    return 0;
}

// ── Save raw JPEG to file ─────────────────────────────────────────────────────
static int save_jpeg(const char* path, const uint8_t* data, size_t len)
{
    FILE* f = fopen(path, "wb");
    if (!f) { LOGE("Cannot open %s for writing", path); return -1; }
    fwrite(data, 1, len, f);
    fclose(f);
    LOGI("Saved %zu bytes → %s", len, path);
    return 0;
}

// ── Draw bounding box on RGB buffer (in-place) ───────────────────────────────
static void draw_box_rgb(uint8_t* rgb, int img_w, int img_h,
                         int x, int y, int w, int h,
                         uint8_t r, uint8_t g, uint8_t b)
{
    auto clamp = [](int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); };
    int x0 = clamp(x, 0, img_w - 1);
    int y0 = clamp(y, 0, img_h - 1);
    int x1 = clamp(x + w - 1, 0, img_w - 1);
    int y1 = clamp(y + h - 1, 0, img_h - 1);
    // top / bottom
    for (int px = x0; px <= x1; px++) {
        uint8_t* t = rgb + (y0 * img_w + px) * 3;
        uint8_t* bot = rgb + (y1 * img_w + px) * 3;
        t[0] = bot[0] = r; t[1] = bot[1] = g; t[2] = bot[2] = b;
    }
    // left / right
    for (int py = y0; py <= y1; py++) {
        uint8_t* l = rgb + (py * img_w + x0) * 3;
        uint8_t* ri = rgb + (py * img_w + x1) * 3;
        l[0] = ri[0] = r; l[1] = ri[1] = g; l[2] = ri[2] = b;
    }
}

// ── Save RGB888 as JPEG ───────────────────────────────────────────────────────
static int save_rgb_as_jpeg(const char* path, const uint8_t* rgb, int w, int h)
{
    tjhandle tj = tjInitCompress();
    if (!tj) { LOGE("tjInitCompress failed"); return -1; }
    unsigned char* buf = nullptr;
    unsigned long  buf_len = 0;
    int ret = tjCompress2(tj, rgb, w, 0, h, TJPF_RGB,
                          &buf, &buf_len, TJSAMP_420, 90, TJFLAG_FASTDCT);
    tjDestroy(tj);
    if (ret < 0) { LOGE("tjCompress2 failed"); return -1; }
    FILE* f = fopen(path, "wb");
    if (!f) { tjFree(buf); LOGE("Cannot open %s for writing", path); return -1; }
    fwrite(buf, 1, buf_len, f);
    fclose(f);
    tjFree(buf);
    LOGI("Saved %lu bytes → %s", buf_len, path);
    return 0;
}

// ── test-uvc: capture one frame and save as JPEG ─────────────────────────────
static int cmd_test_uvc(int uvc_index)
{
    UvcCapture capture;
    if (capture.open(uvc_index, FRAME_WIDTH, FRAME_HEIGHT, 30) != 0) {
        LOGE("Failed to open UVC device %d", uvc_index);
        return 1;
    }
    LOGI("Camera opened (%dx%d), waiting for frame...", FRAME_WIDTH, FRAME_HEIGHT);

    const size_t BUF = 1024 * 1024;
    uint8_t* buf = static_cast<uint8_t*>(malloc(BUF));

    // Skip first 20 frames to let AE/AWB settle
    LOGI("Warming up camera (skip 20 frames)...");
    for (int i = 0; i < 20; i++)
        capture.getFrame(buf, BUF, 500);

    int len = capture.getFrame(buf, BUF, 2000);
    capture.close();

    if (len <= 0) {
        LOGE("No frame received (timeout)");
        free(buf);
        return 1;
    }
    LOGI("Got MJPEG frame: %d bytes", len);
    save_jpeg("capture.jpg", buf, len);
    free(buf);
    return 0;
}

// ── test-yolo: capture one frame, run inference, draw bboxes, save result ────
static int cmd_test_yolo(const char* model_path, int uvc_index)
{
    // Open camera
    UvcCapture capture;
    if (capture.open(uvc_index, FRAME_WIDTH, FRAME_HEIGHT, 30) != 0) {
        LOGE("Failed to open UVC device %d", uvc_index);
        return 1;
    }

    // Load model
    rknn_app_context_t rknn_ctx;
    if (detect_init(model_path, &rknn_ctx) != 0) {
        LOGE("Failed to load model: %s", model_path);
        capture.close();
        return 1;
    }
    int model_w = rknn_ctx.model_width;
    int model_h = rknn_ctx.model_height;
    LOGI("Model input %dx%d", model_w, model_h);

    // Capture frame
    const size_t MJPEG_BUF = 1024 * 1024;
    uint8_t* mjpeg_buf = static_cast<uint8_t*>(malloc(MJPEG_BUF));

    // Skip first 20 frames to let AE/AWB settle
    LOGI("Warming up camera (skip 20 frames)...");
    for (int i = 0; i < 20; i++)
        capture.getFrame(mjpeg_buf, MJPEG_BUF, 500);

    LOGI("Waiting for frame...");
    int jpeg_len = capture.getFrame(mjpeg_buf, MJPEG_BUF, 2000);
    capture.close();

    if (jpeg_len <= 0) {
        LOGE("No frame received");
        free(mjpeg_buf); detect_deinit(&rknn_ctx); return 1;
    }
    LOGI("Got MJPEG frame: %d bytes", jpeg_len);
    save_jpeg("capture.jpg", mjpeg_buf, jpeg_len);  // also save raw capture

    // Decode to RGB letterboxed to model resolution, get letterbox params
    uint8_t* rgb_model = static_cast<uint8_t*>(malloc(model_w * model_h * 3));
    int lb_pad_x = 0, lb_pad_y = 0;
    float lb_scale = 1.0f;
    if (decode_mjpeg(mjpeg_buf, jpeg_len, rgb_model, model_w, model_h,
                     &lb_pad_x, &lb_pad_y, &lb_scale) != 0) {
        LOGE("JPEG decode failed");
        free(mjpeg_buf); free(rgb_model); detect_deinit(&rknn_ctx); return 1;
    }
    free(mjpeg_buf);
    LOGI("Letterbox: scale=%.4f pad=(%d,%d)", lb_scale, lb_pad_x, lb_pad_y);

    // Decode again at native resolution for display (no letterbox needed for display)
    // Reuse: extract the letterboxed region back to FRAME_WIDTH×FRAME_HEIGHT
    uint8_t* rgb_disp = static_cast<uint8_t*>(malloc(FRAME_WIDTH * FRAME_HEIGHT * 3));
    for (int y = 0; y < FRAME_HEIGHT; y++) {
        int sy = (int)(y * lb_scale) + lb_pad_y;
        sy = std::max(0, std::min(sy, model_h - 1));
        for (int x = 0; x < FRAME_WIDTH; x++) {
            int sx = (int)(x * lb_scale) + lb_pad_x;
            sx = std::max(0, std::min(sx, model_w - 1));
            const uint8_t* s = rgb_model + (sy * model_w + sx) * 3;
            uint8_t*       d = rgb_disp  + (y  * FRAME_WIDTH + x) * 3;
            d[0] = s[0]; d[1] = s[1]; d[2] = s[2];
        }
    }

    // Inference
    std::vector<detection> dets;
    {
        int saved_stdout = dup(STDOUT_FILENO);
        int devnull = open("/dev/null", O_WRONLY);
        dup2(devnull, STDOUT_FILENO);
        close(devnull);
        detect_run(&rknn_ctx, rgb_model, model_w, model_h,
                   FRAME_WIDTH, FRAME_HEIGHT, lb_pad_x, lb_pad_y, lb_scale,
                   0.5f, 0.45f, dets, nullptr, nullptr, nullptr, nullptr);
        fflush(stdout);
        dup2(saved_stdout, STDOUT_FILENO);
        close(saved_stdout);
    }
    detect_deinit(&rknn_ctx);
    free(rgb_model);

    LOGI("Detections: %d", (int)dets.size());
    for (int i = 0; i < (int)dets.size(); i++) {
        const detection& d = dets[i];
        LOGI("  [%d] cls=%d score=%.3f bbox=(%.0f,%.0f,%.0f,%.0f)",
             i, d.cls, d.score, d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h);
        // bbox.x/y is center; draw_box_rgb expects top-left corner
        draw_box_rgb(rgb_disp, FRAME_WIDTH, FRAME_HEIGHT,
                     (int)(d.bbox.x - d.bbox.w * 0.5f),
                     (int)(d.bbox.y - d.bbox.h * 0.5f),
                     (int)d.bbox.w, (int)d.bbox.h,
                     0, 255, 0);
    }

    save_rgb_as_jpeg("result.jpg", rgb_disp, FRAME_WIDTH, FRAME_HEIGHT);
    free(rgb_disp);
    return 0;
}

// ── test-motor: test the Motor abstraction layer ─────────────────────────────
static int cmd_test_motor(const char* uart_dev, int argc, char** argv)
{
    // Parse optional speed from remaining argv
    // argv here is the full argv; scan from index 3 onward
    int  speed = 0;
    for (int i = 3; i < argc; i++) {
        if (strncmp(argv[i], "speed=", 6) == 0) { speed = atoi(argv[i] + 6); }
    }

    printf("=== test-motor: %s ===\n", uart_dev);
    if (speed > 0) {
        printf("Manual mode: speed=%d\n", speed);
    }

    Motor motor(MotorDriverType::UART, uart_dev);

    if (speed > 0) {
        // Manual mode: hold speed for 5s then stop
        printf("Forward %d%% for 5s...\n", speed);
        motor.forward(speed);
        sleep(5);
        motor.standby();
    } else {
        // Auto sequence
        printf("\n[1] Forward 50%% for 2s\n");
        motor.forward(50);
        sleep(2);

        printf("\n[2] Stop\n");
        motor.standby();
        sleep(1);

        printf("\n[3] Backward 50%% for 2s\n");
        motor.backward(50);
        sleep(2);

        printf("\n[4] Stop\n");
        motor.standby();
        sleep(1);

        printf("\n[5] Turn left 50%% for 2s\n");
        motor.left(50);
        sleep(2);

        printf("\n[6] Stop\n");
        motor.standby();
        sleep(1);

        printf("\n[7] Turn right 50%% for 2s\n");
        motor.right(50);
        sleep(2);

        printf("\n[8] Stop\n");
        motor.standby();
    }

    printf("\n=== test-motor DONE ===\n");
    return 0;
}

// ── Usage ─────────────────────────────────────────────────────────────────────
static void usage(const char* prog) {
    LOGI("Usage:");
    LOGI("  %s <model.rknn> [uart_dev] [uvc_device_index]", prog);
    LOGI("  Example: %s tennis.rknn /dev/ttyS3 0", prog);
    LOGI("  uart_dev default: /dev/ttyS3");
    LOGI("  uvc_device_index default: 0");
    LOGI("");
    LOGI("  %s test-uvc [uvc_device_index]          -- capture one frame → capture.jpg", prog);
    LOGI("  %s test-yolo <model.rknn> [uvc_index]   -- detect one frame  → result.jpg", prog);
    LOGI("  %s test-motor [uart_dev] [speed=N]", prog);
    LOGI("      -- no speed: run auto sequence");
    LOGI("      -- with speed: hold that speed (%%) for 5s then stop");
    LOGI("      -- example: %s test-motor /dev/ttyUSB0 speed=50", prog);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    if (argc < 2) { usage(argv[0]); return 1; }

    // ── Sub-commands ──────────────────────────────────────────────────────────
    if (strcmp(argv[1], "test-uvc") == 0) {
        int idx = (argc >= 3) ? atoi(argv[2]) : 0;
        return cmd_test_uvc(idx);
    }
    if (strcmp(argv[1], "test-yolo") == 0) {
        if (argc < 3) {
            LOGE("test-yolo requires <model.rknn>");
            usage(argv[0]); return 1;
        }
        int idx = (argc >= 4) ? atoi(argv[3]) : 0;
        return cmd_test_yolo(argv[2], idx);
    }
    if (strcmp(argv[1], "test-motor") == 0) {
        const char* dev = (argc >= 3) ? argv[2] : "/dev/ttyUSB0";
        return cmd_test_motor(dev, argc, argv);
    }

    const char* model_path  = argv[1];
    const char* uart_dev    = (argc >= 3) ? argv[2] : "/dev/ttyS3";
    int         uvc_index   = (argc >= 4) ? atoi(argv[3]) : 0;

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    // ── Motor ─────────────────────────────────────────────────────────────────
    Motor motor(MotorDriverType::UART, uart_dev);
    g_motor = &motor;
    LOGI("Motor initialized (UART %s)", uart_dev);

    // ── Camera ────────────────────────────────────────────────────────────────
    UvcCapture capture;
    g_capture = &capture;
    if (capture.open(uvc_index, FRAME_WIDTH, FRAME_HEIGHT, 30) != 0) {
        LOGE("Failed to open UVC device %d", uvc_index);
        return 1;
    }
    LOGI("Camera opened (%dx%d)", FRAME_WIDTH, FRAME_HEIGHT);

    // ── RKNN model ────────────────────────────────────────────────────────────
    rknn_app_context_t rknn_ctx;
    g_rknn_ctx = &rknn_ctx;
    if (detect_init(model_path, &rknn_ctx) != 0) {
        LOGE("Failed to load model: %s", model_path);
        return 1;
    }
    int model_w = rknn_ctx.model_width;
    int model_h = rknn_ctx.model_height;
    LOGI("Model input %dx%d", model_w, model_h);

    // Suppress detect.cpp stderr spam (one-time redirect)
    g_devnull = open("/dev/null", O_WRONLY);
    g_saved_stderr = dup(STDERR_FILENO);
    dup2(g_devnull, STDERR_FILENO);

    // Buffers
    const size_t MJPEG_BUF = 1024 * 1024; // 1 MB should be enough for 640×480 MJPEG
    uint8_t* mjpeg_buf = static_cast<uint8_t*>(malloc(MJPEG_BUF));
    uint8_t* rgb_buf   = static_cast<uint8_t*>(malloc(model_w * model_h * 3));
    if (!mjpeg_buf || !rgb_buf) { LOGE("OOM"); return 1; }

    // ── State machine ─────────────────────────────────────────────────────────
    RobotState robot;

    int frame_idx   = 0;
    int detect_idx  = 0;
    long total_us   = 0;
    int  frame_cnt  = 0;

    // Timing stats - only for processed frames
    int processed_cnt = 0;
    long t_decode = 0, t_decode_header = 0, t_decode_decomp = 0, t_decode_copy = 0;
    long t_inference = 0, t_in_input = 0, t_in_run = 0, t_in_output = 0, t_in_post = 0;
    long t_control = 0;

    while (true) {
        struct timeval t_start, t_end, t_stage;
        gettimeofday(&t_start, nullptr);
        frame_idx++;

        // 1. Grab MJPEG frame (includes waiting for next frame)
        int jpeg_len = capture.getFrame(mjpeg_buf, MJPEG_BUF, 200);
        if (jpeg_len <= 0) {
            // Restore stderr for log message
            dup2(g_saved_stderr, STDERR_FILENO);
            LOGW("[Frame %d] No frame (timeout)", frame_idx);
            dup2(g_devnull, STDERR_FILENO);
            usleep(10000);
            continue;
        }

        // Frame skip: only process every Nth frame
#if SKIP_FRAMES > 0
        if ((frame_idx % (SKIP_FRAMES + 1)) != 0) {
            continue;
        }
#endif
        detect_idx++;
        processed_cnt++;

        // 2. Decode MJPEG → RGB letterboxed to model input, capture letterbox params
        long th = 0, td = 0, tc = 0;
        int lb_pad_x = 0, lb_pad_y = 0;
        float lb_scale = 1.0f;
        if (decode_mjpeg(mjpeg_buf, jpeg_len, rgb_buf, model_w, model_h,
                         &lb_pad_x, &lb_pad_y, &lb_scale, &th, &td, &tc) != 0) {
            continue;
        }
        t_decode_header += th;
        t_decode_decomp += td;
        t_decode_copy   += tc;
        t_decode += (th + td + tc);

        // 3. Inference
        long ti = 0, tr = 0, to = 0, tp = 0;
        std::vector<detection> dets;
        detect_run(&rknn_ctx, rgb_buf, model_w, model_h,
                   FRAME_WIDTH, FRAME_HEIGHT, lb_pad_x, lb_pad_y, lb_scale,
                   0.5f, 0.45f, dets, &ti, &tr, &to, &tp);
        t_in_input  += ti;
        t_in_run    += tr;
        t_in_output += to;
        t_in_post   += tp;
        t_inference += (ti + tr + to + tp);

        // 4. State machine / motor control
        gettimeofday(&t_stage, nullptr);
        const int center = FRAME_WIDTH / 2;

        if (!dets.empty()) {
            // Select largest (nearest) detection
            int best = 0;
            for (int i = 1; i < (int)dets.size(); i++) {
                if (dets[i].bbox.w * dets[i].bbox.h >
                    dets[best].bbox.w * dets[best].bbox.h)
                    best = i;
            }

            const box& b = dets[best].bbox;
            float img_area  = FRAME_WIDTH * FRAME_HEIGHT;
            float area_ratio = (b.w * b.h) / img_area;
            int   ball_cx    = static_cast<int>(b.x);

            robot.area_ratio = area_ratio;
            robot.ball_cx    = ball_cx;

            LOGD("[DETECT] area=%.3f cx=%d conf=%.3f status=%s",
                 area_ratio, ball_cx, dets[best].score, status_name(robot.status));

            int  offset   = ball_cx - center;
            bool centered = abs(offset) <= CENTER_MARGIN;

            int pulse_us = std::max(TURN_PULSE_MIN,
                           std::min(TURN_PULSE_MAX,
                                    (int)(K_TURN_PULSE * area_ratio * 1000)));

            if (area_ratio >= GRAB_AREA && centered) {
                robot.grab_confirm_count++;
                LOGD("[GRAB] confirm %d/%d (area=%.3f)",
                     robot.grab_confirm_count, GRAB_CONFIRM_THRESHOLD, area_ratio);

                if (area_ratio >= GRAB_AREA_MAX) {
                    LOGD("[GRAB] Too close, backing up");
                    motor.backward(BACKWARD_SPEED);
                    usleep(BACKWARD_PULSE_US);
                    motor.standby();
                } else {
                    motor.standby();
                }

                if (robot.grab_confirm_count >= GRAB_CONFIRM_THRESHOLD) {
                    if (area_ratio >= GRAB_AREA_MAX) {
                        motor.backward(BACKWARD_SPEED);
                        usleep(BACKWARD_PULSE_US);
                        motor.standby();
                    }

                    // 爪子偏右，连续左转补偿
                    for (int i = 0; i < GRAB_LEFT_TURN_COUNT; i++) {
                        LOGD("[GRAB] Left turn compensation %d/%d", i + 1, GRAB_LEFT_TURN_COUNT);
                        motor.drive(-GRAB_LEFT_TURN_SPEED, GRAB_LEFT_TURN_SPEED);
                        usleep(GRAB_LEFT_TURN_US);
                        motor.standby();
                        usleep(100 * 1000);
                    }

                    // TODO: trigger arm/gripper (no arm module on rk3588 yet)
                    LOGI("[GRAB] Grab confirmed - trigger gripper here");
                    usleep(2000 * 1000);

                    robot.grab_confirm_count = 0;
                    robot.status = STATUS_CHASE_TENNIS;
                }

            } else if (area_ratio >= GRAB_AREA && !centered) {
                robot.grab_confirm_count = 0;
                LOGD("[ALIGN] Aligning cx=%d offset=%d pulse=%dms",
                     ball_cx, offset, pulse_us / 1000);
                if (offset < 0)
                    segmented_turn(motor, -TURN_SPEED, TURN_SPEED, pulse_us);
                else
                    segmented_turn(motor, TURN_SPEED, -TURN_SPEED, pulse_us);

            } else {
                // Chase
                robot.grab_confirm_count = 0;
                robot.status = STATUS_CHASE_TENNIS;

                if (!centered) {
                    if (offset < 0) {
                        LOGD("[MOTOR] TURN LEFT cx=%d pulse=%dms", ball_cx, pulse_us / 1000);
                        segmented_turn(motor, -TURN_SPEED, TURN_SPEED, pulse_us);
                    } else {
                        LOGD("[MOTOR] TURN RIGHT cx=%d pulse=%dms", ball_cx, pulse_us / 1000);
                        segmented_turn(motor, TURN_SPEED, -TURN_SPEED, pulse_us);
                    }
                } else {
                    // Dynamic speed: slower when closer
                    int approach_speed = get_approach_speed(area_ratio);
                    LOGD("[MOTOR] FORWARD area=%.3f speed=%d", area_ratio, approach_speed);
                    motor.forward(approach_speed);
                }
            }

        } else {
            LOGD("[DETECT] No ball, searching...");
            robot.grab_confirm_count = 0;
            robot.status = STATUS_CHASE_TENNIS;
            motor.drive(IDLE_SPEED, -IDLE_SPEED); // 原地右转搜索
        }
        t_control += elapsed_us(t_stage);

        // FPS — only print when a ball is detected
        gettimeofday(&t_end, nullptr);
        long frame_us = (t_end.tv_sec  - t_start.tv_sec)  * 1000000L
                      + (t_end.tv_usec - t_start.tv_usec);
        frame_cnt++;
        total_us += frame_us;

        if (!dets.empty()) {
            // Restore stderr for printf
            dup2(g_saved_stderr, STDERR_FILENO);
            long avg_process = (t_decode + t_inference + t_control) / processed_cnt;
            long wait_us = frame_us - avg_process;
            if (wait_us < 0) wait_us = 0;  // 处理速度快于摄像头时，等待时间为0
            printf("\n[TIMING] FPS=%.1f (%.1fms)  frame=%d proc=%d\n", 1e6f / frame_us, frame_us / 1000.0f, frame_idx, processed_cnt);
            printf("  wait_cam: %.2fms  (等待下一帧)\n", wait_us / 1000.0);
            printf("  dec     : %.2fms  (hdr=%.2f decomp=%.2f copy=%.2f)\n",
                   t_decode / 1000.0 / processed_cnt,
                   t_decode_header / 1000.0 / processed_cnt,
                   t_decode_decomp / 1000.0 / processed_cnt,
                   t_decode_copy / 1000.0 / processed_cnt);
            printf("  inf     : %.2fms  (in=%.2f run=%.2f out=%.2f post=%.2f)\n",
                   t_inference / 1000.0 / processed_cnt,
                   t_in_input / 1000.0 / processed_cnt,
                   t_in_run / 1000.0 / processed_cnt,
                   t_in_output / 1000.0 / processed_cnt,
                   t_in_post / 1000.0 / processed_cnt);
            printf("  ctrl    : %.2fms\n", t_control / 1000.0 / processed_cnt);
            printf("  process : %.2fms  (dec+inf+ctrl)\n", avg_process / 1000.0);
            // Re-suppress stderr
            dup2(g_devnull, STDERR_FILENO);
        }
    }

    free(mjpeg_buf);
    free(rgb_buf);
    cleanup_and_exit();
    return 0;
}
