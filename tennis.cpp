// Author: Zhihang Shao <dio_ro@outlook.com>
// Source: aka-sg2002/tennis.cpp - ported to rk3588 / RKNN YOLOv8
// Description: 追球抓取状态机 (两状态: chase / grab)
//   - UVC camera (MJPEG) → libjpeg-turbo decode → RKNN YOLOv8 inference
//   - Motor driver abstraction: UART (ESP32-C3) or PWM
//   - 差速脉冲转向 + 动态脉冲 + 太近后退
//   - Ctrl-C 安全退出

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
#include <termios.h>
#include <sys/select.h>
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
static const float K_TURN_PULSE         = 5000.0f;
static const int   TURN_PULSE_MAX       = 500 * 1000;
static const int   TURN_PULSE_MIN       = 75  * 1000;
// Speed unit is Motor [-100,100] → RPM via speed_scale=250.
// Measured: min usable ≈ 180 RPM (unit=72), full speed = 250 RPM (unit=100).
static const int   CHASE_SPEED          = 100; // 250 RPM - full forward
static const int   TURN_SPEED           = 80;  // 200 RPM - turn in place
static const int   IDLE_SPEED           = 75;  // 188 RPM - search spin (just above min)
static const int   GRAB_CONFIRM_THRESHOLD = 5;
static const int   BACKWARD_SPEED       = 75;  // 188 RPM - back up
static const int   BACKWARD_PULSE_US    = 200 * 1000;
static const int   GRAB_LEFT_TURN_SPEED = 75;  // 188 RPM - alignment turns
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

static void cleanup_and_exit() {
    if (g_motor)   g_motor->standby();
    if (g_capture) g_capture->close();
    if (g_rknn_ctx) detect_deinit(g_rknn_ctx);
}

static void signal_handler(int /*sig*/) {
    cleanup_and_exit();
    exit(0);
}

// ── JPEG decode helper ────────────────────────────────────────────────────────
// Decode MJPEG → native RGB, then letterbox-pad to (out_w × out_h).
// pad_x, pad_y: pixels added on each side; scale: native→model scale factor.
// Returns 0 on success.
static int decode_mjpeg(const uint8_t* jpeg_data, size_t jpeg_len,
                        uint8_t* rgb_out, int out_w, int out_h,
                        int* pad_x = nullptr, int* pad_y = nullptr,
                        float* scale_out = nullptr)
{
    tjhandle tj = tjInitDecompress();
    if (!tj) return -1;

    int w, h, subsamp, colorspace;
    if (tjDecompressHeader3(tj, jpeg_data, jpeg_len,
                            &w, &h, &subsamp, &colorspace) < 0) {
        tjDestroy(tj); return -1;
    }

    uint8_t* tmp = static_cast<uint8_t*>(malloc(w * h * 3));
    if (!tmp) { tjDestroy(tj); return -1; }
    int ret = tjDecompress2(tj, jpeg_data, jpeg_len,
                            tmp, w, 0, h, TJPF_RGB, TJFLAG_FASTDCT);
    tjDestroy(tj);
    if (ret < 0) { free(tmp); return -1; }

    // Letterbox: scale uniformly so image fits in out_w × out_h, pad with 114
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

    // Nearest-neighbour scale into the padded region
    for (int y = 0; y < new_h; y++) {
        int src_y = (int)(y / scale);
        if (src_y >= h) src_y = h - 1;
        for (int x = 0; x < new_w; x++) {
            int src_x = (int)(x / scale);
            if (src_x >= w) src_x = w - 1;
            const uint8_t* src = tmp + (src_y * w + src_x) * 3;
            uint8_t*       dst = rgb_out + ((y + off_y) * out_w + (x + off_x)) * 3;
            dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
        }
    }

    free(tmp);
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
                   0.5f, 0.45f, dets);
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

// ── test-motor: exercise the ESP32 UART motor controller ─────────────────────

// Raw UART helpers (mirrors test_uart.py logic exactly)
namespace raw_uart {

static uint8_t calc_chk(uint8_t cmd, uint8_t len, const uint8_t* p) {
    uint8_t c = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) c ^= p[i];
    return c;
}

static bool open_port(const char* dev, int& fd) {
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return false; }
    struct termios tty{};
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB|PARODD|CSTOPB|CRTSCTS);
    tty.c_iflag &= ~(IXON|IXOFF|IXANY|IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &tty);
    return true;
}

static bool read_byte(int fd, uint8_t& b, int timeout_ms = 200) {
    fd_set fds; FD_ZERO(&fds); FD_SET(fd, &fds);
    struct timeval tv = { timeout_ms/1000, (timeout_ms%1000)*1000 };
    if (select(fd+1, &fds, nullptr, nullptr, &tv) <= 0) return false;
    return read(fd, &b, 1) == 1;
}

static void send_frame(int fd, uint8_t cmd, const uint8_t* payload = nullptr, uint8_t len = 0) {
    uint8_t buf[64];
    buf[0]=0xAA; buf[1]=0x55; buf[2]=cmd; buf[3]=len;
    if (len && payload) memcpy(&buf[4], payload, len);
    buf[4+len] = calc_chk(cmd, len, payload ? payload : (const uint8_t*)"");
    ssize_t w = write(fd, buf, 5+len); (void)w;
}

// Returns true and fills cmd_out/payload_out/len_out on success
static bool recv_frame(int fd, uint8_t& cmd_out, uint8_t* payload_out, uint8_t& len_out,
                        int timeout_ms = 500) {
    // Wait for 0xAA 0x55
    int deadline = timeout_ms;
    while (deadline > 0) {
        uint8_t b;
        if (!read_byte(fd, b, 20)) { deadline -= 20; continue; }
        if (b != 0xAA) continue;
        if (!read_byte(fd, b, 50)) return false;
        if (b == 0x55) break;
    }
    if (deadline <= 0) return false;
    uint8_t cmd, len;
    if (!read_byte(fd, cmd, 100)) return false;
    if (!read_byte(fd, len, 100)) return false;
    uint8_t payload[32]{};
    for (uint8_t i = 0; i < len && i < 32; i++)
        if (!read_byte(fd, payload[i], 100)) return false;
    uint8_t chk;
    if (!read_byte(fd, chk, 100)) return false;
    if (calc_chk(cmd, len, payload) != chk) {
        printf("  [WARN] checksum mismatch\n"); return false;
    }
    cmd_out = cmd; len_out = len;
    if (payload_out) memcpy(payload_out, payload, len);
    return true;
}

static const char* resp_name(uint8_t cmd) {
    switch(cmd) {
        case 0x80: return "ACK";
        case 0x81: return "NACK";
        case 0x90: return "RPM_DATA";
        case 0x91: return "STATUS";
        default:   return "???";
    }
}
static const char* err_name(uint8_t e) {
    switch(e) {
        case 0x01: return "WRONG_STATE";
        case 0x02: return "BAD_CHECKSUM";
        case 0x03: return "INVALID_PARAM";
        case 0x04: return "UNKNOWN_CMD";
        default:   return "?";
    }
}
static const char* state_name(uint8_t s) {
    switch(s) {
        case 0: return "UNINIT"; case 1: return "IDLE";
        case 2: return "READY";  case 3: return "RUNNING";
        case 4: return "ERROR";  default: return "?";
    }
}

// Send a command, print response. Returns true if ACK.
static bool do_cmd(int fd, const char* label, uint8_t cmd,
                   const uint8_t* payload = nullptr, uint8_t len = 0) {
    send_frame(fd, cmd, payload, len);
    uint8_t rcmd=0, rlen=0, rbuf[32]{};
    bool got = recv_frame(fd, rcmd, rbuf, rlen);
    if (!got) {
        printf("  [FAIL] %-30s → NO RESPONSE\n", label);
        return false;
    }
    if (rcmd == 0x80) { // ACK
        printf("  [ACK ] %-30s → ACK(cmd=%02x)\n", label, rbuf[0]);
        return true;
    } else if (rcmd == 0x81) { // NACK
        uint8_t err = rlen >= 2 ? rbuf[1] : 0;
        printf("  [NACK] %-30s → NACK err=%s(%02x)\n", label, err_name(err), err);
        return false;
    } else {
        printf("  [????] %-30s → rsp=%02x(%s) len=%d\n", label, rcmd, resp_name(rcmd), rlen);
        return false;
    }
}

static void get_status(int fd) {
    send_frame(fd, 0x21); // CMD_GET_STATUS
    uint8_t rcmd=0, rlen=0, rbuf[32]{};
    if (!recv_frame(fd, rcmd, rbuf, rlen) || rcmd != 0x91 || rlen < 5) {
        printf("  [WARN] GET_STATUS: no valid response (rcmd=%02x rlen=%d)\n", rcmd, rlen);
        return;
    }
    int16_t rpm1 = (int16_t)((rbuf[1]<<8)|rbuf[2]);
    int16_t rpm2 = (int16_t)((rbuf[3]<<8)|rbuf[4]);
    printf("  [STAT] state=%-8s  M1=%d RPM  M2=%d RPM\n",
           state_name(rbuf[0]), rpm1, rpm2);
}

} // namespace raw_uart

static int cmd_test_motor(const char* uart_dev, int argc, char** argv)
{
    // Parse optional m1=<speed> m2=<speed> from remaining argv
    // argv here is the full argv; scan from index 3 onward
    bool manual = false;
    int  m1_speed = 0, m2_speed = 0;
    for (int i = 3; i < argc; i++) {
        if (strncmp(argv[i], "m1=", 3) == 0) { m1_speed = atoi(argv[i] + 3); manual = true; }
        if (strncmp(argv[i], "m2=", 3) == 0) { m2_speed = atoi(argv[i] + 3); manual = true; }
    }

    printf("=== test-motor RAW: %s ===\n", uart_dev);

    int fd;
    if (!raw_uart::open_port(uart_dev, fd)) return 1;
    usleep(100000);
    tcflush(fd, TCIOFLUSH);

    // Always: INIT → CONFIG
    printf("\n[INIT]\n");
    raw_uart::do_cmd(fd, "INIT",               0x01);
    printf("\n[CONFIG]\n");
    uint8_t cfg[4] = {0x12,0x48, 0x4E,0x20}; // PPR=4680, FREQ=20000
    raw_uart::do_cmd(fd, "CONFIG(4680,20000)", 0x02, cfg, 4);
    raw_uart::get_status(fd);

    if (manual) {
        // ── Manual mode: set requested speeds and hold ────────────────────────
        auto make_spd = [](int16_t spd, uint8_t id, uint8_t out[3]) {
            out[0] = id;
            out[1] = (uint8_t)((uint16_t)spd >> 8);
            out[2] = (uint8_t)((uint16_t)spd & 0xFF);
        };
        printf("\n[MANUAL] M1=%d  M2=%d\n", m1_speed, m2_speed);
        uint8_t p1[3], p2[3];
        make_spd((int16_t)m1_speed, 0, p1);
        make_spd((int16_t)m2_speed, 1, p2);
        char lbl1[32], lbl2[32];
        snprintf(lbl1, sizeof(lbl1), "SET_SPEED M1=%d", m1_speed);
        snprintf(lbl2, sizeof(lbl2), "SET_SPEED M2=%d", m2_speed);
        raw_uart::do_cmd(fd, lbl1, 0x10, p1, 3);
        raw_uart::do_cmd(fd, lbl2, 0x10, p2, 3);
        raw_uart::get_status(fd);

        printf("  (running for 5s, then STOP)\n");
        sleep(5);

        raw_uart::do_cmd(fd, "STOP M1", 0x11, (const uint8_t*)"\x00", 1);
        raw_uart::do_cmd(fd, "STOP M2", 0x11, (const uint8_t*)"\x01", 1);
        raw_uart::get_status(fd);

    } else {
        // ── Auto sequence ─────────────────────────────────────────────────────
        printf("\n[3] SET_SPEED forward (200, 200)\n");
        uint8_t spd_fwd_m1[3] = {0x00, 0x00,0xC8};
        uint8_t spd_fwd_m2[3] = {0x01, 0x00,0xC8};
        raw_uart::do_cmd(fd, "SET_SPEED M1=200",  0x10, spd_fwd_m1, 3);
        raw_uart::do_cmd(fd, "SET_SPEED M2=200",  0x10, spd_fwd_m2, 3);
        raw_uart::get_status(fd);
        sleep(2);

        printf("\n[4] STOP both\n");
        raw_uart::do_cmd(fd, "STOP M1", 0x11, (const uint8_t*)"\x00", 1);
        raw_uart::do_cmd(fd, "STOP M2", 0x11, (const uint8_t*)"\x01", 1);
        raw_uart::get_status(fd);
        sleep(1);

        printf("\n[5] SET_SPEED backward (-200, -200)\n");
        uint8_t spd_bwd_m1[3] = {0x00, 0xFF,0x38};
        uint8_t spd_bwd_m2[3] = {0x01, 0xFF,0x38};
        raw_uart::do_cmd(fd, "SET_SPEED M1=-200", 0x10, spd_bwd_m1, 3);
        raw_uart::do_cmd(fd, "SET_SPEED M2=-200", 0x10, spd_bwd_m2, 3);
        raw_uart::get_status(fd);
        sleep(2);

        printf("\n[6] STOP both\n");
        raw_uart::do_cmd(fd, "STOP M1", 0x11, (const uint8_t*)"\x00", 1);
        raw_uart::do_cmd(fd, "STOP M2", 0x11, (const uint8_t*)"\x01", 1);
        raw_uart::get_status(fd);
        sleep(1);

        printf("\n[7] Spin left (-180, +180)\n");
        uint8_t spd_l_m1[3] = {0x00, 0xFF,0x4A};
        uint8_t spd_l_m2[3] = {0x01, 0x00,0xB4};
        raw_uart::do_cmd(fd, "SET_SPEED M1=-180", 0x10, spd_l_m1, 3);
        raw_uart::do_cmd(fd, "SET_SPEED M2=+180", 0x10, spd_l_m2, 3);
        raw_uart::get_status(fd);
        sleep(2);

        printf("\n[8] STOP → RESET\n");
        raw_uart::do_cmd(fd, "STOP M1", 0x11, (const uint8_t*)"\x00", 1);
        raw_uart::do_cmd(fd, "STOP M2", 0x11, (const uint8_t*)"\x01", 1);
        usleep(300000);
        raw_uart::do_cmd(fd, "RESET", 0xFF);
    }

    printf("\n=== test-motor DONE ===\n");
    close(fd);
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
    LOGI("  %s test-motor [uart_dev] [m1=<rpm>] [m2=<rpm>]", prog);
    LOGI("      -- no m1/m2: run auto sequence");
    LOGI("      -- with m1/m2: hold that speed for 5s then stop");
    LOGI("      -- example: %s test-motor /dev/ttyUSB0 m1=200 m2=200", prog);
    LOGI("      -- example: %s test-motor /dev/ttyUSB0 m1=-150 m2=150", prog);
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

    // Buffers
    const size_t MJPEG_BUF = 1024 * 1024; // 1 MB should be enough for 640×480 MJPEG
    uint8_t* mjpeg_buf = static_cast<uint8_t*>(malloc(MJPEG_BUF));
    uint8_t* rgb_buf   = static_cast<uint8_t*>(malloc(model_w * model_h * 3));
    if (!mjpeg_buf || !rgb_buf) { LOGE("OOM"); return 1; }

    // ── State machine ─────────────────────────────────────────────────────────
    RobotState robot;

    int frame_idx   = 0;
    long total_us   = 0;
    int  frame_cnt  = 0;

    while (true) {
        struct timeval t_start, t_end;
        gettimeofday(&t_start, nullptr);
        frame_idx++;

        // 1. Grab MJPEG frame
        int jpeg_len = capture.getFrame(mjpeg_buf, MJPEG_BUF, 200);
        if (jpeg_len <= 0) {
            LOGW("[Frame %d] No frame (timeout)", frame_idx);
            usleep(10000);
            continue;
        }

        // 2. Decode MJPEG → RGB letterboxed to model input, capture letterbox params
        int lb_pad_x = 0, lb_pad_y = 0;
        float lb_scale = 1.0f;
        if (decode_mjpeg(mjpeg_buf, jpeg_len, rgb_buf, model_w, model_h,
                         &lb_pad_x, &lb_pad_y, &lb_scale) != 0) {
            LOGW("[Frame %d] JPEG decode failed", frame_idx);
            continue;
        }

        // 3. Inference (suppress library stdout chatter)
        std::vector<detection> dets;
        {
            int saved_stdout = dup(STDOUT_FILENO);
            int devnull = open("/dev/null", O_WRONLY);
            dup2(devnull, STDOUT_FILENO);
            close(devnull);
            detect_run(&rknn_ctx, rgb_buf, model_w, model_h,
                       FRAME_WIDTH, FRAME_HEIGHT, lb_pad_x, lb_pad_y, lb_scale,
                       0.5f, 0.45f, dets);
            fflush(stdout);
            dup2(saved_stdout, STDOUT_FILENO);
            close(saved_stdout);
        }

        // 4. State machine
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
                    motor.drive(-TURN_SPEED, TURN_SPEED);
                else
                    motor.drive(TURN_SPEED, -TURN_SPEED);
                usleep(pulse_us);
                motor.standby();

            } else {
                // Chase
                robot.grab_confirm_count = 0;
                robot.status = STATUS_CHASE_TENNIS;

                if (!centered) {
                    if (offset < 0) {
                        LOGD("[MOTOR] TURN LEFT cx=%d pulse=%dms", ball_cx, pulse_us / 1000);
                        motor.drive(-TURN_SPEED, TURN_SPEED);
                    } else {
                        LOGD("[MOTOR] TURN RIGHT cx=%d pulse=%dms", ball_cx, pulse_us / 1000);
                        motor.drive(TURN_SPEED, -TURN_SPEED);
                    }
                    usleep(pulse_us);
                    motor.standby();
                } else {
                    LOGD("[MOTOR] FORWARD area=%.3f", area_ratio);
                    motor.forward(CHASE_SPEED);
                }
            }

        } else {
            LOGD("[DETECT] No ball, searching...");
            robot.grab_confirm_count = 0;
            robot.status = STATUS_CHASE_TENNIS;
            motor.drive(IDLE_SPEED, -IDLE_SPEED); // 原地右转搜索
        }

        // FPS — only print when a ball is detected
        gettimeofday(&t_end, nullptr);
        long frame_us = (t_end.tv_sec  - t_start.tv_sec)  * 1000000L
                      + (t_end.tv_usec - t_start.tv_usec);
        frame_cnt++;
        total_us += frame_us;
        if (!dets.empty()) {
            printf("[FPS] %.2f  avg: %.2f  (%.1fms)\n",
                   1e6f / frame_us,
                   1e6f * frame_cnt / total_us,
                   frame_us / 1000.0f);
        }
    }

    free(mjpeg_buf);
    free(rgb_buf);
    cleanup_and_exit();
    return 0;
}
