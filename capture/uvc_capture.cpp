// UVC capture wrapper for rk3588.
// Logic follows uvc-fps-c/uvc-fps.c - only libuvc functions used there are used here.
#include "uvc_capture.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <time.h>

// ── helpers from uvc-fps.c ────────────────────────────────────────────────────
// UVC_FRAME_FORMAT_MJPEG is defined in libuvc.h - do not redefine

UvcCapture::UvcCapture()
{
    pthread_mutex_init(&mutex_, nullptr);
    pthread_cond_init(&cond_, nullptr);
}

UvcCapture::~UvcCapture()
{
    close();
    free(pending_data_);
    pthread_cond_destroy(&cond_);
    pthread_mutex_destroy(&mutex_);
}

// ── open ──────────────────────────────────────────────────────────────────────
int UvcCapture::open(int device_index, int width, int height, int fps)
{
    width_  = width;
    height_ = height;

    // uvc_init
    int res = uvc_init(&ctx_, nullptr);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_init failed: %s\n", uvc_strerror((uvc_error_t)res));
        return -1;
    }

    // enumerate devices
    uvc_device_t** dev_list = nullptr;
    res = uvc_get_device_list(ctx_, &dev_list);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_get_device_list failed: %s\n", uvc_strerror((uvc_error_t)res));
        uvc_exit(ctx_); ctx_ = nullptr;
        return -1;
    }

    uvc_device_t* dev = nullptr;
    for (int i = 0; dev_list[i] != nullptr; i++) {
        if (i == device_index) {
            dev = dev_list[i];
            uvc_ref_device(dev);
            break;
        }
    }
    uvc_free_device_list(dev_list, 1);

    if (!dev) {
        fprintf(stderr, "[UvcCapture] device index %d not found\n", device_index);
        uvc_exit(ctx_); ctx_ = nullptr;
        return -1;
    }

    res = uvc_open(dev, &devh_);
    uvc_unref_device(dev);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_open failed: %s\n", uvc_strerror((uvc_error_t)res));
        uvc_exit(ctx_); ctx_ = nullptr;
        return -1;
    }

    // negotiate stream control (MJPEG)
    res = uvc_get_stream_ctrl_format_size(
        devh_, &ctrl_, UVC_FRAME_FORMAT_MJPEG, width, height, fps);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_get_stream_ctrl_format_size failed: %s\n", uvc_strerror((uvc_error_t)res));
        uvc_close(devh_); devh_ = nullptr;
        uvc_exit(ctx_);   ctx_  = nullptr;
        return -1;
    }

    // start streaming
    res = uvc_start_streaming(devh_, &ctrl_, frame_cb, this, 0);
    if (res < 0) {
        fprintf(stderr, "[UvcCapture] uvc_start_streaming failed: %s\n", uvc_strerror((uvc_error_t)res));
        uvc_close(devh_); devh_ = nullptr;
        uvc_exit(ctx_);   ctx_  = nullptr;
        return -1;
    }

    printf("[UvcCapture] streaming %dx%d @ %d fps\n", width, height, fps);
    return 0;
}

void UvcCapture::close()
{
    if (devh_) {
        uvc_stop_streaming(devh_);
        uvc_close(devh_);
        devh_ = nullptr;
    }
    if (ctx_) {
        uvc_exit(ctx_);
        ctx_ = nullptr;
    }
}

// ── frame callback (libuvc thread) ───────────────────────────────────────────
void UvcCapture::frame_cb(uvc_frame_t* frame, void* ptr)
{
    static_cast<UvcCapture*>(ptr)->on_frame(frame);
}

void UvcCapture::on_frame(uvc_frame_t* frame)
{
    if (!frame->data || frame->data_bytes == 0) return;

    pthread_mutex_lock(&mutex_);

    // Grow buffer if needed
    if (pending_cap_ < frame->data_bytes) {
        free(pending_data_);
        pending_data_ = static_cast<uint8_t*>(malloc(frame->data_bytes));
        pending_cap_  = pending_data_ ? frame->data_bytes : 0;
    }

    if (pending_data_) {
        memcpy(pending_data_, frame->data, frame->data_bytes);
        pending_len_ = frame->data_bytes;
        has_pending_ = true;
        pthread_cond_signal(&cond_);
    }

    pthread_mutex_unlock(&mutex_);
}

// ── getFrame ──────────────────────────────────────────────────────────────────
int UvcCapture::getFrame(uint8_t* buf, size_t cap, int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }

    pthread_mutex_lock(&mutex_);
    while (!has_pending_) {
        int rc = pthread_cond_timedwait(&cond_, &mutex_, &ts);
        if (rc != 0) {
            pthread_mutex_unlock(&mutex_);
            return -1; // timeout
        }
    }

    size_t copy_len = pending_len_ < cap ? pending_len_ : cap;
    memcpy(buf, pending_data_, copy_len);
    has_pending_ = false;
    pthread_mutex_unlock(&mutex_);

    return static_cast<int>(copy_len);
}
