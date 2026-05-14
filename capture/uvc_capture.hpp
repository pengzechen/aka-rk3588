#ifndef UVC_CAPTURE_HPP
#define UVC_CAPTURE_HPP

#include <cstdint>
#include <cstddef>
#include <pthread.h>

#include <libuvc/libuvc.h>

// UVC camera capture wrapper.
// Uses only libuvc functions already used in uvc-fps-c/uvc-fps.c.
// After open(), call getFrame() in a loop; it fills a caller-supplied buffer
// with raw MJPEG bytes.  Call close() when done.
class UvcCapture {
public:
    UvcCapture();
    ~UvcCapture();

    // Open device at zero-based index with requested format.
    // Returns 0 on success, -1 on failure.
    int open(int device_index = 0,
             int width = 640,
             int height = 480,
             int fps = 30);

    void close();

    // Block until the next frame arrives (or timeout_ms elapses).
    // Copies MJPEG data into buf (capacity cap).
    // Returns number of bytes written, or -1 on error/timeout.
    int getFrame(uint8_t* buf, size_t cap, int timeout_ms = 200);

    int width()  const { return width_;  }
    int height() const { return height_; }

private:
    // Called by libuvc from its internal thread
    static void frame_cb(uvc_frame_t* frame, void* ptr);
    void on_frame(uvc_frame_t* frame);

    uvc_context_t*       ctx_  = nullptr;
    uvc_device_handle_t* devh_ = nullptr;
    uvc_stream_ctrl_t    ctrl_ = {};

    int width_  = 640;
    int height_ = 480;

    // Single-slot frame ring: libuvc thread writes, getFrame() reads.
    pthread_mutex_t mutex_;
    pthread_cond_t  cond_;

    uint8_t* pending_data_ = nullptr;
    size_t   pending_len_  = 0;
    size_t   pending_cap_  = 0;
    bool     has_pending_  = false;
};

#endif // UVC_CAPTURE_HPP
