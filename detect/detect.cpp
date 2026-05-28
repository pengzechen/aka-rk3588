#include "detect.hpp"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <sys/time.h>

#include "rknn_api.h"
#include "image_utils.h"
#include "common.h"

// ── Timing helper ───────────────────────────────────────────────────────────────
static long elapsed_us(const struct timeval& start)
{
    struct timeval now;
    gettimeofday(&now, nullptr);
    return (now.tv_sec - start.tv_sec) * 1000000L + (now.tv_usec - start.tv_usec);
}

// ── NMS ───────────────────────────────────────────────────────────────────────
static float iou_cx(float ax, float ay, float aw, float ah,
                    float bx, float by, float bw, float bh)
{
    float ax1 = ax - aw * 0.5f, ay1 = ay - ah * 0.5f;
    float ax2 = ax + aw * 0.5f, ay2 = ay + ah * 0.5f;
    float bx1 = bx - bw * 0.5f, by1 = by - bh * 0.5f;
    float bx2 = bx + bw * 0.5f, by2 = by + bh * 0.5f;
    float iw = std::max(0.0f, std::min(ax2, bx2) - std::max(ax1, bx1));
    float ih = std::max(0.0f, std::min(ay2, by2) - std::max(ay1, by1));
    float inter = iw * ih;
    float uni = aw * ah + bw * bh - inter;
    return uni > 0 ? inter / uni : 0.0f;
}

// ── Init / deinit ─────────────────────────────────────────────────────────────
int detect_init(const char* model_path, rknn_app_context_t* ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    if (init_yolov8_model(model_path, ctx) != 0) {
        fprintf(stderr, "[detect] init_yolov8_model failed\n");
        return -1;
    }
    fprintf(stderr, "[detect] model loaded  w=%d h=%d ch=%d  n_out=%d\n",
            ctx->model_width, ctx->model_height, ctx->model_channel,
            ctx->io_num.n_output);
    for (int i = 0; i < (int)ctx->io_num.n_output; i++) {
        rknn_tensor_attr& a = ctx->output_attrs[i];
        fprintf(stderr, "[detect]   out[%d] dims=[%d,%d,%d,%d] type=%s\n",
                i, a.dims[0], a.dims[1], a.dims[2], a.dims[3],
                get_type_string(a.type));
    }
    return 0;
}

void detect_deinit(rknn_app_context_t* ctx)
{
    release_yolov8_model(ctx);
}

// ── Inference ─────────────────────────────────────────────────────────────────
// rgb_data : RGB888 letterboxed to model_w×model_h by decode_mjpeg().
// pad_x/y  : pixel offset added during letterbox.
// lbox_scale: scale factor (original → model).
// Outputs bbox in original camera frame (orig_w × orig_h), cx/cy center format.
int detect_run(rknn_app_context_t* ctx,
               const uint8_t* rgb_data, int model_w, int model_h,
               int orig_w, int orig_h,
               int pad_x, int pad_y, float lbox_scale,
               float conf_thresh, float iou_thresh,
               std::vector<detection>& dets)
{
    struct timeval t_start, t_stage;
    gettimeofday(&t_start, nullptr);

    dets.clear();

    // 1. Set input
    gettimeofday(&t_stage, nullptr);
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index  = 0;
    inputs[0].type   = RKNN_TENSOR_UINT8;
    inputs[0].fmt    = RKNN_TENSOR_NHWC;
    inputs[0].size   = model_w * model_h * ctx->model_channel;
    inputs[0].buf    = const_cast<unsigned char*>(rgb_data);

    int ret = rknn_inputs_set(ctx->rknn_ctx, 1, inputs);
    if (ret < 0) { fprintf(stderr, "[detect] rknn_inputs_set %d\n", ret); return -1; }
    long t_input = elapsed_us(t_stage);

    // 2. Run
    gettimeofday(&t_stage, nullptr);
    ret = rknn_run(ctx->rknn_ctx, nullptr);
    if (ret < 0) { fprintf(stderr, "[detect] rknn_run %d\n", ret); return -1; }
    long t_run = elapsed_us(t_stage);

    // 3. Get output as FP32
    gettimeofday(&t_stage, nullptr);
    int n_out = ctx->io_num.n_output;
    rknn_output outputs[n_out];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < n_out; i++) { outputs[i].index = i; outputs[i].want_float = 1; }
    ret = rknn_outputs_get(ctx->rknn_ctx, n_out, outputs, nullptr);
    if (ret < 0) { fprintf(stderr, "[detect] rknn_outputs_get %d\n", ret); return -1; }
    long t_output = elapsed_us(t_stage);

    // 4. Parse [1, 5, N] — single merged output, conf already sigmoid
    gettimeofday(&t_stage, nullptr);
    const rknn_tensor_attr& oa = ctx->output_attrs[0];
    int d1 = oa.dims[1], d2 = oa.dims[2];
    bool layout_5N = (d1 < d2);
    int N = layout_5N ? d2 : d1;
    int F = layout_5N ? d1 : d2;
    const float* buf = (const float*)outputs[0].buf;

    float max_conf = 0.0f;
    for (int j = 0; j < N; j++) {
        float c = layout_5N ? buf[4 * N + j] : buf[j * F + 4];
        if (c > max_conf) max_conf = c;
    }

    // 5. Filter + undo letterbox
    std::vector<detection> cands;
    for (int j = 0; j < N; j++) {
        float cx, cy, w, h, conf;
        if (layout_5N) {
            cx = buf[0*N+j]; cy = buf[1*N+j];
            w  = buf[2*N+j]; h  = buf[3*N+j]; conf = buf[4*N+j];
        } else {
            cx = buf[j*F+0]; cy = buf[j*F+1];
            w  = buf[j*F+2]; h  = buf[j*F+3]; conf = buf[j*F+4];
        }
        if (conf < conf_thresh) continue;

        // Undo letterbox → original camera frame
        float rx = (cx - pad_x) / lbox_scale;
        float ry = (cy - pad_y) / lbox_scale;
        float rw = w / lbox_scale;
        float rh = h / lbox_scale;
        rx = std::max(0.0f, std::min(rx, (float)orig_w));
        ry = std::max(0.0f, std::min(ry, (float)orig_h));

        detection d;
        d.bbox.x = rx; d.bbox.y = ry;
        d.bbox.w = rw; d.bbox.h = rh;
        d.score = conf; d.cls = 0; d.batch_idx = 0;
        cands.push_back(d);
    }

    // 6. NMS
    std::sort(cands.begin(), cands.end(),
              [](const detection& a, const detection& b){ return a.score > b.score; });
    std::vector<bool> sup(cands.size(), false);
    for (int i = 0; i < (int)cands.size(); i++) {
        if (sup[i]) continue;
        dets.push_back(cands[i]);
        for (int j = i+1; j < (int)cands.size(); j++) {
            if (!sup[j] &&
                iou_cx(cands[i].bbox.x, cands[i].bbox.y, cands[i].bbox.w, cands[i].bbox.h,
                       cands[j].bbox.x, cands[j].bbox.y, cands[j].bbox.w, cands[j].bbox.h)
                > iou_thresh)
                sup[j] = true;
        }
    }
    long t_post = elapsed_us(t_stage);

    // 7. Release
    gettimeofday(&t_stage, nullptr);
    rknn_outputs_release(ctx->rknn_ctx, n_out, outputs);
    long t_release = elapsed_us(t_stage);

    long t_total = elapsed_us(t_start);

    // Print timing breakdown - always print for debugging
    printf("[detect] total=%.1fms  in=%.1f run=%.1f out=%.1f post=%.1f rel=%.1f  cands=%d nms=%d\n",
            t_total / 1000.0f,
            t_input / 1000.0f,
            t_run / 1000.0f,
            t_output / 1000.0f,
            t_post / 1000.0f,
            t_release / 1000.0f,
            (int)cands.size(), (int)dets.size());
    fflush(stdout);

    return (int)dets.size();
}
