#include "uart_motor_driver.hpp"

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

// ── Protocol constants ────────────────────────────────────────────────────────
static const uint8_t FRAME_SOF0 = 0xAA;
static const uint8_t FRAME_SOF1 = 0x55;

static const uint8_t CMD_INIT       = 0x01;
static const uint8_t CMD_CONFIG     = 0x02;
static const uint8_t CMD_SET_SPEED  = 0x10;   // per-motor: [id, int16 BE]
static const uint8_t CMD_STOP       = 0x11;
static const uint8_t CMD_BRAKE      = 0x12;
// CMD_SET_SPEEDS (0x13) not supported by current firmware
static const uint8_t CMD_RESET      = 0xFF;

static const uint8_t RSP_ACK  = 0x80;
static const uint8_t RSP_NACK = 0x81;

// ── Helpers ───────────────────────────────────────────────────────────────────
static uint8_t calc_checksum(uint8_t cmd, uint8_t len, const uint8_t* payload)
{
    uint8_t chk = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) chk ^= payload[i];
    return chk;
}

static void put_be16(uint8_t* buf, uint16_t v)
{
    buf[0] = (v >> 8) & 0xFF;
    buf[1] =  v       & 0xFF;
}

static void put_be16_signed(uint8_t* buf, int16_t v)
{
    put_be16(buf, static_cast<uint16_t>(v));
}

// ── Constructor / Destructor ──────────────────────────────────────────────────
UartMotorDriver::UartMotorDriver(const std::string& uart_dev,
                                 int speed_scale,
                                 uint16_t ppr,
                                 uint16_t pwm_freq)
    : fd_(-1), speed_scale_(speed_scale), ppr_(ppr), pwm_freq_(pwm_freq)
{
    if (!open_uart(uart_dev)) {
        fprintf(stderr, "[UartMotorDriver] Failed to open %s\n", uart_dev.c_str());
        return;
    }
    // Match tennis.cpp open_port: wait for ESP32 to settle then flush stale bytes
    usleep(100000);
    tcflush(fd_, TCIOFLUSH);
    cmd_init();
    cmd_config(ppr_, pwm_freq_);
}

UartMotorDriver::~UartMotorDriver()
{
    if (fd_ >= 0) {
        // Reset ESP32 controller: stops motors and returns to UNINIT state
        uint8_t no_payload = 0;
        send_frame(CMD_RESET, &no_payload, 0);
        close_uart();
    }
}

// ── UART open / close ─────────────────────────────────────────────────────────
bool UartMotorDriver::open_uart(const std::string& dev)
{
    fd_ = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) { close(fd_); fd_ = -1; return false; }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);          // no parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // no HW flow control

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_lflag  = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(fd_); fd_ = -1; return false; }

    return true;
}

void UartMotorDriver::close_uart()
{
    if (fd_ >= 0) { close(fd_); fd_ = -1; }
}

// ── Frame send / recv ─────────────────────────────────────────────────────────
void UartMotorDriver::send_frame(uint8_t cmd, const uint8_t* payload, uint8_t len)
{
    if (fd_ < 0) return;

    uint8_t buf[64];
    buf[0] = FRAME_SOF0;
    buf[1] = FRAME_SOF1;
    buf[2] = cmd;
    buf[3] = len;
    if (len > 0 && payload) memcpy(&buf[4], payload, len);
    buf[4 + len] = calc_checksum(cmd, len, payload ? payload : (const uint8_t*)"");
    ssize_t _w = write(fd_, buf, 5 + len);
    (void)_w;
}

bool UartMotorDriver::recv_frame(uint8_t* cmd_out, uint8_t* payload_out,
                                  uint8_t* len_out, int timeout_ms)
{
    if (fd_ < 0) return false;

    auto wait_byte = [&](uint8_t* b, int ms) -> bool {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        struct timeval tv = { ms / 1000, (ms % 1000) * 1000 };
        if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;
        return read(fd_, b, 1) == 1;
    };

    // Wait for SOF
    uint8_t b;
    int deadline = timeout_ms;
    while (deadline > 0) {
        if (!wait_byte(&b, 10)) { deadline -= 10; continue; }
        if (b != FRAME_SOF0) continue;
        if (!wait_byte(&b, 10)) return false;
        if (b == FRAME_SOF1) break;
    }
    if (deadline <= 0) return false;

    uint8_t cmd, len;
    if (!wait_byte(&cmd, 50)) return false;
    if (!wait_byte(&len, 50)) return false;

    uint8_t payload[32] = {};
    for (uint8_t i = 0; i < len && i < sizeof(payload); i++) {
        if (!wait_byte(&payload[i], 50)) return false;
    }

    uint8_t chk;
    if (!wait_byte(&chk, 50)) return false;

    if (calc_checksum(cmd, len, payload) != chk) return false;

    *cmd_out = cmd;
    *len_out = len;
    if (payload_out) memcpy(payload_out, payload, len);
    return true;
}

// ── Protocol commands ─────────────────────────────────────────────────────────
void UartMotorDriver::cmd_init()
{
    send_frame(CMD_INIT, nullptr, 0);
    uint8_t cmd, payload[8], len;
    recv_frame(&cmd, payload, &len, 500);
    // Expect RSP_ACK; ignore errors (best-effort init)
}

void UartMotorDriver::cmd_config(uint16_t ppr, uint16_t pwm_freq)
{
    uint8_t payload[4];
    put_be16(&payload[0], ppr);
    put_be16(&payload[2], pwm_freq);
    send_frame(CMD_CONFIG, payload, 4);
    uint8_t cmd, rsp[8], len;
    recv_frame(&cmd, rsp, &len, 500);
}

void UartMotorDriver::cmd_set_speeds(int16_t m1_rpm, int16_t m2_rpm)
{
    // Firmware only supports CMD_SET_SPEED (0x10) per-motor:
    //   payload: [motor_id(1B)] [speed(int16 BE)]
    uint8_t p1[3] = { 0x00,
                      (uint8_t)((uint16_t)m1_rpm >> 8),
                      (uint8_t)((uint16_t)m1_rpm & 0xFF) };
    uint8_t p2[3] = { 0x01,
                      (uint8_t)((uint16_t)m2_rpm >> 8),
                      (uint8_t)((uint16_t)m2_rpm & 0xFF) };
    send_frame(0x10, p1, 3);   // CMD_SET_SPEED M1
    // Use 500ms timeout to match tennis.cpp do_cmd behaviour (was 80ms, too short)
    uint8_t cmd, rsp[8], len;
    recv_frame(&cmd, rsp, &len, 500);

    send_frame(0x10, p2, 3);   // CMD_SET_SPEED M2
    recv_frame(&cmd, rsp, &len, 500);
}

void UartMotorDriver::cmd_stop(uint8_t motor_id)
{
    send_frame(CMD_STOP, &motor_id, 1);
    uint8_t cmd, rsp[8], len;
    recv_frame(&cmd, rsp, &len, 500);
}

void UartMotorDriver::cmd_brake(uint8_t motor_id)
{
    send_frame(CMD_BRAKE, &motor_id, 1);
    uint8_t cmd, rsp[8], len;
    recv_frame(&cmd, rsp, &len, 500);
}

// ── MotorDriver interface ─────────────────────────────────────────────────────

// Convert Motor [-100,100] to RPM
static int16_t to_rpm(int speed, int scale)
{
    // Clamp
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;
    return static_cast<int16_t>(speed * scale / 100);
}

void UartMotorDriver::drive(int left_speed, int right_speed)
{
    cmd_set_speeds(to_rpm(left_speed, speed_scale_),
                   to_rpm(right_speed, speed_scale_));
}

void UartMotorDriver::brake()
{
    cmd_brake(0);
    cmd_brake(1);
}

void UartMotorDriver::standby()
{
    cmd_stop(0);
    cmd_stop(1);
}
