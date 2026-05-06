// can0_monitor.cpp — 事件驱动的 CAN 监听，只输出关键状态变化
// 编译: g++ -O2 can0_monitor.cpp -o can0_monitor -lm
// 运行: sudo ./can0_monitor can0

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cstring>
#include <ctime>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>

static volatile bool g_running = true;
static void on_signal(int) { g_running = false; }

// ── 状态跟踪 ──────────────────────────────────────────────────────────────

struct MotorState {
  int32_t last_cmd_pos = 0;       // 上次命令位置
  int32_t last_report_pos = 0;    // 上次反馈位置
  bool has_cmd_pos = false;
  bool has_report_pos = false;
  double last_cmd_time = 0;       // 上次命令时间
  double last_resp_time = 0;      // 上次响应时间
  int cmd_count = 0;              // 命令计数
  int resp_count = 0;             // 响应计数
  int stale_resp_count = 0;       // 残留响应计数（命令帧之间收到的响应）
};

struct CylState {
  int16_t last_cmd_pos_high = 0;
  int16_t last_cmd_pos_low = 0;
  bool last_is_read = false;       // 上次是读还是写
  double last_cmd_time = 0;
  double last_resp_time = 0;
  int cmd_count = 0;
  int read_count = 0;
  int write_count = 0;
  int read_resp_count = 0;
  int write_resp_count = 0;
  int skipped_func_count = 0;     // 功能码不匹配被跳过的帧数
};

static std::map<uint8_t, MotorState> g_zeroerr;   // Node → State
static std::map<uint8_t, MotorState> g_rmd;        // MotorID → State
static CylState g_cyl;

static double g_start_time = 0;
static double g_last_summary_time = 0;
static int g_total_frames = 0;
static int g_unknown_frames = 0;
static FILE * g_logfile = nullptr;

// ── 时间 ──────────────────────────────────────────────────────────────────

static double now_sec()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static double elapsed() { return now_sec() - g_start_time; }

// ── 事件输出 ──────────────────────────────────────────────────────────────

static void event(const char * fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  char buf[512];
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  printf("[%.3f] %s\n", elapsed(), buf);
  fflush(stdout);
  if (g_logfile) { fprintf(g_logfile, "[%.3f] %s\n", elapsed(), buf); fflush(g_logfile); }
}

// ── 定期摘要（每 2 秒） ──────────────────────────────────────────────────

static void print_summary()
{
  double t = elapsed();
  event("── SUMMARY ── total_frames=%d, unknown=%d", g_total_frames, g_unknown_frames);
  for (auto & [node, s] : g_zeroerr) {
    if (s.cmd_count > 0 || s.resp_count > 0) {
      event("  ZeroErr Node%d: cmds=%d resps=%d stale=%d last_cmd=%.3fs ago",
        node, s.cmd_count, s.resp_count, s.stale_resp_count,
        t - (g_start_time + s.last_cmd_time));
    }
  }
  event("  Cylinder: writes=%d reads=%d write_ack=%d read_ack=%d skipped_func=%d",
    g_cyl.write_count, g_cyl.read_count,
    g_cyl.write_resp_count, g_cyl.read_resp_count, g_cyl.skipped_func_count);
  for (auto & [motor, s] : g_rmd) {
    if (s.cmd_count > 0 || s.resp_count > 0) {
      event("  RMD Motor%d: cmds=%d resps=%d last_cmd=%.3fs ago",
        motor, s.cmd_count, s.resp_count,
        t - (g_start_time + s.last_cmd_time));
    }
  }
}

// ── 协议解码 ──────────────────────────────────────────────────────────────

static void decode_zeroerr(uint32_t id, const uint8_t * data, uint8_t dlc)
{
  uint8_t node = id & 0x0F;
  bool is_cmd = ((id & 0x7F0) == 0x640);
  auto & s = g_zeroerr[node];
  double t = now_sec() - g_start_time;

  if (is_cmd) {
    s.cmd_count++;
    s.last_cmd_time = t;

    if (dlc >= 2) {
      uint8_t cmd = data[1];
      if (cmd == 0x86 && dlc >= 6) {
        // 设置目标位置
        int32_t pos = (int32_t(data[2]) << 24) | (int32_t(data[3]) << 16) |
                      (int32_t(data[4]) << 8)  | int32_t(data[5]);
        if (!s.has_cmd_pos || pos != s.last_cmd_pos) {
          event("ZE Node%d SET_POS %d (delta=%d)", node, pos,
                s.has_cmd_pos ? pos - s.last_cmd_pos : 0);
          s.last_cmd_pos = pos;
          s.has_cmd_pos = true;
        }
      } else if (cmd == 0x83) {
        event("ZE Node%d START_MOVE", node);
      } else if (cmd == 0x02) {
        // 读取位置 — 静默计数，只在新一轮读取时检测残留帧
      } else if (cmd == 0x84) {
        event("ZE Node%d STOP", node);
      } else if (cmd == 0x01 && dlc >= 6) {
        event("ZE Node%d ENABLE(%d)", node, data[5]);
      }
    }
  } else {
    // 响应帧
    s.resp_count++;
    s.last_resp_time = t;

    // 检查是否是残留帧（距上次命令 > 10ms 说明不是对应这次命令的响应）
    // 如果上次命令是 0x02(读位置)，响应应该很快
    if (dlc >= 5 && data[4] == 0x3E) {
      int32_t pos = (int32_t(data[0]) << 24) | (int32_t(data[1]) << 16) |
                    (int32_t(data[2]) << 8)  | int32_t(data[3]);
      if (!s.has_report_pos || pos != s.last_report_pos) {
        event("ZE Node%d POS_FEEDBACK %d (delta=%d)", node, pos,
              s.has_report_pos ? pos - s.last_report_pos : 0);
        s.last_report_pos = pos;
        s.has_report_pos = true;
      }
    }
    // 检测残留帧：如果距上次命令很久才收到响应
    if (t - s.last_cmd_time > 0.010) {
      s.stale_resp_count++;
    }
  }
}

static void decode_cylinder(uint32_t id, const uint8_t * data, uint8_t dlc)
{
  bool is_cmd = (id == 0x03);
  double t = now_sec() - g_start_time;

  if (is_cmd) {
    g_cyl.cmd_count++;
    g_cyl.last_cmd_time = t;

    if (dlc >= 2) {
      uint8_t func = data[1];
      if (func == 0x1A) {
        g_cyl.write_count++;
        // 写位置命令 — 检测位置变化
        if (dlc >= 8 && data[5] == 0x05) {
          int16_t h = (int16_t(data[3]) << 8) | data[4];
          int16_t l = (int16_t(data[6]) << 8) | data[7];
          if (h != g_cyl.last_cmd_pos_high || l != g_cyl.last_cmd_pos_low) {
            int32_t pos = (int32_t(h) << 16) | (int32_t(l) & 0xFFFF);
            event("CYL WRITE_POS %d (0x%04X%04X)", pos,
                  (uint16_t)h & 0xFFFF, (uint16_t)l & 0xFFFF);
            g_cyl.last_cmd_pos_high = h;
            g_cyl.last_cmd_pos_low = l;
          }
        }
      } else if (func == 0x2A) {
        g_cyl.read_count++;
      } else if (func == 0x00) {
        // 使能
        if (dlc >= 5) {
          event("CYL ENABLE(%d)", (int16_t(data[3]) << 8) | data[4]);
        }
      }
    }
  } else {
    // 响应
    g_cyl.last_resp_time = t;

    if (dlc >= 2) {
      uint8_t func = data[1];
      if (func == 0x2B) {
        g_cyl.read_resp_count++;
        if (dlc >= 8) {
          int16_t h = (int16_t(data[3]) << 8) | data[4];
          int16_t l = (int16_t(data[6]) << 8) | data[7];
          int32_t pos = (int32_t(h) << 16) | (int32_t(l) & 0xFFFF);
          static int32_t last_cyl_pos = 0;
          static bool has_cyl_pos = false;
          if (!has_cyl_pos || pos != last_cyl_pos) {
            event("CYL POS_FEEDBACK %d (delta=%d)", pos, has_cyl_pos ? pos - last_cyl_pos : 0);
            last_cyl_pos = pos;
            has_cyl_pos = true;
          }
        }
      } else if (func == 0x1B) {
        g_cyl.write_resp_count++;
      } else {
        g_cyl.skipped_func_count++;
        event("CYL UNEXPECTED func=0x%02X", func);
      }
    }
  }
}

static void decode_rmd(uint32_t id, const uint8_t * data, uint8_t dlc)
{
  uint8_t motor = id & 0x0F;
  auto & s = g_rmd[motor];
  double t = now_sec() - g_start_time;

  if (dlc < 1) return;

  uint8_t cmd = data[0];

  // 简单区分：0xA4 写命令帧 data[1-2] 是速度值，响应帧 data[1-7] 是状态
  // 0x92 读命令帧，响应帧也是 0x92 开头
  if (cmd == 0xA4 && dlc >= 7) {
    // 区分命令和响应：命令帧 data[3-6] 是角度(小端)
    // 用经验方法：如果距离上次命令很近(<5ms)，可能是响应
    bool likely_response = (s.last_cmd_time > 0 && (t - s.last_cmd_time) < 0.005 && s.cmd_count > s.resp_count);

    if (!likely_response) {
      s.cmd_count++;
      s.last_cmd_time = t;
      int32_t ac = int32_t(data[3]) | (int32_t(data[4]) << 8) |
                   (int32_t(data[5]) << 16) | (int32_t(data[6]) << 24);
      static std::map<uint8_t, int32_t> last_rmd_cmd;
      if (last_rmd_cmd[motor] != ac) {
        event("RMD Motor%d SET_POS %d ac (delta=%d)", motor, ac,
              last_rmd_cmd.count(motor) ? ac - last_rmd_cmd[motor] : 0);
        last_rmd_cmd[motor] = ac;
      }
    } else {
      s.resp_count++;
      s.last_resp_time = t;
    }
  } else if (cmd == 0x92) {
    // 读位置：可能是命令也可能是响应
    // 读命令一般由驱动发起，响应紧随其后
    // 只记录关键事件
    s.resp_count++;
    s.last_resp_time = t;
  } else if (cmd == 0x88) {
    event("RMD Motor%d ENABLE", motor);
    s.cmd_count++;
    s.last_cmd_time = t;
  }
}

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  const char * iface = (argc > 1) ? argv[1] : "can0";

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);

  int fd = ::socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) { perror("socket"); return 1; }

  struct ifreq ifr;
  strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); ::close(fd); return 1; }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); ::close(fd); return 1; }

  // 不设过滤器 — 接收 can0 上所有帧
  struct can_filter rfilter[1];
  rfilter[0].can_id = 0;
  rfilter[0].can_mask = 0;
  setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  g_start_time = now_sec();
  g_last_summary_time = g_start_time;

  printf("=== can0 event monitor on %s ===\n", iface);
  printf("Output saved to can0_monitor.log\n");
  printf("Press Ctrl+C to stop\n\n");

  FILE * logfile = fopen("can0_monitor.log", "w");
  if (!logfile) { perror("fopen log"); ::close(fd); return 1; }
  g_logfile = logfile;

  while (g_running) {
    struct pollfd pfd = {fd, POLLIN, 0};
    int rc = ::poll(&pfd, 1, 100);
    if (rc <= 0) {
      // 检查是否需要打印定期摘要
      double t = now_sec();
      if (t - g_last_summary_time >= 5.0 && g_total_frames > 0) {
        print_summary();
        g_last_summary_time = t;
      }
      continue;
    }

    struct can_frame frame;
    ssize_t n = ::read(fd, &frame, sizeof(frame));
    if (n != sizeof(frame)) continue;

    g_total_frames++;
    uint32_t id = frame.can_id & CAN_SFF_MASK;
    uint8_t dlc = frame.can_dlc;

    if ((id & 0x7F0) == 0x640 || (id & 0x7F0) == 0x5C0) {
      decode_zeroerr(id, frame.data, dlc);
    } else if (id == 0x03 || id == 0x103) {
      decode_cylinder(id, frame.data, dlc);
    } else if ((id & 0x7F0) == 0x140) {
      decode_rmd(id, frame.data, dlc);
    } else if (id == 0x7FF) {
      event("ESTOP!");
    } else {
      g_unknown_frames++;
      if (g_unknown_frames <= 5) {
        event("UNKNOWN ID=0x%03X", id);
      }
    }

    // 定期摘要
    double t = now_sec();
    if (t - g_last_summary_time >= 5.0) {
      print_summary();
      g_last_summary_time = t;
    }
  }

  print_summary();
  if (g_logfile) { fclose(g_logfile); }
  ::close(fd);
  printf("\n=== can0 monitor stopped ===\n");
  return 0;
}