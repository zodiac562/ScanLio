#include "car_base_v2/frame_parser.h"

namespace protocol {

// 比例系数 (来自原版 origincar_base.h, 协议冻结, 物理含义:
//   1671.84    = 16384 LSB/g ÷ 9.8 m/s^2  -> ±2g 量程, 输出 m/s^2
//   0.00026644 ≈ π/(180×65.5)             -> ±500°/s 量程, 输出 rad/s)
static const float kAccelRatio = 1671.84f;
static const float kGyroRatio  = 0.00026644f;

void FrameParser::feed(const uint8_t* data, size_t len) {
  buffer_.insert(buffer_.end(), data, data + len);
  if (buffer_.size() > kBufferLimit) {
    // 防御: 解析停滞/垃圾流场景下缓冲失控, 清空重同步
    buffer_.clear();
    ++stats_.buffer_overflows;
  }
}

bool FrameParser::tryExtract(RawFrame& out) {
  for (;;) {
    // 1. 越过第一个帧头前的所有字节
    //    (初始偏移/断流残尾在此被无条件跳过, 无需知道偏移量)
    size_t begin = 0;
    while (begin < buffer_.size() && buffer_[begin] != kFrameHeader) {
      ++begin;
    }
    if (begin > 0) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + begin);
    }

    // 2. 不足一帧: 残余留在缓冲区, 与下次到达的字节拼齐 (半帧不丢失)
    if (buffer_.size() < kFrameSize) {
      return false;
    }

    // 3. 验证: 帧尾 + XOR 校验, 无例外
    //    (修复点: 原版在环形对齐分支用 || 绕过校验, 嵌合帧因此漏网)
    if (buffer_[kFrameSize - 1] == kFrameTail &&
        buffer_[kChecksumPos] == checksum(buffer_.data(), kChecksumPos)) {
      decode(buffer_.data(), out);
      buffer_.erase(buffer_.begin(), buffer_.begin() + kFrameSize);
      ++stats_.frames_parsed;
      return true;
    }

    // 4. 假帧头: 只否决这 1 个字节, 从下一字节重扫 (决策: 无终止阈值)
    buffer_.erase(buffer_.begin());
    ++stats_.candidates_rejected;
  }
}

// ---------------------------------------------------------------------------
// 以下解码逻辑逐行迁移自原版 car_base.cpp, 协议冻结, 一行不改写
// ---------------------------------------------------------------------------

// 两字节合成 16 位有符号数 (先高后低, 大端)
short FrameParser::combineBytes(uint8_t high, uint8_t low) {
  return static_cast<short>(
      static_cast<uint16_t>(static_cast<uint16_t>(high) << 8 | low));
}

// 千倍定标还原: 下位机发送 实际值×1000 取整, 拆整数部分与小数部分还原
// (数学上等价 x×0.001, 按决策保留原写法)
float FrameParser::wheelSpeed(uint8_t high, uint8_t low) {
  short raw = combineBytes(high, low);
  return static_cast<float>(raw / 1000) +
         static_cast<float>(raw % 1000) * 0.001f;
}

// 异或校验 (原版 Check_Sum 的 READ 分支)
uint8_t FrameParser::checksum(const uint8_t* data, size_t count) {
  uint8_t sum = 0;
  for (size_t i = 0; i < count; ++i) {
    sum ^= data[i];
  }
  return sum;
}

// 帧内字段布局 (协议冻结):
//  [0]帧头 [1]停止标志 [2..7]轮速x/y/z [8..13]加速度 [14..19]角速度
//  [20..21]电压 [22]校验 [23]帧尾
void FrameParser::decode(const uint8_t* f, RawFrame& out) {
  out.stop_flag = f[1];
  out.vx = wheelSpeed(f[2], f[3]);
  out.vy = wheelSpeed(f[4], f[5]);
  out.wz = wheelSpeed(f[6], f[7]);
  for (int i = 0; i < 3; ++i) {
    out.accel[i] = combineBytes(f[8 + i * 2], f[9 + i * 2]) / kAccelRatio;
    out.gyro[i]  = combineBytes(f[14 + i * 2], f[15 + i * 2]) * kGyroRatio;
  }
  out.voltage = wheelSpeed(f[20], f[21]);
}

}  // namespace protocol
