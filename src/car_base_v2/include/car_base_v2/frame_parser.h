#ifndef CAR_BASE_V2_FRAME_PARSER_H_
#define CAR_BASE_V2_FRAME_PARSER_H_

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <vector>

// ---------------------------------------------------------------------------
// 串口协议常量 (协议冻结, 不可修改) —— 归协议层所有
// ---------------------------------------------------------------------------
namespace protocol {

constexpr uint8_t kFrameHeader = 0x7B;
constexpr uint8_t kFrameTail   = 0x7D;
constexpr size_t  kFrameSize   = 24;    // 下位机 -> 上位机 帧长
constexpr size_t  kChecksumPos = 22;    // XOR 校验字节位置
constexpr size_t  kSendSize    = 11;    // 上位机 -> 下位机 帧长
constexpr size_t  kBufferLimit = 4096;  // 缓冲上限 (约170帧), 超限清空防失控

// 解码结果. 字段语义与原版 car_base 一致
struct RawFrame {
  float vx, vy, wz;     // 轮速 m/s (vy/wz 实测恒 0, 仍如实解码)
  float accel[3];       // 加速度 m/s^2, 含重力分量, 量程 ±2g
  float gyro[3];        // 角速度 rad/s, 量程 ±500°/s
  float voltage;        // 电池电压 V
  uint8_t stop_flag;    // 下位机停止标志 (原版解析后未使用, 保留)
};

// 解析统计 (决策节点5: 诊断话题数据源)
// 计数器为 atomic: 写方=读线程(feed/extract), 读方=执行器线程(诊断定时器),
// relaxed 序即可 —— 单字段一致, 字段间撕裂对诊断用途可容忍.
struct ParserStats {
  std::atomic<uint64_t> frames_parsed{0};       // 成功解析帧数
  std::atomic<uint64_t> candidates_rejected{0}; // 假帧头否决次数
  std::atomic<uint64_t> buffer_overflows{0};    // 缓冲超限清空次数
};

// ---------------------------------------------------------------------------
// 方案A: 批量搜索式解析器 (决策节点3)
//   - 两段式接口: feed 只进不出, tryExtract 只出不进
//   - 消费单位 = 验证通过的完整帧, 免疫初始偏移/错位/断流重设
//   - 帧尾 + XOR 校验无例外拒绝 (修复原版环形分支绕过校验的缺陷)
//   - 假帧头否决只损失 1 字节, 从下一字节重扫, 无终止阈值 (仅计数)
//   - 零 ROS 依赖, 读线程独占 (单线程使用, 无需锁)
// ---------------------------------------------------------------------------
class FrameParser {
public:
  void feed(const uint8_t* data, size_t len);   // 追加字节
  bool tryExtract(RawFrame& out);               // 提取一帧, 无帧返回 false
  // 断连重同步: 清空内部缓冲 (决策节点5: 断连前后是两段不连续字节流,
  // 拼接会产生假帧). 统计计数保留不清.
  void reset() { buffer_.clear(); }
  const ParserStats& stats() const { return stats_; }

private:
  // 四个转换逐行迁移自原版 car_base.cpp / origincar_base.h, 语义不变
  static short   combineBytes(uint8_t high, uint8_t low);
  static float   wheelSpeed(uint8_t high, uint8_t low);
  static uint8_t checksum(const uint8_t* data, size_t count);
  void decode(const uint8_t* frame, RawFrame& out);

  std::vector<uint8_t> buffer_;
  ParserStats stats_;
};

}  // namespace protocol

#endif  // CAR_BASE_V2_FRAME_PARSER_H_
