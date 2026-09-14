#pragma once
// ============================================================================
// logger.h —— 轻量级日志模块（header-only）
// 供全工程输出统一格式的运行日志：[级别] 消息
// LDEBUG/LINFO 输出到 stdout，LWARN/LERROR 输出到 stderr
// 线程安全不做要求（本工程为单线程流水线）
// ============================================================================

#include <iostream>
#include <string>

namespace common {

// 日志级别枚举：调试 / 信息 / 警告 / 错误
enum LogLevel {
    LDEBUG = 0,  // 调试信息，默认不显示
    LINFO  = 1,  // 常规运行信息
    LWARN  = 2,  // 警告（流程可继续）
    LERROR = 3   // 错误（流程通常中止）
};

// 当前全局日志阈值（低于该级别的消息被丢弃）
// 注：inline 变量要求 C++17，本工程已启用
inline LogLevel g_logLevel = LINFO;

// 设置全局日志阈值
// @param lv 新的日志级别阈值
inline void SetLogLevel(LogLevel lv) {
    g_logLevel = lv;
}

// 将日志级别转为中文字符串
// @param lv 日志级别
// @return 对应的级别名称字符串
inline const char* LogLevelName(LogLevel lv) {
    switch (lv) {
        case LDEBUG: return "DEBUG";
        case LINFO:  return "INFO";
        case LWARN:  return "WARN";
        case LERROR: return "ERROR";
        default:     return "UNKNOWN";
    }
}

// 输出一条日志消息
// 格式：[级别] 消息。LWARN/LERROR 输出到 stderr，其余输出到 stdout
// @param lv  本条消息的日志级别
// @param msg 消息内容（中文）
inline void LogMsg(LogLevel lv, const std::string& msg) {
    if (lv < g_logLevel) {
        return;
    }
    std::ostream& os = (lv >= LWARN) ? std::cerr : std::cout;
    os << "[" << LogLevelName(lv) << "] " << msg << std::endl;
}

}  // namespace common
