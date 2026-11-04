#pragma once

#include <cstdint>

namespace serial {

// 错误码，和固件 uart_protocol.c 里 "$ERR,<code>" 的 <code> 一一对应。
// 为什么用 enum class（强类型枚举）而不是普通 enum？
//   普通 enum 会悄悄转成 int，容易误用；enum class 必须写 ErrorCode::Xxx，
//   编译器帮你挡住「把错误码当普通数字乱用」的 bug。这是新手也该养成的好习惯。
enum class ErrorCode : std::uint8_t {
    None = 0,
    UnknownCommand = 1,   // ERR,1 未知命令
    BadChecksum = 2,      // ERR,2 校验失败
    BadArgument = 3,      // ERR,3 格式/参数错误
    StateForbidden = 4,   // ERR,4 当前状态禁止该操作
};

// 把错误码变成给人看的文字（中文）。
// 为什么写成函数而不是让枚举隐式转字符串？因为枚举本身不会自动变成文字，
// 必须显式写「我要的是给人看的文字」，避免把它当整数用。
inline const char* toString(ErrorCode code) {
    switch (code) {
        case ErrorCode::None:           return "无错误";
        case ErrorCode::UnknownCommand: return "未知命令";
        case ErrorCode::BadChecksum:    return "校验失败";
        case ErrorCode::BadArgument:    return "格式/参数错误";
        case ErrorCode::StateForbidden: return "当前状态禁止该操作";
    }
    return "未知错误码";
}

}  // namespace serial
