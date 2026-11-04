#pragma once

#include "serial/command_dispatcher.hpp"
#include "serial/error_code.hpp"
#include "serial/frame.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace serial {

// 快速造一个错误应答帧，例如 makeError(BadArgument) -> Frame("ERR,3")。
// 错误信息统一走这里，避免到处手写 "ERR," + 数字，减少拼错。
inline Frame makeError(ErrorCode code) {
    return Frame("ERR," + std::to_string(static_cast<int>(code)));
}

// 下面是每个命令的「处理函数」。签名和 CommandDispatcher::Handler 一致：输入 const Frame&，返回 Frame。
// 每个函数自己负责校验自己的参数（对应固件 ERR,3），分派器只管「查表 + 调用」。
// 为什么 MOVE/JOINT/LOG 用「命名函数」、PING 用「lambda」（见下面 registerFirmwareCommands）？
//   - 复杂命令带参数校验逻辑，写成命名函数更清晰、能复用、能单独测试；
//   - PING 只有一行、无参数，用 lambda 就地写最简洁。
//   现在 Handler 是 std::function，两种都能注册，所以按「清晰优先」来挑。

// MOVE,<ch>,<pulse>：ch 必须 0~5，pulse 必须 <= 2500（和固件 cmd_move 一致）
inline Frame handleMove(const Frame& req) {
    std::uint32_t ch = 0;
    std::uint32_t pulse = 0;
    if (!req.argUint(0, ch) || !req.argUint(1, pulse)) {
        return makeError(ErrorCode::BadArgument);   // 参数不是合法数字
    }
    if (ch >= 6 || pulse > 2500) {
        return makeError(ErrorCode::BadArgument);   // 通道越界 / 脉宽超限
    }
    return Frame("OK");
}

// JOINT,<6 路脉宽>：必须 6 路、每路 <= 2500（和固件 cmd_joint 一致）
inline Frame handleJoint(const Frame& req) {
    for (std::size_t i = 0; i < 6; ++i) {
        std::uint32_t v = 0;
        if (!req.argUint(i, v) || v > 2500) {
            return makeError(ErrorCode::BadArgument);
        }
    }
    return Frame("OK");
}

// LOG,<0|1>：只有 0 或 1 合法（和固件 cmd_log 一致）
inline Frame handleLog(const Frame& req) {
    std::uint32_t v = 0;
    if (!req.argUint(0, v) || v > 1) {
        return makeError(ErrorCode::BadArgument);
    }
    return Frame("OK");
}

// 把固件里的命令表一次性注册进去（PING / MOVE / JOINT / LOG 共 4 条）。
// 想加新命令，照着写一个 handleXxx 函数，在这里多注册一行即可。
inline void registerFirmwareCommands(CommandDispatcher& d) {
    // PING：无参数、一行式处理，用 lambda 就地注册最简洁。
    // lambda 语法：[捕获列表](参数列表) { 函数体 }。
    //   [ ] 是「捕获列表」：写 & 表示按引用捕获外部变量、= 表示按值捕获；空 [] 表示不借用任何外部变量。
    //   PING 无状态，所以用空 []。以后做「记住上次位置」这类带状态的命令，就能写 [this] 捕获成员。
    d.registerCommand("PING", [](const Frame&) { return Frame("OK,PONG"); });
    d.registerCommand("MOVE", handleMove);
    d.registerCommand("JOINT", handleJoint);
    d.registerCommand("LOG", handleLog);
}

}  // namespace serial
