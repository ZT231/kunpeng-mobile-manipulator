#pragma once

#include "serial/frame.hpp"

#include <cstddef>
#include <functional>
#include <string>
#include <unordered_map>

namespace serial {

// 命令分派器：把「命令名 → 处理函数」的对应关系记下来，收帧后按命令名查表并调用处理函数。
//
// 它只做两件事：按命令名查表、调用对应的处理函数。不做参数校验（那是处理函数自己的事），
// 不做「未知命令回什么」（那是调用方的决策，本类只返回 false）。
// 这样替换固件里的 if-else / switch 链之后，加新命令 = 多注册一行，不用改分派器本身。
class CommandDispatcher {
public:
    // 处理函数的类型：吃一帧请求、返回一帧应答。
    // 用 std::function 而不是裸函数指针：std::function 是「通用可调用对象包装器」，
    // 函数指针、lambda、仿函数（重载了 operator() 的对象）都能装进去，比裸函数指针灵活。
    //   裸函数指针只能指向「普通函数」，指向不了「捕获了外部变量的 lambda」；
    //   std::function 都能装，所以注册命令时可以自由用普通函数或 lambda（见 commands.hpp）。
    using Handler = std::function<Frame(const Frame&)>;

    // 注册命令。命令名重复或处理函数为空（nullptr）都返回 false（禁止静默覆盖）。
    bool registerCommand(const std::string& command, Handler handler) {
        if (command.empty() || handler == nullptr) {
            return false;
        }
        // 已存在就拒绝：count 返回 1 表示键已存在
        if (handlers_.count(command) != 0) {
            return false;
        }
        handlers_[command] = handler;
        return true;
    }

    // 注销命令；返回是否真的删掉了东西
    bool unregisterCommand(const std::string& command) {
        return handlers_.erase(command) != 0;
    }

    // 分发：按命令名找到处理函数并调用，把应答帧写进 out；找不到返回 false。
    bool dispatch(const Frame& request, Frame& out) const {
        // std::unordered_map<std::string, Handler>::const_iterator 太长，这里直接写出来，新手能看清楚类型
        std::unordered_map<std::string, Handler>::const_iterator it =
            handlers_.find(request.command());
        if (it == handlers_.end()) {
            return false;   // 调用方负责回 ERR,1（未知命令）
        }
        out = it->second(request);   // 间接调用处理函数，结果写进 out
        return true;
    }

    bool has(const std::string& command) const { return handlers_.count(command) != 0; }
    std::size_t size() const { return handlers_.size(); }
    bool empty() const { return handlers_.empty(); }

private:
    // unordered_map：哈希表，按命令名（字符串）查处理函数，平均 O(1)。
    // 比 if-else 链更利于加命令，也比线性查找快。
    std::unordered_map<std::string, Handler> handlers_;
};

}  // namespace serial
