#pragma once

#include <cstddef>
#include <ostream>
#include <stdexcept>
#include <string>

namespace serial {

// RingBuffer<T> —— 环形缓冲区（也叫循环队列）。
//
// 它解决什么问题？串口收数据往往是「一波一波」来的，处理可能跟不上。
// 我们就开一块固定大小的数组当「临时仓库」：数据来了先存着（push），
// 处理线程慢慢取走（pop）。环形 = 写满后绕回开头继续用，不搬数据。
//
// 为什么用模板 <typename T>？因为仓库里想存什么类型都行：
// 字节（uint8_t）、帧（Frame）都能用同一套逻辑，不用为每种类型重写一遍。
// 模板是「编译期复制粘贴」：你用 RingBuffer<int> 时，编译器就按 int 生成一份代码。
template <typename T>
class RingBuffer {
public:
    // 构造函数：开一块能装 capacity 个 T 的数组。
    // explicit 防止 int 被偷偷当成 RingBuffer 去隐式构造。
    explicit RingBuffer(std::size_t capacity) {
        // 先校验参数：容量为 0 是编程错误（后面取模会乱），在分配内存之前就抛异常，
        // 比「先 new 再发现参数错」更直白，也不会出现 new T[0] 这种让人困惑的写法。
        if (capacity == 0) {
            throw std::invalid_argument("RingBuffer: capacity must be > 0");
        }
        capacity_ = capacity;
        data_ = new T[capacity];   // 在堆上开数组（裸 new，必须在析构里 delete[] 配对）
        head_ = 0;                 // 读位置：下一个被取走的元素下标
        tail_ = 0;                 // 写位置：下一个被写入的格子下标
        size_ = 0;                 // 当前有效元素个数
    }

    // 析构函数：对象销毁时释放数组。这就是「RAII」——资源跟着对象生命周期走，不用手动管。
    ~RingBuffer() {
        delete[] data_;
    }

    // 拷贝构造函数：用另一个对象 other 来构造自己。
    // 必须「深拷贝」= 自己重新开一块数组，把 other 里的元素一个个复制过来，
    // 这样两个缓冲区互不影响（改一个不会动另一个）。
    // 为什么不能浅拷贝（直接拷指针）？那样两个对象会指向同一块内存，
    // 一个析构把内存释放了，另一个再用就崩溃（经典 double free）。
    RingBuffer(const RingBuffer& other) {
        capacity_ = other.capacity_;
        data_ = new T[capacity_];
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        for (std::size_t i = 0; i < other.size_; ++i) {
            // 按「从旧到新」的顺序复制，保证下标语义一致
            push(other.data_[(other.head_ + i) % other.capacity_]);
        }
    }

    // 拷贝赋值运算符：把 another 的内容复制给自己。
    // 套路：先判断「是不是自己赋值给自己」（a = a 不能崩），再释放旧内存、拷新内容。
    RingBuffer& operator=(const RingBuffer& another) {
        if (this == &another) {
            return *this;   // 自赋值，直接返回
        }
        delete[] data_;     // 先释放自己原来的数组
        capacity_ = another.capacity_;
        data_ = new T[capacity_];
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        for (std::size_t i = 0; i < another.size_; ++i) {
            push(another.data_[(another.head_ + i) % another.capacity_]);
        }
        return *this;
    }

    // 写入一个元素。满了返回 false（由调用方决定：是丢新数据还是丢旧数据）。
    bool push(const T& value) {
        if (full()) {
            return false;
        }
        data_[tail_] = value;
        tail_ = (tail_ + 1) % capacity_;   // 写指针后移，到末尾就绕回 0（这就是「环形」）
        ++size_;
        return true;
    }

    // 取走一个元素。空时返回 false——「暂时没数据」是正常流程，不是错误，所以不用抛异常。
    bool pop(T& out) {
        if (empty()) {
            return false;
        }
        out = data_[head_];
        head_ = (head_ + 1) % capacity_;   // 读指针后移，到末尾绕回 0
        --size_;
        return true;
    }

    // 看一眼最旧/最新的元素，但不取走
    T& front() { return at_logical(0); }
    const T& front() const { return at_logical(0); }
    T& back() { return at_logical(size_ - 1); }
    const T& back() const { return at_logical(size_ - 1); }

    // 逻辑下标：buf[0] 永远是最旧的元素，buf[size()-1] 是最新的。
    // 这样外部代码不用关心 head_/tail_ 现在绕到哪了。
    T& operator[](std::size_t index) { return at_logical(index); }
    const T& operator[](std::size_t index) const { return at_logical(index); }

    void clear() { head_ = tail_ = size_ = 0; }

    std::size_t size() const { return size_; }
    std::size_t capacity() const { return capacity_; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ >= capacity_; }

    // 重载 << 运算符，让 std::cout << buf 能直接打印内容（调试时用，方便看）。
    friend std::ostream& operator<<(std::ostream& os, const RingBuffer& rb) {
        os << "RingBuffer(" << rb.size_ << "/" << rb.capacity_ << ", [";
        for (std::size_t i = 0; i < rb.size_; ++i) {
            if (i != 0) {
                os << ", ";
            }
            os << rb.data_[(rb.head_ + i) % rb.capacity_];
        }
        os << "])";
        return os;
    }

private:
    // 统一的「逻辑下标 -> 真实下标」换算，并做越界检查。
    // 越界说明调用方逻辑错了（比如对空缓冲区取 front），必须让它知道，所以抛异常。
    T& at_logical(std::size_t index) {
        if (index >= size_) {
            throw std::out_of_range("RingBuffer: index out of range");
        }
        return data_[(head_ + index) % capacity_];
    }
    const T& at_logical(std::size_t index) const {
        if (index >= size_) {
            throw std::out_of_range("RingBuffer: index out of range");
        }
        return data_[(head_ + index) % capacity_];
    }

    T* data_;               // 环形数组（在堆上）
    std::size_t capacity_;  // 总容量
    std::size_t head_;      // 读位置
    std::size_t tail_;      // 写位置
    std::size_t size_;      // 有效元素个数（用它区分「空」和「满」，否则 head_==tail_ 有歧义）
};

}  // namespace serial
