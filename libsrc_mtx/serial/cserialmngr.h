#ifndef CSERIALMNGR_H
#define CSERIALMNGR_H

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <functional>
#include <atomic> // <-- 【新增】 包含 atomic
#include <mutex>
#include <cstring> // for strerror
#include <cerrno>  // for errno
#include <chrono>

/**
 * @class SerialManager
 * @brief 管理串口通信（I/O、数据帧解析、发送）的单例类
 *
 * 该类封装了串口的打开、配置、读取线程以及数据帧解析。
 * 它使用事件驱动模型，通过回调函数向上层应用通知已解析的完整数据帧。
 */
class SerialManager {
public:
    /**
     * @brief 帧数据回调函数类型
     * @param cmd 命令码 (从数据帧中解析)
     * @param data 数据负载 (Payload)
     */
    typedef std::function<void(uint16_t cmd, const std::vector<uint8_t>& data)> FrameCallback;

    /**
     * @brief 获取 SerialManager 的单例实例
     * @return SerialManager* 实例指针
     */
    static SerialManager *GetInstance();

    /**
     * @brief 析构函数，停止线程并关闭串口
     */
    ~SerialManager();

    // 禁用拷贝构造和赋值操作
    SerialManager(const SerialManager&) = delete;
    SerialManager& operator=(const SerialManager&) = delete;

    /**
     * @brief 初始化串口
     *
     * 打开指定串口设备，配置串口参数（115200, 8N1），并启动读取线程。
     *
     * @param port_name 串口设备名称 (例如 "/dev/ttySLB0")
     * @return bool true 初始化成功, false 初始化失败
     */
    // bool init(const std::string& port_name, FrameCallback cb);  <-- 旧
    bool init(const std::string& port_name); // <-- 新


    /**
     * @brief 【新增】 注册或更新帧数据回调函数
     *
     * 线程安全。您可以在任何时候调用此函数来设置或更改回调。
     * @param cb 回调函数。传入 nullptr 可以取消注册。
     */
    void registerCallback(FrameCallback cb);


    /**
     * @brief 停止通信
     *
     * 向读取线程发送停止信号，等待其退出，并关闭串口文件描述符。
     */
    void stop();

    /**
     * @brief 发送指令（带数据负载）
     *
     * 根据新协议（0x5808帧头, CRC16/MODBUS）构建并发送数据帧。
     * 帧序号会自动递增。
     *
     * @param cmd 2字节命令码
     * @param data 数据负载 (payload)
     */
    void sendCommand(uint16_t cmd, const std::vector<uint8_t>& data);
    
    bool addMirrorPort(const std::string& mirror_port_name);
    void sendCommandToMirror(uint16_t cmd, const std::vector<uint8_t>& data);

    void removeMirrorPort();

private:
    /**
     * @brief 私有构造函数 (单例模式)
     */
    SerialManager();

    /**
     * @brief 配置串口参数
     *
     * 设置为 115200 波特率, 8N1, 原始模式, 阻塞读取。
     */
    void configurePort();

    /**
     * @brief 串口数据读取线程的执行函数
     *
     * 循环阻塞读取串口数据，并将数据喂给内部解析器。
     */
    void readLoop();

    /**
     * @brief 【新增】 内部函数，由解析器在解析成功时调用
     *
     * 负责安全地调用用户注册的回调。
     */
    void onFrameParsed(uint16_t cmd, const std::vector<uint8_t>& data);

    /**
     * @brief 计算 CRC16/MODBUS
     * @param data 数据指针
     * @param length 数据长度
     * @return uint16_t CRC校验码
     */
    static uint16_t crc16_modbus(const uint8_t *data, size_t length);

    // --- 内部帧解析器 ---
    /**
     * @class InternalFrameParser
     * @brief 内部状态机，用于解析新的串口协议
     */
    class InternalFrameParser {
    public:
        // 使用 ParserCallback 以区别于 SerialManager 的 FrameCallback
        typedef std::function<void(uint16_t, const std::vector<uint8_t>&)> ParserCallback;
        
        InternalFrameParser() : state(State::HEADER_1), last_byte_time_(std::chrono::steady_clock::now()) {}
        
        /**
         * @brief 设置解析成功后的回调
         */
        void set_callback(ParserCallback cb) { m_parser_callback = cb; }
        
        /**
         * @brief 处理传入的字节流
         */
        void process_bytes(const std::vector<uint8_t>& data);

    private:
        // 解析器状态
        enum class State {
            HEADER_1, HEADER_2,
            SEQ_1, SEQ_2,
            CMD_1, CMD_2,
            LEN_1, LEN_2,
            PAYLOAD,
            CRC_1, CRC_2
        };

        State state;
        ParserCallback m_parser_callback; // <-- 新
        std::vector<uint8_t> m_buffer;         // 存储正在接收的完整帧（用于CRC）
        std::vector<uint8_t> m_payload_buffer; // 存储数据负载
        uint16_t m_current_cmd = 0;
        uint16_t m_payload_len = 0;
        size_t m_payload_cnt = 0;

        std::chrono::steady_clock::time_point last_byte_time_;
        /**
         * @brief 重置状态机
         */
        void reset_parser();

        /**
         * @brief 处理一帧完整的数据（校验CRC并回调）
         * @param frame_buffer 包含完整帧（含CRC）的缓冲区
         */
        void handle_frame(const std::vector<uint8_t>& frame_buffer);
    };

    static SerialManager *m_instance;   // 单例实例
    static std::mutex m_instance_mutex; // 保护实例创建
    std::mutex m_write_mutex;           // 保护串口写入操作
    std::mutex m_mirror_write_mutex;
    
    std::mutex m_callback_mutex;        // 保护 m_callback
    std::atomic<uint16_t> m_frame_seq;  // 帧序号 (线程安全)


    int fd;                             // 文件描述符
    std::atomic<bool> m_running;        // 线程运行标志
    std::thread m_read_thread;          // 读取线程
    FrameCallback m_callback;           // 上层回调
    InternalFrameParser m_parser;       // 内部解析器实例

    int m_mirror_fd;                    // 镜像串口的文件描述符
    void configureMirrorPort(int port_fd); // 配置镜像串口的私有函数
};

#endif // CSERIALMNGR_H

