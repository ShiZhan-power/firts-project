#include "cserialmngr.h"
#include <stdio.h>
#include <atomic>

// 初始化单例指针和互斥锁
SerialManager* SerialManager::m_instance = nullptr;
std::mutex SerialManager::m_instance_mutex;

SerialManager *SerialManager::GetInstance() {
    // 使用双重检查锁定（DCLP）确保线程安全
    if (m_instance == nullptr) {
        std::lock_guard<std::mutex> lock(m_instance_mutex);
        if (m_instance == nullptr) {
            m_instance = new SerialManager();
        }
    }
    return m_instance;
}

// 构造函数，初始化帧序号
SerialManager::SerialManager() : fd(-1), m_running(false), m_frame_seq(0), m_mirror_fd(-1) {
    // 构造函数
}

SerialManager::~SerialManager() {
    stop();
}

bool SerialManager::init(const std::string& port_name) {
    if (fd != -1) {
        printf("Serial port already initialized.\n");
        return true;
    }

    // O_RDWR (读写) | O_NOCTTY (非控制终端)
    // 注意：我们使用阻塞I/O，因此不设置 O_NONBLOCK
    fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("打开串口失败");
        return false;
    }

    m_callback = nullptr;
    
    // 设置解析器的回调为内部的 onFrameParsed 函数
    // 使用 lambda 捕获 this 指针
    m_parser.set_callback([this](uint16_t cmd, const std::vector<uint8_t>& data) {
        this->onFrameParsed(cmd, data);
    });

    configurePort(); // 配置串口参数

    // 启动读取线程
    m_running = true;
    m_read_thread = std::thread(&SerialManager::readLoop, this);

    printf("串口 %s 初始化成功。\n", port_name.c_str());
    return true;
}

/**
 * @brief 注册回调函数
 */
void SerialManager::registerCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    m_callback = cb;
    // printf("回调函数已注册。\n");
}

/**
 * @brief 内部帧处理函数（由解析器调用）
 */
void SerialManager::onFrameParsed(uint16_t cmd, const std::vector<uint8_t>& data) {
    // 锁住互斥锁，以防 m_callback 在此时被 registerCallback 更改
    std::lock_guard<std::mutex> lock(m_callback_mutex);
    
    // 安全检查：确保回调函数已经被注册
    if (m_callback) {
        try {
            m_callback(cmd, data);
        } catch (const std::exception& e) {
            printf("回调函数中发生异常: %s\n", e.what());
        } catch (...) {
            printf("回调函数中发生未知异常。\n");
        }
    }
    // 如果 m_callback 为空，则不执行任何操作，保证安全
}

void SerialManager::stop() {
    m_running = false; // 通知读取线程停止

    if (m_read_thread.joinable()) {
        m_read_thread.join(); // 等待读取线程退出
    }

    removeMirrorPort();
    
    if (fd != -1) {
        close(fd);
        fd = -1;
        printf("串口已关闭。\n");
    }
}

void SerialManager::configurePort() {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        perror("获取串口属性失败");
        if (fd != -1) close(fd);
        fd = -1; // 标记失败
        return;
    }

    // 设置为原始模式 (Raw Mode)
    cfmakeraw(&tty);

    // 配置波特率为 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1 (8 数据位, 无校验, 1 停止位)
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 数据位
    tty.c_cflag &= ~PARENB;                     // 无校验位
    tty.c_cflag &= ~CSTOPB;                     // 1 停止位
    
    // 开启接收和本地连接
    tty.c_cflag |= (CLOCAL | CREAD);            

    // 禁用硬件/软件流控
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 设置为阻塞读取 (线程模型的关键)
    // read() 将阻塞，直到至少读取了 VMIN 字节
    tty.c_cc[VMIN] = 1;   // 至少读取 1 字节
    tty.c_cc[VTIME] = 0;  // 无超时

    // 应用修改的属性
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("设置串口属性失败");
    }

    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);
}

void SerialManager::readLoop() {
    uint8_t buffer[256];
    std::vector<uint8_t> vec_buffer;

    // printf("读取线程开始...\n");

    while(m_running) {
        // 阻塞式读取
        ssize_t bytes_read = read(fd, buffer, sizeof(buffer));

        if (bytes_read > 0) {
            // 如果启用了镜像端口，写入数据
            if (m_mirror_fd != -1) {
                std::lock_guard<std::mutex> lock(m_mirror_write_mutex);
                ssize_t bytes_written = write(m_mirror_fd, buffer, bytes_read);
                if (bytes_written < bytes_read) {
                    printf("警告: 镜像串口写入未完整 (可能缓冲区已满)\n");
                }
            }

            // 将读取到的数据喂给解析器
            vec_buffer.assign(buffer, buffer + bytes_read);
            m_parser.process_bytes(vec_buffer);
        } else if (bytes_read == 0) {
            // 理论上在 VMIN=1 时不应发生，除非串口断开
            printf("读取到 0 字节，串口可能已断开。\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            // read < 0
            if (m_running) { // 避免在 stop() 刚调用后报告错误
                if (errno != EINTR) { // EINTR (中断) 是正常的，其他是错误
                    perror("串口读取错误");
                    m_running = false; // 发生错误，停止循环
                }
            }
        }
    }
    printf("读取线程退出。\n");
}

void SerialManager::sendCommand(uint16_t cmd, const std::vector<uint8_t>& data) {
    if (fd == -1 || !m_running) {
        printf("发送失败：串口未初始化或已停止。\n");
        return;
    }

    uint16_t data_len = data.size();
    // if (data_len % 2 != 0) {
    //     printf("发送失败：数据负载长度为 %zu (奇数)，必须为偶数。\n", data.size());
    //     return;
    // }

    // 帧头(2) + 序号(2) + 命令码(2) + 长度(2) + 数据(N) + CRC(2)
    size_t frame_size = 2 + 2 + 2 + 2 + data_len + 2;
    std::vector<uint8_t> frame(frame_size);

    size_t i = 0;
    // 帧头 (0x5808)
    frame[i++] = 0x08;
    frame[i++] = 0x58;

    // 帧序号 (递增)
    uint16_t seq = m_frame_seq++; 
    frame[i++] = seq & 0xFF;        // 低位
    frame[i++] = (seq >> 8) & 0xFF; // 高位
    
    // 命令码 (位在前)
    frame[i++] = cmd & 0xFF;
    frame[i++] = (cmd >> 8) & 0xFF;
    
    // 数据长度 (位在前)
    frame[i++] = data_len & 0xFF;
    frame[i++] = (data_len >> 8) & 0xFF;
    
    // 数据内容
    for(uint8_t byte : data) {
        frame[i++] = byte;
    }

    // 计算CRC (从帧头到数据内容)
    uint16_t crc = crc16_modbus(frame.data(), frame_size - 2);
    // printf("CRC: 0x%04X\n", crc);
    // CRC (低字节在前)
    frame[i++] = crc & 0xFF;
    frame[i++] = (crc >> 8) & 0xFF;

    // 线程安全写入
    ssize_t written = write(fd, frame.data(), frame.size());
    
    if (written == -1) {
        perror("写入串口失败");
    } else if (written < (ssize_t)frame.size()) {
        printf("警告：串口写入未完整 (%zd / %zu 字节)\n", written, frame.size());
    }
}

// -----------------------------------------------------------------
// InternalFrameParser 实现
// -----------------------------------------------------------------

void SerialManager::InternalFrameParser::reset_parser() {
    state = State::HEADER_1;
    m_buffer.clear();
    m_payload_buffer.clear();
    m_payload_len = 0;
    m_payload_cnt = 0;
    m_current_cmd = 0;
    last_byte_time_ = std::chrono::steady_clock::now();
}

void SerialManager::InternalFrameParser::process_bytes(const std::vector<uint8_t>& data) {
    auto now = std::chrono::steady_clock::now();
    
    // 仅当 FSM 不处于空闲状态 (HEADER_1) 时才检查超时
    if (state != State::HEADER_1) {
        // 计算自上一个字节（或上一个数据包）以来的时间
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_byte_time_);
        
        // 假设 50 毫秒为帧间超时阈值
        if (elapsed.count() > 50) { 
            // printf("Parser timeout (%lld ms). Resetting state machine.\n", elapsed.count());
            reset_parser();
        }
    }
    
    // 收到新数据（或即将处理新数据），更新时间戳
    last_byte_time_ = now;

    for(uint8_t byte : data) {
        switch(state) {
            case State::HEADER_1:
                if(byte == 0x08) {
                    reset_parser(); // 确保从干净的状态开始
                    m_buffer.push_back(byte);
                    state = State::HEADER_2;
                }
                break;
            case State::HEADER_2:
                if(byte == 0x58) {
                    m_buffer.push_back(byte);
                    state = State::SEQ_1;
                } else {
                    reset_parser();
                }
                break;
            case State::SEQ_1: // 帧序号1
                m_buffer.push_back(byte);
                state = State::SEQ_2;
                break;
            case State::SEQ_2: // 帧序号2
                m_buffer.push_back(byte);
                state = State::CMD_1;
                break;
            case State::CMD_1: // 命令码1 (di位)
                m_buffer.push_back(byte);
                // m_current_cmd = (uint16_t)byte << 8;
                m_current_cmd = (uint16_t)byte;
                state = State::CMD_2;
                break;
            case State::CMD_2: // 命令码2 (gao位)
                m_buffer.push_back(byte);
                // m_current_cmd |= byte;
                m_current_cmd |= (uint16_t)byte << 8;
                state = State::LEN_1;
                break;
            case State::LEN_1: // 长度 1 (高位)
                m_buffer.push_back(byte);
                // m_payload_len = (uint16_t)byte << 8;
                m_payload_len = (uint16_t)byte;
                state = State::LEN_2;
                break;
            case State::LEN_2: // 长度 2 (低位)
                m_buffer.push_back(byte);
                // m_payload_len |= byte;
                m_payload_len |= (uint16_t)byte << 8;
                m_payload_cnt = 0;
                
                if (m_payload_len == 0) {
                    // printf("数据长度为0,跳过数据段直接读CRC\n");
                    state = State::CRC_1;
                } else if (m_payload_len > 2) {
                    printf("错误,协议里没写数据长度超过2的指令 (%u)\n", m_payload_len);
                    reset_parser();
                } else {
                    m_payload_buffer.reserve(m_payload_len);
                    state = State::PAYLOAD;
                }
                break;
            case State::PAYLOAD:
                m_buffer.push_back(byte);
                m_payload_buffer.push_back(byte);
                m_payload_cnt++;
                if (m_payload_cnt >= m_payload_len) {
                    state = State::CRC_1;
                }
                break;
            case State::CRC_1: // CRC 1 (低位)
                m_buffer.push_back(byte);
                state = State::CRC_2;
                break;
            case State::CRC_2: // CRC 2 (高位)
                m_buffer.push_back(byte);
                handle_frame(m_buffer);
                reset_parser();
                break;
        }
    }
}

void SerialManager::InternalFrameParser::handle_frame(const std::vector<uint8_t>& frame_buffer) {
    if (frame_buffer.size() < 10) return; // 最小帧长 (2+2+2+2+0+2)

    uint16_t received_crc = (uint16_t)frame_buffer.back() << 8; // 高位
    received_crc |= frame_buffer[frame_buffer.size() - 2];      // 低位

    size_t len_to_check = frame_buffer.size() - 2;
    uint16_t calculated_crc = crc16_modbus(frame_buffer.data(), len_to_check);

    if (received_crc == calculated_crc) {
        if (m_parser_callback) {
            m_parser_callback(m_current_cmd, m_payload_buffer);
        }
    } else {
        printf("指令CRC校验失败! R: 0x%04X, C: 0x%04X\n", received_crc, calculated_crc);
    }
}

uint16_t SerialManager::crc16_modbus(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // MODBUS 多项式
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 添加一个镜像(透传)串口
 */
bool SerialManager::addMirrorPort(const std::string& mirror_port_name) {
    // 如果已经有一个打开的，先移除它
    if (m_mirror_fd != -1) {
        removeMirrorPort();
    }

    // O_RDWR | O_NOCTTY | O_NONBLOCK (非阻塞模式适合透传)
    m_mirror_fd = open(mirror_port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); 
    
    if (m_mirror_fd == -1) {
        // perror("打开镜像(透传)串口失败");
        printf("错误: 无法打开 %s, 透传功能将禁用。\n", mirror_port_name.c_str());
        return false;
    }

    printf("镜像串口 %s 初始化成功。\n", mirror_port_name.c_str());
    // 配置镜像串口 (115200, 8N1, raw)
    configureMirrorPort(m_mirror_fd);
    return true;
}

/**
 * @brief 移除并关闭当前的镜像串口
 */
void SerialManager::removeMirrorPort() {
    if (m_mirror_fd != -1) {
        close(m_mirror_fd);
        m_mirror_fd = -1;
        printf("镜像串口已关闭。\n");
    }
}


/**
 * @brief (私有) 配置镜像串口 (非阻塞)
 */
void SerialManager::configureMirrorPort(int port_fd) {
    struct termios tty;
    if (tcgetattr(port_fd, &tty) != 0) {
        perror("获取镜像串口属性失败");
        return;
    }
    
    // 设置为 raw 模式
    cfmakeraw(&tty); 

    // 波特率 115200
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= (CLOCAL | CREAD);
    
    // 禁用流控
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // 设置为非阻塞 (VMIN=0, VTIME=0)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
        perror("设置镜像串口属性失败");
    }
    tcflush(port_fd, TCIOFLUSH);
}

void SerialManager::sendCommandToMirror(uint16_t cmd, const std::vector<uint8_t>& data) {
    if (m_mirror_fd == -1 || !m_running) {
//        printf("镜像发送失败：未添加或已停止。\n");
        return;
    }

    std::lock_guard<std::mutex> lock(m_mirror_write_mutex);
    uint16_t data_len = data.size();
    // if (data_len % 2 != 0) {
    //     printf("发送失败：数据负载长度为 %zu (奇数)，必须为偶数。\n", data.size());
    //     return;
    // }

    // 帧头(2) + 序号(2) + 命令码(2) + 长度(2) + 数据(N) + CRC(2)
    size_t frame_size = 2 + 2 + 2 + 2 + data_len + 2;
    std::vector<uint8_t> frame(frame_size);

    size_t i = 0;
    // 帧头 (0x5808)
    frame[i++] = 0x08;
    frame[i++] = 0x58;

    // 帧序号 (递增)
    uint16_t seq = m_frame_seq++; 
    frame[i++] = seq & 0xFF;        // 低位
    frame[i++] = (seq >> 8) & 0xFF; // 高位
    
    // 命令码 (位在前)
    frame[i++] = cmd & 0xFF;
    frame[i++] = (cmd >> 8) & 0xFF;
    
    // 数据长度 (位在前)
    frame[i++] = data_len & 0xFF;
    frame[i++] = (data_len >> 8) & 0xFF;
    
    // 数据内容
    for(uint8_t byte : data) {
        frame[i++] = byte;
    }

    // 计算CRC (从帧头到数据内容)
    uint16_t crc = crc16_modbus(frame.data(), frame_size - 2);
    // printf("CRC: 0x%04X\n", crc);
    // CRC (低字节在前)
    frame[i++] = crc & 0xFF;
    frame[i++] = (crc >> 8) & 0xFF;

    // 线程安全写入
    ssize_t written = write(m_mirror_fd, frame.data(), frame.size());
    
    if (written == -1) {
        // EAGAIN 或 EWOULDBLOCK 意味着缓冲区满了，这是非阻塞模式的正常现象
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
             printf("警告: 镜像发送失败 (缓冲区已满)。\n");
        } else {
            // 其他错误
            perror("镜像发送失败: 写入错误");
        }
    } else if (written < (ssize_t)data.size()) {
        printf("警告：镜像发送未完整 (%zd / %zu 字节)\n", written, data.size());
    }
}
