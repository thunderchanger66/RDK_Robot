#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp> // 使用 Point 消息类型
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <iostream>

// 定义 UPIXELS 协议的数据结构
struct UpixelsOpticalFlowDistance {
    int16_t flow_x_integral; // X 像素点累计时间内的累加位移 (radians*10000)
    int16_t flow_y_integral; // Y 像素点累计时间内的累加位移 (radians*10000)
    uint16_t integration_timespan; // 上一次发送光流数据到本次发送光流数据的累计时间 (us)
    uint16_t ground_distance; // 预留，默认为 999 (0x03E7)
    uint8_t valid; // 状态值：0 (0x00) 表示光流数据不可用，245 (0xF5) 表示光流数据可用
    uint8_t version; // 光流模块的版本号
};

UpixelsOpticalFlowDistance up_data;

// UPIXELS 协议解析函数
int16_t up_parse_char(uint8_t ch) {
    static int state = 0, pos = 0;
    static uint8_t xor_calculated = 0, xor_received = 0;
    static uint8_t buffer[14]; // 包括包头、数据长度、10 字节数据、校验值、包尾

    switch (state) {
        case 0: // 等待包头
            if (ch == 0xFE) {
                state = 1;
            }
            break;
        case 1: // 等待数据长度
            if (ch == 0x0A) {
                state = 2;
                pos = 0;
                xor_calculated = 0;
            } else {
                state = 0;
            }
            break;
        case 2: // 接收光流数据结构体
            buffer[pos++] = ch;
            xor_calculated ^= ch;
            if (pos == 10) {
                state = 3;
            }
            break;
        case 3: // 接收校验值
            xor_received = ch;
            state = 4;
            break;
        case 4: // 接收包尾
            if (ch == 0x55 && xor_calculated == xor_received) {
                memcpy(&up_data, buffer, sizeof(UpixelsOpticalFlowDistance));
                state = 0;
                return 0; // 解析成功
            } else {
                state = 0;
            }
            break;
        default:
            state = 0;
            break;
    }
    return -1; // 未完成解析
}

// 打开串口
int open_serial_port(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B460800); // 设置波特率
    cfsetispeed(&tty, B460800);

    tty.c_cflag &= ~PARENB; // 无校验位
    tty.c_cflag &= ~CSTOPB; // 1 个停止位
    tty.c_cflag &= ~CSIZE;  // 清除数据位掩码
    tty.c_cflag |= CS8;     // 8 位数据
    tty.c_cflag &= ~CRTSCTS; // 关闭硬件流控制
    tty.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略控制线

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控制
    tty.c_iflag &= ~(ICRNL | INLCR); // 禁用回车换行转换

    tty.c_lflag &= ~ICANON; // 禁用规范模式
    tty.c_lflag &= ~ECHO; // 关闭回显
    tty.c_lflag &= ~ISIG; // 禁用信号字符

    tty.c_oflag &= ~OPOST; // 禁用输出处理

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

class UPIXELSNode : public rclcpp::Node {
public:
    UPIXELSNode() : Node("upixels_node") {
        // 获取串口路径参数
        std::string port = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");

        // 创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("upixels_flow", 10);

        // 打开串口
        fd_ = open_serial_port(port.c_str());
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            exit(1);
        }
    }

    ~UPIXELSNode() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    void run() {
        uint8_t buffer[1024];
        int bytes_read;

        while (rclcpp::ok()) {
            bytes_read = read(fd_, buffer, sizeof(buffer));
            if (bytes_read < 0) {
                perror("read");
                break;
            }

            for (int i = 0; i < bytes_read; i++) {
                if (up_parse_char(buffer[i]) == 0) {
                    // 解析成功，发布数据
                    if (up_data.valid == 0xF5)
                    {
                        geometry_msgs::msg::Point msg;
                        double dx = up_data.flow_x_integral / 10000.0 * 50; // 本次位移mm
                        double dy = up_data.flow_y_integral / 10000.0 * 50;
                        total_x += dx;
                        total_y += dy;

                        msg.x = total_x;
                        msg.y = total_y;

                        publisher_->publish(msg);
                        RCLCPP_INFO(this->get_logger(), "Publish flow point: [x=%.3f, y=%.3f]", msg.x, msg.y);
                    }
                    
                }
            }
        }
    }

private:
    int fd_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    double total_x;
    double total_y;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UPIXELSNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}