#include "pacr_hardware_interface/pacr_hardware_interface.hpp"

#include <fcntl.h>      // O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>    // struct termios, tcgetattr, tcsetattr, baud rates
#include <unistd.h>     // write(), close(), usleep()
#include <cstring>      // memset
#include <cstdint>      // uint8_t

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pacr_hardware_interface
{

hardware_interface::CallbackReturn
PACRHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "PACRHardwareInterface on_init: minimal mode (serial test only)");

    serial_fd_ = -1;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 
PACRHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(logger_, "Configuring PACR hardware interface...");

    // Open serial port
    serial_fd_ = ::open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to open serial port");
        return hardware_interface::CallbackReturn::ERROR;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(logger_, "tcgetattr() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(logger_, "tcsetattr() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Send Roomba startup commands
    uint8_t start_cmd = 128;
    if (::write(serial_fd_, &start_cmd, 1) != 1) {
        RCLCPP_ERROR(logger_, "Failed to send START command (128)");
        return hardware_interface::CallbackReturn::ERROR;
    }
    usleep(20000);  // 20 ms delay

    uint8_t full_cmd = 132;
    if (::write(serial_fd_, &full_cmd, 1) != 1) {
        RCLCPP_ERROR(logger_, "Failed to send FULL MODE command (132)");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Roomba placed in FULL mode");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PACRHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(logger_, "Cleaning up PACR hardware interface...");

    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(logger_, "Serial port closed.");
    } else {
        RCLCPP_WARN(logger_, "Cleanup called but serial port was not open.");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PACRHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(logger_, "PACR hardware interface activated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PACRHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(logger_, "PACR hardware interface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Minimal implementations to satisfy SystemInterface
hardware_interface::return_type
PACRHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PACRHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}


std::vector<hardware_interface::StateInterface>
PACRHardwareInterface::export_state_interfaces()
{
    // Return empty vector for now (fill with your actual joints later)
    return {};
}

std::vector<hardware_interface::CommandInterface>
PACRHardwareInterface::export_command_interfaces()
{
    // Return empty vector for now (fill with your actual joints later)
    return {};
}

}  // namespace pacr_hardware_interface

PLUGINLIB_EXPORT_CLASS(pacr_hardware_interface::PACRHardwareInterface,
                       hardware_interface::SystemInterface)
