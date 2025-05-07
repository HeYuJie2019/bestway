#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "SVNETSDK.h"
#include <vector>
#include <cstring>
#include <unistd.h>

class TemperaturePublisher : public rclcpp::Node
{
public:
    TemperaturePublisher() : Node("temperature_publisher")
    {
        // 创建发布者
        temp_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("temperature_matrix", 10);

        // 初始化设备
        if (init_lib() != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize SDK");
            return;
        }

        // 登录设备
        if (login_device() != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to login to device");
            return;
        }

        // 开始实时播放并设置回调
        start_real_play();
    }

    ~TemperaturePublisher()
    {
        // 停止实时播放
        if (play_handle_ >= 0)
        {
            SV_NET_DEV_StopRealPlay(play_handle_, nullptr);
        }

        // 注销设备
        if (device_login_id_ >= 0)
        {
            SV_NET_DEV_Logout(device_login_id_);
        }

        // 释放 SDK
        // SV_NET_DEV_SDK_Cleanup(); // Ensure the correct header file is included or replace this with the appropriate cleanup function from the SDK.
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr temp_publisher_;
    LONG device_login_id_ = -1;
    LONG play_handle_ = -1;

    int init_lib()
    {
        if (!SV_NET_DEV_SDK_Init())
        {
            RCLCPP_ERROR(this->get_logger(), "SDK initialization failed");
            return -1;
        }
        return 0;
    }

    int login_device()
    {
        SV_NET_DEV_USER_LOGIN_INFO login_info = {0};
        std::strcpy(login_info.sDeviceAddress, "192.168.2.64");
        login_info.wPort = 8000;
        std::strcpy(login_info.sUserName, "admin");
        std::strcpy(login_info.sPassword, "ipc12345");

        device_login_id_ = SV_NET_DEV_Login(&login_info, nullptr);
        if (device_login_id_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Device login failed");
            return -1;
        }

        RCLCPP_INFO(this->get_logger(), "Device login successful");
        return 0;
    }

    void start_real_play()
    {
        SV_NET_DEV_PREVIEWINFO preview_info = {0};
        preview_info.lChannel = 0;
        preview_info.hPlayWnd = 0;
        preview_info.byVideoCodingType = 1; // 温度数据
        preview_info.dwLinkMode = 4;

        play_handle_ = SV_NET_DEV_RealPlay(device_login_id_, &preview_info, fThmlDataCallBack, this);
        if (play_handle_ < 0)
        {
            int err = SV_NET_DEV_GetLastError();
            RCLCPP_ERROR(this->get_logger(), "RealPlay failed, error code: %d", err);
        }
    }

    static void CALLBACK fThmlDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD unWidth, DWORD unHeight, void *pUser)
    {
        auto node = static_cast<TemperaturePublisher *>(pUser);
        if (dwDataType == TYPE_THML16)
        {
            signed short *ptrS16TempVal = (signed short *)pBuffer;

            // 构造温度矩阵消息
            std_msgs::msg::Float32MultiArray temp_msg;
            temp_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            temp_msg.layout.dim[0].label = "rows";
            temp_msg.layout.dim[0].size = unHeight;
            temp_msg.layout.dim[0].stride = unHeight * unWidth;
            temp_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            temp_msg.layout.dim[1].label = "cols";
            temp_msg.layout.dim[1].size = unWidth;
            temp_msg.layout.dim[1].stride = unWidth;

            temp_msg.data.resize(unHeight * unWidth);

            int tempVal = 0;
            int minVal = 16383;
            int maxVal = -1000;
            for (int i = 0; i < unHeight; i+=2)
            {
                for (int j = 0; j < unWidth; j+=2)
                {
                    temp_msg.data[i * unWidth + j] = static_cast<float>(ptrS16TempVal[i * unWidth + j]);
                    if (ptrS16TempVal[i * unWidth + j] > maxVal)
                    {
                        maxVal = ptrS16TempVal[i * unWidth + j];
                    }
                    if (ptrS16TempVal[i * unWidth + j] < minVal)
                    {
                        minVal = ptrS16TempVal[i * unWidth + j];
                    }
                }
            }
            RCLCPP_INFO(node->get_logger(), "Max Temp: %d, Min Temp: %d", maxVal, minVal);
            // 发布温度矩阵
            node->temp_publisher_->publish(temp_msg);
        }
        else if (dwDataType == TYPE_MTRGN)
        {
            SV_NET_DEV_THMLMTRULE_PARAM_V12 strThmlMtRuleParamV12 = {0};
            int ret = SV_NET_DEV_GetThmlMtRuleListConfig_V13(pBuffer, 0, &strThmlMtRuleParamV12);
            if (ret == 0)
            {
                RCLCPP_INFO(node->get_logger(), "Temperature data: %f %f %f",
                            strThmlMtRuleParamV12.outTemp[0],
                            strThmlMtRuleParamV12.outTemp[1],
                            strThmlMtRuleParamV12.outTemp[2]);
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperaturePublisher>());
    rclcpp::shutdown();
    return 0;
}