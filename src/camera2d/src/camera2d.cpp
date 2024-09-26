#include "rclcpp/rclcpp.hpp"
#include "MvCameraControl.h"

class camera2d final : public rclcpp::Node
{
public:
    explicit camera2d(const std::string& name) : Node(name){
        do {
            int nRet = MV_OK;
            nRet = MV_CC_Initialize();
            if (MV_OK != nRet) {
                RCLCPP_INFO(this->get_logger(), "Initialize SDK fail! nRet [0x%x]\n", nRet);
            }
            MV_CC_DEVICE_INFO_LIST stDeviceList;
            nRet = MV_CC_EnumDevices(
                MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE,
                &stDeviceList);
            if (MV_OK != nRet) {
                RCLCPP_INFO(this->get_logger(), "MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
                break;
            }
            if (stDeviceList.nDeviceNum > 0) {
                for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
                    RCLCPP_INFO(this->get_logger(), "[device %d]:\n", i);
                    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                    if (nullptr == pDeviceInfo) {
                        break;
                    }
                    PrintDeviceInfo(pDeviceInfo);
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Find No Devices!\n");
                break;
            }
            RCLCPP_INFO(this->get_logger(), "Please Intput camera index: ");
            unsigned int nIndex = 0;
            scanf("%d", &nIndex);
            nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
            if (MV_OK != nRet) {
                RCLCPP_INFO(this->get_logger(), "MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
                break;
            }
            nRet = MV_CC_OpenDevice(handle);
            if (MV_OK != nRet) {
                RCLCPP_INFO(this->get_logger(), "MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
                break;
            }

            //ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
            if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
                if (const int nPacketSize = MV_CC_GetOptimalPacketSize(handle); nPacketSize > 0) {
                    nRet = MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", nPacketSize);
                    if (nRet != MV_OK) {
                        RCLCPP_INFO(this->get_logger(), "Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
                }
            }
            MV_CC_SetEnumValue(handle, "TriggerMode", 0);
            MV_CC_SetEnumValue(handle, "AcquisitionLineRate", 3600);
            MV_CC_SetEnumValue(handle, "HeightMax", 100);

            nRet = MV_CC_SetEnumValue(handle, "AcquisitionLineRateEnable  ", true);

            if (MV_OK != nRet) {
                RCLCPP_INFO(this->get_logger(), "MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
                break;
            }
        } while (false);
    }

    ~camera2d() override{
        if (handle != nullptr) {
            MV_CC_StopGrabbing(handle);
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            MV_CC_Finalize();
        }
    }

    void spin(){
        MV_CC_RegisterImageCallBack(handle, ImageCallBackEx, this);
        MV_CC_StartGrabbing(handle);

        while (rclcpp::ok()) {
            if (num > 20) {
                MVCC_INTVALUE_EX ar;
                MV_CC_GetIntValueEx(handle, "ResultingLineRate", &ar);
                RCLCPP_INFO(this->get_logger(), "输出图片+1,实际行频：%ld", ar.nCurValue);
                num = 0;
            }
        }
    }

    static void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO* pFrameInfo, void* pUser){
        if (pFrameInfo) {
            auto* ptr = static_cast<camera2d*>(pUser);
            RCLCPP_INFO(ptr->get_logger(), "输出图片+1,宽：%d,高：%d", pFrameInfo->nWidth, pFrameInfo->nHeight);
            ptr->num++;
        }
    }

private:
    void* handle = nullptr;
    int num;

    bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) const{
        if (nullptr == pstMVDevInfo) {
            RCLCPP_INFO(this->get_logger(), "The Pointer of pstMVDevInfo is NULL!\n");
            return false;
        }

        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            const uint8_t nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            const uint8_t nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            const uint8_t nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            const uint8_t nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
            RCLCPP_INFO(this->get_logger(), "Device Model Name: %p\n",
                        pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
            RCLCPP_INFO(this->get_logger(), "CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
            RCLCPP_INFO(this->get_logger(), "UserDefinedName: %p\n\n",
                        pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            RCLCPP_INFO(this->get_logger(), "Device Model Name: %p\n",
                        pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
            RCLCPP_INFO(this->get_logger(), "UserDefinedName: %p\n\n",
                        pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        } else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE) {
            RCLCPP_INFO(this->get_logger(), "UserDefinedName: %p\n",
                        pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
            RCLCPP_INFO(this->get_logger(), "Serial Number: %p\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
            RCLCPP_INFO(this->get_logger(), "Model Name: %p\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        } else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE) {
            RCLCPP_INFO(this->get_logger(), "UserDefinedName: %p\n",
                        pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
            RCLCPP_INFO(this->get_logger(), "Serial Number: %p\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
            RCLCPP_INFO(this->get_logger(), "Model Name: %p\n\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);
        } else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE) {
            RCLCPP_INFO(this->get_logger(), "UserDefinedName: %p\n",
                        pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
            RCLCPP_INFO(this->get_logger(), "Serial Number: %p\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
            RCLCPP_INFO(this->get_logger(), "Model Name: %p\n\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);
        } else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE) {
            RCLCPP_INFO(this->get_logger(), "UserDefinedName: %p\n",
                        pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
            RCLCPP_INFO(this->get_logger(), "Serial Number: %p\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
            RCLCPP_INFO(this->get_logger(), "Model Name: %p\n\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);
        } else {
            RCLCPP_INFO(this->get_logger(), "Not support.\n");
        }

        return true;
    }
};

int main(const int argc, char** argv){
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<camera2d>("node1");
    RCLCPP_INFO(node->get_logger(), "相机已经连接");
    node->spin();
    rclcpp::shutdown();
    return 0;
}








