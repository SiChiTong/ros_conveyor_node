/*
 *  conveyor.cpp
 *  Communicate Protocol.
 *  Author: Kaka Xie
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <conveyor.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <mrobot_msgs/vci_can.h>
#include <roscan/can_long_frame.h>

#define conveyorInfo     ROS_INFO



//class Conveyor;
#define GET_SYS_STSTUS_TIME_OUT                 500//ms
#define GET_VERSION_TIME_OUT                    500//ms
#define SET_CONVEYOR_BELT_WORK_MODE_TIME_OUT    500//ms

#define GET_VERSION_RETRY_CNT                   5
#define SET_CONVEYOR_BELT_WORK_MODE_RETRY_CNT   5
void *CanProtocolProcess(void* arg)
{
    get_sys_status_t get_sys_status;
    get_version_t get_version;
    conveyor_belt_t set_conveyor_belt_mode;

    Conveyor *pConveyor =  (Conveyor*)arg;

    bool get_sys_status_flag = 0;
    bool get_version_flag = 0;
    bool set_conveyor_belt_flag = 0;
    while(ros::ok())
    {


        /* -------- get sys status protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pConveyor->get_sys_status_vector.empty())
            {
                auto a = pConveyor->get_sys_status_vector.begin();
                get_sys_status = *a;
                if(get_sys_status.reserve == 0 )
                {
                    get_sys_status_flag = 1;
                }

                pConveyor->get_sys_status_vector.erase(a);

            }

        }while(0);

        if(get_sys_status_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            get_sys_status_flag = 0;

            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("get get sys status cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pConveyor->get_sys_status_ack_vector.clear();
            }while(0);

get_sys_status_restart:

            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("get sys status:send cmd to mcu");
            }
            pConveyor->GetSysStatus(pConveyor->sys_conveyor);
#if 0
            bool get_sys_status_ack_flag = 0;
            get_sys_status_ack_t get_sys_status_ack;
            while(time_out_cnt < GET_SYS_STSTUS_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pConveyor->get_sys_status_ack_vector.empty())
                    {
                        if(pConveyor->is_log_on == true)
                        {
                            ROS_INFO("get_sys_status_vector is not empty");
                        }
                        auto b = pConveyor->get_sys_status_ack_vector.begin();
                        get_sys_status_ack = *b;
                        pConveyor->get_sys_status_ack_vector.erase(b);

                        if(pConveyor->is_log_on == true)
                        {
                            ROS_INFO("get_sys_status_ack.sys_status = %d",get_sys_status_ack.sys_status);
                        }

                        if(get_sys_status_ack.sys_status <= 0x1ff )
                        {
                            get_sys_status_ack_flag = 1;
                            if(pConveyor->is_log_on == true)
                            {
                                ROS_INFO("get right get_sys_status  ack");
                            }
                        }
                    }
                }while(0);
                if(get_sys_status_ack_flag == 1)
                {
                    get_sys_status_ack_flag = 0;
                    if(pConveyor->is_log_on == true)
                    {
                        ROS_INFO("get right get_sys_status ack, status is 0x%x",get_sys_status_ack.sys_status);
                        ROS_INFO("get get_sys_status_ack_vector data");
                    }

                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < GET_SYS_STSTUS_TIME_OUT/10)
            {
                //ROS_INFO("get sys status flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get sys status time out");
                time_out_cnt = 0;
                if(err_cnt++ < 3)
                {
                    ROS_ERROR("get sys status start to resend msg....");
                    goto get_sys_status_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with conveyor mcu, get_sys_status failed !");
                err_cnt = 0;
            }
#endif
        }
        /* -------- get sys status protocol end -------- */


        /* --------  get version protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pConveyor->get_version_vector.empty())
            {
                auto a = pConveyor->get_version_vector.begin();
                get_version = *a;
                if(get_version.get_version_type <= 3 )
                {
                    get_version_flag = 1;
                }

                pConveyor->get_version_vector.erase(a);
            }
        }while(0);

        if(get_version_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            get_version_flag = 0;

            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("get get version cmd");
            }
            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("get_version.get_version_type = 0x%x",get_version.get_version_type);
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pConveyor->get_version_ack_vector.clear();
            }while(0);

get_version_restart:
            pConveyor->sys_conveyor->get_version_type = get_version.get_version_type;
            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("get_verion:send cmd to mcu");
            }
            pConveyor->GetVersion(pConveyor->sys_conveyor);
            bool get_version_ack_flag = 0;
            get_version_ack_t get_version_ack;
            while(time_out_cnt < GET_VERSION_TIME_OUT/10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pConveyor->get_version_ack_vector.empty())
                    {
                        if(pConveyor->is_log_on == true)
                        {
                            ROS_INFO("get_version_ack_vector is not empty");
                        }
                        auto b = pConveyor->get_version_ack_vector.begin();

                        get_version_ack = *b;

                        pConveyor->get_version_ack_vector.erase(b);

                        if(pConveyor->is_log_on == true)
                        {
                            ROS_INFO("version type : %d",get_version_ack.get_version_type);
                        }
                        if( get_version_ack.get_version_type <= 3)
                        {
                            get_version_ack_flag = 1;
                            if(pConveyor->is_log_on == true)
                            {
                                ROS_INFO("get right get_version ack");
                            }
                        }
                    }
                }while(0);
                if(get_version_ack_flag == 1)
                {
                    get_version_ack_flag = 0;
                    if(pConveyor->is_log_on == true)
                    {
                        ROS_INFO("get get_version_ack_vector data");
                    }
                    if(get_version_ack.get_version_type == get_version.get_version_type)
                    {
                        if(pConveyor->is_log_on == true)
                        {
                            ROS_INFO("get right get version ack");
                        }
                        break;
                    }
                    else
                    {
                        usleep(10*1000);
                    }
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < GET_VERSION_TIME_OUT/10)
            {
                //ROS_INFO("get version flow OK");
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("get version time out");
                time_out_cnt = 0;
                if(err_cnt++ < GET_VERSION_RETRY_CNT)
                {
                    ROS_ERROR("get version start to resend msg....");
                    goto get_version_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with conveyor mcu, get version failed !");
                err_cnt = 0;
            }

        }
        /* --------  get version protocol end -------- */



        /* --------  ser conveyor belt work mode protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pConveyor->set_conveyor_belt_work_mode_vector.empty())
            {
                auto a = pConveyor->set_conveyor_belt_work_mode_vector.begin();
                set_conveyor_belt_mode = *a;
                {
                    set_conveyor_belt_flag = 1;
                }

                pConveyor->set_conveyor_belt_work_mode_vector.erase(a);

            }

        }while(0);

        if(set_conveyor_belt_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            set_conveyor_belt_flag = 0;

            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("set_conveyor_belt_mode.set_work_mode = %d",set_conveyor_belt_mode.set_work_mode);
                ROS_INFO("get set set_conveyor_belt_mode cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pConveyor->set_conveyor_belt_work_mode_vector.clear();
            }while(0);

set_conveyor_belt_work_mode_restart:
            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("set conveyor belt work mode :send cmd to mcu");
            }
            pConveyor->sys_conveyor->conveyor_belt = set_conveyor_belt_mode;

            pConveyor->set_conveyor_belt_work_mode(pConveyor->sys_conveyor);
            bool set_conveyor_belt_work_mode_ack_flag = 0;
            conveyor_belt_t set_conveyor_belt_work_mode_ack;
            while(time_out_cnt < SET_CONVEYOR_BELT_WORK_MODE_TIME_OUT / 10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pConveyor->set_conveyor_belt_work_mode_ack_vector.empty())
                    {
                        if(pConveyor->is_log_on == true)
                        {
                            ROS_INFO("set_conveyor_belt_work_mode_ack_vector is not empty");
                        }
                        auto b = pConveyor->set_conveyor_belt_work_mode_ack_vector.begin();

                        set_conveyor_belt_work_mode_ack = *b;

                        pConveyor->set_conveyor_belt_work_mode_ack_vector.erase(b);

                        if(set_conveyor_belt_work_mode_ack.set_work_mode == set_conveyor_belt_mode.set_work_mode)
                        {
                            set_conveyor_belt_work_mode_ack_flag = 1;
                            if(pConveyor->is_log_on == true)
                            {
                                ROS_INFO("get right set conveyor belt work mode ack");
                            }
                        }
                        else
                        {
                            ROS_ERROR("error: get ack set conveyor belt work mode is %d", set_conveyor_belt_work_mode_ack.set_work_mode);
                        }
                    }
                }while(0);
                if(set_conveyor_belt_work_mode_ack_flag == 1)
                {
                    set_conveyor_belt_work_mode_ack_flag = 0;
                    if(pConveyor->is_log_on == true)
                    {
                        //ROS_INFO("get right set led effect ack");
                    }
                    pConveyor->sys_conveyor->conveyor_belt = set_conveyor_belt_work_mode_ack;
                    if(pConveyor->is_log_on == true)
                    {
                        ROS_INFO("get set conveyor belt work mode:%d",set_conveyor_belt_work_mode_ack.set_work_mode);
                    }
                    if(set_conveyor_belt_work_mode_ack.err_status == 0)
                    {
                        ROS_INFO("conveyor belt status ok");
                    }
                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_LOAD_ERROR)
                    {
                        ROS_ERROR("conveyor belt load error !");
                    }
                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_UNLOAD_ERROR)
                    {
                        ROS_ERROR("conveyor belt unload error !");
                    }
                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_STATUS_ERROR)
                    {
                        ROS_ERROR("conveyor belt status error !");
                    }
                    else
                    {
                        ROS_ERROR("conveyor belt unknow error !");
                    }
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < SET_CONVEYOR_BELT_WORK_MODE_TIME_OUT / 10)
            {
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("set conveyor belt work mode time out");
                time_out_cnt = 0;
                if(err_cnt++ < SET_CONVEYOR_BELT_WORK_MODE_RETRY_CNT)
                {
                    ROS_ERROR("set remote power ctrl start to resend msg....");
                    goto set_conveyor_belt_work_mode_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with conveyor mcu, set conveyor belt work mode failed !");
                err_cnt = 0;
            }

        }
        /* -------- set conveyor belt work mode protocol end -------- */


        usleep(10 * 1000);
    }
}


int Conveyor::conveyorParamInit(void)
{
    return 0;
}



int Conveyor::GetVersion(conveyor_t *sys)      // done
{
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_VERSION;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_CONVEYOR_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = sys->get_version_type;//reserve
    this->pub_to_can_node.publish(can_msg);
    return error;
}

int Conveyor::GetSysStatus(conveyor_t *sys)     // done
{
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_SYS_STATE;
    id.CanID_Struct.SrcMACID = 0;//CAN_SUB_PB_SRC_ID;
    id.CanID_Struct.DestMACID = NOAH_CONVEYOR_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = 0;//reserve
    this->pub_to_can_node.publish(can_msg);
    return error;
}

int Conveyor::set_conveyor_belt_work_mode(conveyor_t *sys)
{
    ROS_INFO("start to set conveyor belt work mode . . . ");
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE;
    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = NOAH_CONVEYOR_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    if((sys->conveyor_belt.set_work_mode >= 0) && (sys->conveyor_belt.set_work_mode <= 2))
    {
        can_msg.Data[1] = sys->conveyor_belt.set_work_mode;
    }
    else
    {
        ROS_ERROR("set conveyor belt work mode: parameter error !  set work mode %d", sys->conveyor_belt.set_work_mode);
        return -1;
    }

    this->pub_to_can_node.publish(can_msg);
    return error;
}

void Conveyor::pub_json_msg_to_app( const nlohmann::json j_msg)
{
    std_msgs::String pub_json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    pub_json_msg.data = ss.str();
    this->noah_conveyor_pub.publish(pub_json_msg);
}

void Conveyor::from_app_rcv_callback(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("Rcv test data");
    auto j = json::parse(msg->data.c_str());
    std::string j_str = j.dump();
    ROS_WARN("%s",j_str.data());

    if(j.find("pub_name") != j.end())
    {

        //ROS_INFO("find pub_name");
        if(j["pub_name"] == "set_module_state")
        {
            boost::mutex::scoped_lock(mtx);
            if(j["data"]["dev_name"] == "_24v_printer")
            {
                if(j["data"]["set_state"] == true)
                {
                }
                else if(j["data"]["set_state"] == false)
                {
                    ROS_INFO("set 24v printer off");
                }
            }

            if(j["data"]["dev_name"] == "door_ctrl_state")
            {
                if(j["data"].find("door_id") != j["data"].end())
                {
                    ROS_INFO("start to check door id ...");
                    if(j["data"]["door_id"].empty())
                    {
                        ROS_WARN("door id value is NULL !");//
                        if(j["data"]["set_state"] == true)
                        {
                            ROS_INFO("set door ctrl  on");
                        }
                        else if(j["data"]["set_state"] == false)
                        {
                            ROS_INFO("set door ctrl off");
                        }
                    }
                    else
                    {
                        ROS_INFO("door id is not NULL");
                        uint8_t door_id = j["data"]["door_id"];
                        //bool on_off = j["data"]["set_state"];
                        bool on_off;
                        if(j["data"].find("set_state") != j["data"].end())
                        {
                            on_off = j["data"]["set_state"];
                        }
                        else
                        {
                            return;
                        }
                        ROS_WARN("get door id");//

                        if(on_off == true)
                        {
                        }
                        else
                        {
                        }
                        switch(door_id)
                        {
                            case 1:
                                break;
                            case 2:
                                break;
                            case 3:
                                break;
                            case 4:
                                break;
                            default :
                                ROS_ERROR("door ctrl id  error !");
                                break;
                        }
                    }
                }
            }
        }
    }
}



void Conveyor::rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg)
{
    mrobot_msgs::vci_can can_msg;
    mrobot_msgs::vci_can long_msg;
    CAN_ID_UNION id;

    long_msg = this->long_frame.frame_construct(c_msg);
    mrobot_msgs::vci_can* msg = &long_msg;
    if( msg->ID == 0 )
    {
        return;
    }
    if(this->is_log_on == true)
    {
        for(uint8_t i = 0; i < msg->DataLen; i++)
        {
            ROS_INFO("msg->Data[%d] = 0x%x",i,msg->Data[i]);
        }
    }
    can_msg.ID = msg->ID;
    id.CANx_ID = can_msg.ID;
    can_msg.DataLen = msg->DataLen;
    if(id.CanID_Struct.SrcMACID != NOAH_CONVEYOR_CAN_SRCMAC_ID)
    {
        return ;
    }

    //can_msg.Data.resize(can_msg.DataLen);
    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_GET_SYS_STATE)
    {
        if(id.CanID_Struct.ACK == 1)
        {
            //ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SYS_STATE");
            get_sys_status_ack_t get_sys_status_ack;
            *(uint16_t *)&msg->Data[1];
            get_sys_status_ack.sys_status = *(uint16_t *)&msg->Data[1];
            this->sys_conveyor->sys_status = get_sys_status_ack.sys_status;

            do
            {
                boost::mutex::scoped_lock(this->mtx);
                this->get_sys_status_ack_vector.push_back(get_sys_status_ack);
            }while(0);

            //this->PubPower(this->sys_conveyor);
        }
        if(id.CanID_Struct.ACK == 0)
        {
            //ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SYS_STATE");
            ROS_INFO("sys_status :mcu upload");
            *(uint16_t *)&msg->Data[1];
            this->sys_conveyor->sys_status = *(uint16_t *)&msg->Data[1];

            if((this->sys_conveyor->sys_status & STATE_IS_RECHARGE_IN) || (this->sys_conveyor->sys_status & STATE_IS_CHARGER_IN))
            {
                if(this->sys_conveyor->sys_status & STATE_IS_RECHARGE_IN)
                {
                    ROS_INFO("recharge plug in");
                }
                if(this->sys_conveyor->sys_status & STATE_IS_CHARGER_IN)
                {
                    ROS_INFO("charge plug in");
                }
            }
            else
            {
                ROS_INFO("charge not plug in");
                ROS_INFO("recharge not plug in");
            }
        }
    }

    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_READ_VERSION)
    {
        uint8_t len;
        len = msg->Data[1];
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_READ_VERSION");
        get_version_ack_t get_version_ack;
        get_version_ack.get_version_type = msg->Data[0];
        if(get_version_ack.get_version_type == 1)//software version
        {
            sys_conveyor->sw_version.resize(len);
            sys_conveyor->sw_version.clear();
            for(uint8_t i = 0; i < len; i++)
            {
                sys_conveyor->sw_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.sw_version.push_back(*(char *)&(msg->Data[i+2]));
            }

            n.setParam(software_version_param,sys_conveyor->sw_version.data());
        }
        else if(get_version_ack.get_version_type == 2)//protocol version
        {
            sys_conveyor->protocol_version.resize(len);
            sys_conveyor->protocol_version.clear();
            for(uint8_t i = 0; i < len; i++)
            {
                sys_conveyor->protocol_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.protocol_version.push_back(*(char *)&(msg->Data[i+2]));
            }
            n.setParam(protocol_version_param,sys_conveyor->protocol_version.data());
        }
        else if(get_version_ack.get_version_type == 3)//hardware version
        {
            sys_conveyor->hw_version.resize(len);
            sys_conveyor->hw_version.clear();
            //ROS_ERROR("hardware version length: %d",len);
            for(uint8_t i = 0; i < len; i++)
            {
                sys_conveyor->hw_version.push_back(*(char *)&(msg->Data[i+2]));
                get_version_ack.hw_version.push_back(*(char *)&(msg->Data[i+2]));
            }
            n.setParam(hardware_version_param,sys_conveyor->hw_version.data());
        }

        do
        {
            boost::mutex::scoped_lock(this->mtx);
            this->get_version_ack_vector.push_back(get_version_ack);
        }while(0);
    }


    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE");
        conveyor_belt_t conveyor_belt_ack;
        if(id.CanID_Struct.ACK == 0)
        {
            if(msg->DataLen == 1)
            {
                ROS_INFO("MCU upload: CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE");
                conveyor_belt_ack.set_result = msg->Data[0];
                ROS_INFO("conveyor belt result :%d", conveyor_belt_ack.set_result);
            }
            else
            {
                ROS_ERROR("can data len error ! data len: %d", msg->DataLen);
            }
        }
        else
        {
            conveyor_belt_ack.set_result = msg->Data[0];
            conveyor_belt_ack.set_work_mode = msg->Data[1];
            conveyor_belt_ack.err_status = msg->Data[2];
            do
            {
                boost::mutex::scoped_lock(this->mtx);
                this->set_conveyor_belt_work_mode_ack_vector.push_back(conveyor_belt_ack);
            } while (0);

            this->sys_conveyor->conveyor_belt.ack_work_mode = conveyor_belt_ack.ack_work_mode;
            this->sys_conveyor->conveyor_belt.err_status = conveyor_belt_ack.err_status;
        }
    }
}


void Conveyor::update_sys_status(void)
{
    uint16_t sys_status = this->sys_conveyor->sys_status;
}

