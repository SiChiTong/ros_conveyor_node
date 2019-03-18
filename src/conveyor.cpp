/*
 *  conveyor.cpp
 *  Communicate Protocol.
 *  Author: Kaka Xie
 */

#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include <string.h>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <conveyor.h>


#define GET_SYS_STSTUS_TIME_OUT                 500//ms
#define GET_VERSION_TIME_OUT                    500//ms
#define SET_CONVEYOR_BELT_WORK_MODE_TIME_OUT    500//ms
#define LOCK_CTRL_TIME_OUT                      500//ms

#define GET_VERSION_RETRY_CNT                   5
#define SET_CONVEYOR_BELT_WORK_MODE_RETRY_CNT   5
#define LOCK_CTRL_RETRY_CNT                     5
void *CanProtocolProcess(void* arg)
{
    get_sys_status_t get_sys_status;
    get_version_t get_version;
    conveyor_belt_t set_conveyor_belt_mode;
    lock_ctrl_t lock_ctrl;

    Conveyor *pConveyor =  (Conveyor*)arg;

    bool get_sys_status_flag = 0;
    bool get_version_flag = 0;
    bool set_conveyor_belt_flag = 0;
    bool lock_ctrl_flag = 0;
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
                pConveyor->set_conveyor_belt_work_mode_ack_vector.clear();
            }while(0);

set_conveyor_belt_work_mode_restart:
            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("set conveyor belt work mode :send cmd to mcu");
            }
            pConveyor->sys_conveyor->conveyor_belt = set_conveyor_belt_mode;

            pConveyor->set_conveyor_belt_work_mode(pConveyor->sys_conveyor->conveyor_belt.set_work_mode, pConveyor->sys_conveyor->conveyor_belt.need_lock);
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

                            if ((set_conveyor_belt_work_mode_ack.set_work_mode >= CONVEYOR_BELT_STATUS_STOP) && (set_conveyor_belt_work_mode_ack.set_work_mode < CONVEYOR_BELT_STATUS_MAX))
                            {
                                set_conveyor_belt_work_mode_ack_flag = 1;
                                if (pConveyor->is_log_on == true)
                                {
                                    ROS_INFO("get right set conveyor belt work mode ack");
                                }
                            }
                            else
                            {
                                ROS_ERROR("error: get ack set conveyor belt work mode is %d", set_conveyor_belt_work_mode_ack.set_work_mode);
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
                    std::string mode;
                    set_conveyor_belt_work_mode_ack_flag = 0;
                    if(pConveyor->is_log_on == true)
                    {
                    }
                    pConveyor->sys_conveyor->conveyor_belt = set_conveyor_belt_work_mode_ack;
                    if(pConveyor->is_log_on == true)
                    {
                        ROS_INFO("get set conveyor belt work mode:%d",set_conveyor_belt_work_mode_ack.set_work_mode);
                    }
                    if(set_conveyor_belt_work_mode_ack.set_work_mode == CONVEYOR_BELT_STATUS_STOP)
                    {
                        mode = CONVEYOR_WORK_MODE_STRING_STOP;
                    }
                    else if(set_conveyor_belt_work_mode_ack.set_work_mode == CONVEYOR_BELT_STATUS_LOAD)
                    {
                        mode = CONVEYOR_WORK_MODE_STRING_LOAD;
                    }
                    else if(set_conveyor_belt_work_mode_ack.set_work_mode == CONVEYOR_BELT_STATUS_UNLOAD)
                    {
                        mode = CONVEYOR_WORK_MODE_STRING_UNLOAD;
                    }


                    if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_EXEC_OK)
                    {
                        ROS_INFO("conveyor belt exec ok");
                        pConveyor->ack_work_mode_start_result(mode, CONVEYOR_BELT_EXEC_OK);
                    }
//                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_LOAD_TIMEOUT)
//                    {
//                        ROS_ERROR("conveyor belt load timeout !");
//                        pConveyor->ack_work_mode_exec_result(mode, CONVEYOR_BELT_LOAD_TIMEOUT);
//                    }
//                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_UNLOAD_TIMEOUT)
//                    {
//                        ROS_ERROR("conveyor belt unload timeout !");
//                        pConveyor->ack_work_mode_exec_result(mode, CONVEYOR_BELT_UNLOAD_TIMEOUT);
//                    }
                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_IS_OCCUPIED)
                    {
                        ROS_ERROR("conveyor belt is occupied !");
                        pConveyor->ack_work_mode_start_result(mode, CONVEYOR_BELT_IS_OCCUPIED);
                    }
                    else if(set_conveyor_belt_work_mode_ack.err_status == CONVEYOR_BELT_IS_ALREADY_EMPTY)
                    {
                        ROS_ERROR("conveyor belt is already empty !");
                        pConveyor->ack_work_mode_start_result(mode, CONVEYOR_BELT_IS_ALREADY_EMPTY);
                    }
                    else
                    {
                        ROS_ERROR("conveyor belt unknow error, error type: %d !", set_conveyor_belt_work_mode_ack.err_status);
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
                    ROS_ERROR("set conveyor work mode start to resend msg....");
                    goto set_conveyor_belt_work_mode_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with conveyor mcu, set conveyor belt work mode failed !");
                err_cnt = 0;
            }

        }
        /* -------- set conveyor belt work mode protocol end -------- */





        /* --------  lock ctrl protocol begin -------- */
        do
        {
            boost::mutex::scoped_lock(mtx);
            if(!pConveyor->lock_ctrl_vector.empty())
            {
                auto a = pConveyor->lock_ctrl_vector.begin();
                lock_ctrl = *a;
                lock_ctrl_flag = 1;

                pConveyor->lock_ctrl_vector.erase(a);

            }

        }while(0);

        if(lock_ctrl_flag == 1)
        {
            uint8_t flag = 0;
            uint32_t time_out_cnt = 0;
            static uint8_t err_cnt = 0;

            lock_ctrl_flag = 0;

            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("lock_ctrl.status = %d", lock_ctrl.status);
                ROS_INFO("get lock ctrl cmd");
            }
            do
            {
                boost::mutex::scoped_lock(mtx);
                pConveyor->lock_ctrl_ack_vector.clear();
            }while(0);

lock_ctrl_restart:
            if(pConveyor->is_log_on == true)
            {
                ROS_INFO("lock ctrl :send cmd to mcu");
            }
            pConveyor->sys_conveyor->lock_ctrl = lock_ctrl;

            pConveyor->set_lock_status(pConveyor->sys_conveyor->lock_ctrl.status);
            bool lock_ctrl_ack_flag = 0;
            lock_ctrl_t lock_ctrl_ack;
            while(time_out_cnt < LOCK_CTRL_TIME_OUT / 10)
            {
                time_out_cnt++;
                do
                {
                    boost::mutex::scoped_lock(mtx);
                    if(!pConveyor->lock_ctrl_ack_vector.empty())
                    {
                        if(pConveyor->is_log_on == true)
                        {
                            ROS_INFO("lock_ctrl_ack_vector is not empty");
                        }
                        auto b = pConveyor->lock_ctrl_ack_vector.begin();

                        lock_ctrl_ack = *b;

                        pConveyor->lock_ctrl_ack_vector.erase(b);

                        if(lock_ctrl_ack.status == lock_ctrl.status)
                        {

                            lock_ctrl_ack_flag = 1;
                            if (pConveyor->is_log_on == true)
                            {
                                ROS_INFO("get right lock ctrl ack");
                            }
                        }
                        else
                        {
                            ROS_ERROR("error: get ack lock ctrl status is %d", lock_ctrl_ack.status);
                        }
                    }
                }while(0);
                if(lock_ctrl_ack_flag == 1)
                {
                    lock_ctrl_ack_flag = 0;
                    pConveyor->sys_conveyor->lock_status_ack = lock_ctrl_ack;
                    if(pConveyor->sys_conveyor->lock_status_ack.status == LOCK_STATUS_LOCK)
                    {
                        pConveyor->ack_lock_ctrl("lock", 1);
                    }
                    else if(pConveyor->sys_conveyor->lock_status_ack.status == LOCK_STATUS_UNLOCK)
                    {
                        pConveyor->ack_lock_ctrl("unlock", 1);
                    }
                    else
                    {
                        pConveyor->ack_lock_ctrl("lock", 0);
                    }
                    ROS_INFO("lock ctrl exec ok");
                    break;
                }
                else
                {
                    usleep(10*1000);
                }
            }
            if(time_out_cnt < LOCK_CTRL_TIME_OUT / 10)
            {
                err_cnt = 0;
                time_out_cnt = 0;
            }
            else
            {
                ROS_ERROR("lock ctrl time out");
                time_out_cnt = 0;
                if(err_cnt++ < LOCK_CTRL_RETRY_CNT)
                {
                    ROS_ERROR("lock ctrl start to resend msg....");
                    goto lock_ctrl_restart;
                }
                ROS_ERROR("CAN NOT COMMUNICATE with conveyor mcu, lock ctrl failed !");
                if(pConveyor->sys_conveyor->lock_ctrl.status == LOCK_STATUS_LOCK)
                {
                    pConveyor->ack_lock_ctrl("lock", 0x10);
                }
                else if(pConveyor->sys_conveyor->lock_ctrl.status == LOCK_STATUS_UNLOCK)
                {
                    pConveyor->ack_lock_ctrl("unlock", 0x11);
                }
                else
                {
                    pConveyor->ack_lock_ctrl("lock", 0x20);
                }
                err_cnt = 0;
            }

        }
        /* -------- lock ctrl protocol end -------- */



        usleep(10 * 1000);
    }
}


int Conveyor::conveyorParamInit(void)
{
    return 0;
}



int Conveyor::GetVersion(conveyor_t *sys)
{
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_READ_VERSION;
    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = CONVEYOR_CAN_SRCMAC_ID;
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

int Conveyor::GetSysStatus(conveyor_t *sys)
{
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_GET_SYS_STATE;
    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = CONVEYOR_CAN_SRCMAC_ID;
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

int Conveyor::set_conveyor_belt_work_mode(uint8_t mode, uint8_t need_lock)
{
    ROS_INFO("start to set conveyor belt work mode . . . ");
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE;
    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = CONVEYOR_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 3;
    can_msg.Data.resize(3);
    can_msg.Data[0] = 0x00;
    if((mode >= 0) && (mode < CONVEYOR_BELT_STATUS_MAX))
    {
        can_msg.Data[1] = mode;
    }
    else
    {
        ROS_ERROR("set conveyor belt work mode: parameter error !  set work mode %d", mode);
        return -1;
    }
    if(need_lock < 2)
    {
        can_msg.Data[2] = need_lock;
    }
    else
    {
        ROS_ERROR("param need lock error: %d", need_lock);
        return -1;
    }

    ROS_INFO("set conveyor belt work mode %d, need lock: %d", mode, need_lock);
    this->pub_to_can_node.publish(can_msg);
    return error;
}


int Conveyor::set_lock_status(uint8_t status)
{
    ROS_INFO("start to set lock status . . . ");
    int error = 0;
    mrobot_msgs::vci_can can_msg;
    CAN_ID_UNION id;
    memset(&id, 0x0, sizeof(CAN_ID_UNION));
    id.CanID_Struct.SourceID = CAN_SOURCE_ID_LOCK_CTRL;
    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = CONVEYOR_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 0;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    if((status == LOCK_STATUS_LOCK) || (status == LOCK_STATUS_UNLOCK))
    {
        can_msg.Data[1] = status;
        ROS_INFO("set lock ctrl: %d", status);
    }
    else
    {
        ROS_ERROR("set lock ctrl: parameter error !  set lock status %d", status);
        return -1;
    }

    this->pub_to_can_node.publish(can_msg);
    return error;
}



int Conveyor::ack_mcu_upload(CAN_ID_UNION id, uint8_t serial_num)
{
    ROS_INFO("start to ack mcu upload info. . . ");
    int error = 0;
    mrobot_msgs::vci_can can_msg;

    id.CanID_Struct.SrcMACID = 0;
    id.CanID_Struct.DestMACID = CONVEYOR_CAN_SRCMAC_ID;
    id.CanID_Struct.FUNC_ID = 0x02;
    id.CanID_Struct.ACK = 1;
    id.CanID_Struct.res = 0;

    can_msg.ID = id.CANx_ID;
    can_msg.DataLen = 2;
    can_msg.Data.resize(2);
    can_msg.Data[0] = 0x00;
    can_msg.Data[1] = serial_num;
    this->pub_to_can_node.publish(can_msg);
    return error;
}


void Conveyor::pub_json_msg( const nlohmann::json j_msg)
{
    std_msgs::String json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j_msg;
    json_msg.data = ss.str();
    this->pub_conveyor_work_mode_ack.publish(json_msg);
}

void Conveyor::ack_work_mode_start_result(const std::string &mode, int err_code)
{
    json j;
    j.clear();
    j =
        {
            {"sub_name", "conveyor_ctrl_start_ack"},
                {"data",
                    {
                        {"set_mode", mode.c_str()},
                        {"error_code", err_code},
                }
            }
        };
    this->pub_json_msg(j);
}

void Conveyor::ack_work_mode_exec_result(const std::string &msg, int err_code)
{

    json j;
    j.clear();
    j =
        {
            {"sub_name", "conveyor_ctrl_exec_ack"},
                {"data",
                    {
                        {"error_code", err_code},
                }
            }
        };
    this->pub_json_msg(j);
}

void Conveyor::work_mode_callback(const std_msgs::String::ConstPtr &msg)
{
    conveyor_belt_t conveyor_work_mode = {0};
    auto j = json::parse(msg->data.c_str());
    std::string j_str = j.dump();
    ROS_WARN("%s",j_str.data());

    if(j.find("pub_name") != j.end())
    {
        if(j["pub_name"] == "conveyor_ctrl")
        {
            if(j.find("data") != j.end())
            {
                if (j["data"].find("set_mode") != j["data"].end())
                {
                    if (j["data"].find("lock") != j["data"].end())
                    {
                        ROS_INFO("get lock");
                        if (j["data"]["lock"] == true)
                        {
                            ROS_INFO("get is need lock : true");
                            conveyor_work_mode.need_lock = 1;
                        }
                        else if (j["data"]["lock"] == false)
                        {
                            ROS_INFO("get is need lock : false");
                            conveyor_work_mode.need_lock = 0;
                        }
                        else
                        {
                            ROS_ERROR("is need lock parameter error !");
                        }
                    }

                    if (j["data"]["set_mode"] == CONVEYOR_WORK_MODE_STRING_LOAD)
                    {
                        ROS_WARN("%s:get load ctrl", __func__);
                        conveyor_work_mode.set_work_mode = CONVEYOR_BELT_STATUS_LOAD;
                        this->set_conveyor_belt_work_mode_vector.push_back(conveyor_work_mode);
                    }
                    else if (j["data"]["set_mode"] == CONVEYOR_WORK_MODE_STRING_UNLOAD)
                    {
                        ROS_WARN("%s:get unload ctrl", __func__);
                        conveyor_work_mode.set_work_mode = CONVEYOR_BELT_STATUS_UNLOAD;
                        this->set_conveyor_belt_work_mode_vector.push_back(conveyor_work_mode);
                    }
                    else if (j["data"]["set_mode"] == CONVEYOR_WORK_MODE_STRING_STOP)
                    {
                        ROS_WARN("%s:get stop ctrl", __func__);
                        conveyor_work_mode.set_work_mode = CONVEYOR_BELT_STATUS_STOP;
                        this->set_conveyor_belt_work_mode_vector.push_back(conveyor_work_mode);
                    }
                    else
                    {
                        ROS_ERROR("%s:set mode parse error!", __func__);
                    }
                }
            }
        }
    }
}




void Conveyor::ack_lock_ctrl(const std::string &msg, int err_code)
{

    json j;
    j.clear();
    j =
        {
            {"sub_name", "conveyor_lock_ctrl_ack"},
                {"data",
                    {
                        {"lock_status", msg.c_str()},
                        {"error_code", err_code},
                }
            }
        };

    std_msgs::String json_msg;
    std::stringstream ss;

    ss.clear();
    ss << j;
    json_msg.data = ss.str();
    this->pub_conveyor_lock_ctrl_ack.publish(json_msg);
}


void Conveyor::lock_ctrl_callback(const std_msgs::String::ConstPtr &msg)
{
    lock_ctrl_t lock_ctrl = {0};
    auto j = json::parse(msg->data.c_str());
    std::string j_str = j.dump();
    ROS_WARN("%s: %s", __func__, j_str.data());

    if(j.find("pub_name") != j.end())
    {
        if(j["pub_name"] == "conveyor_lock_ctrl")
        {
            if(j.find("data") != j.end())
            {
                if (j["data"].find("lock_ctrl") != j["data"].end())
                {
                    if (j["data"]["lock_ctrl"] == "lock")
                    {
                        ROS_WARN("%s:get conveyor lock ctrl: lock", __func__);
                        lock_ctrl.status = LOCK_STATUS_LOCK;
                        this->lock_ctrl_vector.push_back(lock_ctrl);
                    }
                    else if (j["data"]["lock_ctrl"] == "unlock")
                    {
                        ROS_WARN("%s:get lock ctrl : unlock", __func__);
                        lock_ctrl.status = LOCK_STATUS_UNLOCK;
                        this->lock_ctrl_vector.push_back(lock_ctrl);
                    }
                    else
                    {
                        ROS_ERROR("%s: conveyor lock ctrl parameter error ! !", __func__);
                    }
                }
            }
        }
    }
}


void Conveyor::work_mode_test_callback(const std_msgs::UInt8MultiArray &msg)
{
    int size = msg.data.size();
    conveyor_belt_t conveyor_work_mode;
    if(size == 1)
    {
        if(msg.data[0] < CONVEYOR_BELT_STATUS_MAX)
        {
            if(msg.data[0] == CONVEYOR_BELT_STATUS_STOP)
            {
                conveyor_work_mode.set_work_mode = CONVEYOR_BELT_STATUS_STOP;
                this->set_conveyor_belt_work_mode_vector.push_back(conveyor_work_mode);
            }
            else if(msg.data[0] == CONVEYOR_BELT_STATUS_LOAD)
            {
                conveyor_work_mode.set_work_mode = CONVEYOR_BELT_STATUS_LOAD;
                this->set_conveyor_belt_work_mode_vector.push_back(conveyor_work_mode);
            }
            else if(msg.data[0] == CONVEYOR_BELT_STATUS_UNLOAD)
            {
                conveyor_work_mode.set_work_mode = CONVEYOR_BELT_STATUS_UNLOAD;
                this->set_conveyor_belt_work_mode_vector.push_back(conveyor_work_mode);
            }
        }
        else
        {
            ROS_ERROR("%s: parameter error: msg.data[0]: %d", __func__, msg.data[0]);
        }
    }
    else
    {
        ROS_ERROR("%s: parameter error: msg.data.size : %d", __func__, size);
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
    if(id.CanID_Struct.SrcMACID != CONVEYOR_CAN_SRCMAC_ID)
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
        }

        if(id.CanID_Struct.ACK == 0)
        {
            //ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_GET_SYS_STATE");
            ROS_INFO("sys_status :mcu upload");
            *(uint16_t *)&msg->Data[1];
            this->sys_conveyor->sys_status = *(uint16_t *)&msg->Data[1];
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
            if(msg->DataLen <= 2)
            {
                std::string mode;
                uint8_t flag = 1;

                ROS_INFO("MCU upload: CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE");
                conveyor_belt_ack.set_result = msg->Data[0];
                if(conveyor_belt_ack.set_result == CONVEYOR_BELT_LOAD_TIMEOUT)
                {
                    ROS_ERROR("load time out ! !");
                    this->ack_work_mode_exec_result(mode, CONVEYOR_BELT_LOAD_TIMEOUT);
                }
                else if(conveyor_belt_ack.set_result == CONVEYOR_BELT_UNLOAD_TIMEOUT)
                {
                    ROS_ERROR("unload time out ! !");
                    this->ack_work_mode_exec_result(mode, CONVEYOR_BELT_UNLOAD_TIMEOUT);
                }
                else if(conveyor_belt_ack.set_result == CONVEYOR_LOAD_FINISHED_OK)
                {
                    ROS_INFO("load exec finished ok.");
                    this->ack_work_mode_exec_result(mode, CONVEYOR_LOAD_FINISHED_OK);
                }
                else if(conveyor_belt_ack.set_result == CONVEYOR_UNLOAD_FINISHED_OK)
                {
                    ROS_INFO("unload exec finished ok.");
                    this->ack_work_mode_exec_result(mode, CONVEYOR_UNLOAD_FINISHED_OK);
                }
                else
                {
                    ROS_ERROR("mcu upload state error: msg->Data[0]: %d ! !", msg->Data[0]);
                    flag = 0;
                }
                if(flag == 1)
                {
                    can_upload_ack_t can_upload_ack = {0};
                    can_upload_ack.serial_num = msg->Data[msg->DataLen - 1];
                    ROS_INFO("serial num: %d", can_upload_ack.serial_num);
                    can_upload_ack.id.CanID_Struct.ACK = 1;
                    can_upload_ack.id.CanID_Struct.SourceID = CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE;
                    this->ack_mcu_upload(can_upload_ack.id, can_upload_ack.serial_num);
                }
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



    if(id.CanID_Struct.SourceID == CAN_SOURCE_ID_LOCK_CTRL)
    {
        ROS_INFO("rcv from mcu,source id CAN_SOURCE_ID_LOCK_CTRL");
        lock_ctrl_t lock_ctrl;
        if(id.CanID_Struct.ACK == 1)
        {
            lock_ctrl.status = msg->Data[0];
            do
            {
                boost::mutex::scoped_lock(this->mtx);
                this->lock_ctrl_ack_vector.push_back(lock_ctrl);
            } while (0);

            this->sys_conveyor->lock_status_ack = lock_ctrl;
        }
    }
}
