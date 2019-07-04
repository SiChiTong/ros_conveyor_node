#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <boost/thread/mutex.hpp>
#include <mrobot_srvs/JString.h>
using json = nlohmann::json;
#ifndef __DRIVER_CONVEYOR_H__
#define __DRIVER_CONVEYOR_H__

#define CONVEYOR_CAN_SRCMAC_ID   0x53

//////  function id define  //////
#define CAN_FUN_ID_RESET        0x06
#define CAN_FUN_ID_WRITE        0x01
#define CAN_FUN_ID_READ         0x02
#define CAN_FUN_ID_TRIGGER      0x03


//////  source id define  //////
#define CAN_SOURCE_ID_READ_VERSION      0x01

#define CAN_SOURCE_ID_GET_SYS_STATE                 0x83
#define CAN_SOURCE_ID_SET_CONVEYOR_BELT_WORK_MODE   0xa0
#define CAN_SOURCE_ID_GET_PHO_ELEC_SWITCH_STATE     0xa1
#define CAN_SOURCE_ID_LOCK_CTRL                     0xa2
#define CAN_SOURCE_ID_GET_SANWEI_RFID_ID            0xa3
#define CAN_SOURCE_ID_WRITE_SANWEI_RFID_INFO        0xa4
#define CAN_SOURCE_ID_READ_SANWEI_RFID_INFO         0xa5

#define HW_VERSION_SIZE             3
#define SW_VERSION_SIZE             16
#define PROTOCOL_VERSION_SIZE       14


#define CONVEYOR_WORK_MODE_STRING_LOAD          "load"
#define CONVEYOR_WORK_MODE_STRING_UNLOAD        "unload"
#define CONVEYOR_WORK_MODE_STRING_STOP          "stop"

typedef struct
{
    uint8_t reserve;
}get_sys_status_t;

typedef struct
{
    uint8_t reserve;
    uint16_t sys_status;
}get_sys_status_ack_t;

typedef struct
{
    uint8_t get_version_type;
}get_version_t;


typedef struct
{
    uint8_t get_version_type;
    std::string    hw_version;
    std::string    sw_version;
    std::string    protocol_version;
}get_version_ack_t;


enum
{
    CONVEYOR_BELT_STATUS_STOP = 0,
    CONVEYOR_BELT_STATUS_LOAD,
    CONVEYOR_BELT_STATUS_UNLOAD,
    CONVEYOR_BELT_STATUS_MAX,
}conveyor_status_e;


#define CONVEYOR_BELT_EXEC_OK                   0x00
#define CONVEYOR_BELT_IS_OCCUPIED               0x81
#define CONVEYOR_BELT_IS_ALREADY_EMPTY          0x82
typedef struct
{
    uint8_t set_work_mode;
    uint8_t need_lock;
    uint8_t set_result;
    uint8_t ack_work_mode;
#define CONVEYOR_BELT_LOAD_TIMEOUT              0xF0
#define CONVEYOR_BELT_UNLOAD_TIMEOUT            0xF1
#define CONVEYOR_BELT_STATUS_ERROR              0xFF
#define CONVEYOR_LOAD_FINISHED_OK               0x01
#define CONVEYOR_UNLOAD_FINISHED_OK             0x02
    uint8_t err_status;
}conveyor_belt_t;


#define LOCK_STATUS_LOCK                        0x01
#define LOCK_STATUS_UNLOCK                      0x00
typedef struct
{
    uint8_t status;
}lock_ctrl_t;


typedef struct
{
    uint8_t result;
    uint16_t id;
}sanwei_rfid_id_t;

typedef struct
{
    uint8_t result;
    uint16_t card_id;
    uint16_t station_id;
    uint16_t dst_id;
    uint16_t src_id;
    uint16_t service_id;
    uint16_t time;
}sanwei_rfid_info_t;

typedef struct
{
    uint8_t serial_num;
    CAN_ID_UNION id;
}can_upload_ack_t;

typedef struct
{
#define VERSION_TYPE_FW             0
#define VERSION_TYPE_PROTOCOL       1
    uint8_t                     get_version_type;

    std::string                 hw_version;
    std::string                 sw_version;
    std::string                 protocol_version;

    uint16_t                     sys_status;
    conveyor_belt_t             conveyor_belt;
    lock_ctrl_t                 lock_ctrl;
    lock_ctrl_t                 lock_status_ack;
    sanwei_rfid_id_t            sanwei_rfid_id;
    sanwei_rfid_info_t          sanwei_rfid_info;
    sanwei_rfid_info_t          read_sanwei_rfid_info;
}conveyor_t;

extern conveyor_t    *sys_conveyor;

class Conveyor
{
    public:
        Conveyor(bool log_on = false)
        {
            is_log_on = log_on;

            pub_to_can_node = n.advertise<mrobot_msgs::vci_can>("conveyor_to_can", 1000);

            sub_from_can_node = n.subscribe("can_to_conveyor", 1000, &Conveyor::rcv_from_can_node_callback, this);
            sub_conveyor_work_mode_test = n.subscribe("/conveyor_work_mode_test", 1000, &Conveyor::work_mode_test_callback, this);
            sub_conveyor_work_mode = n.subscribe("/conveyor_ctrl", 1000, &Conveyor::work_mode_callback, this);
            sub_conveyor_lock_ctrl = n.subscribe("/conveyor_lock_ctrl", 1000, &Conveyor::lock_ctrl_callback, this);
            pub_conveyor_work_mode_ack = n.advertise<std_msgs::String>("conveyor_ctrl_ack", 1000);
            pub_conveyor_lock_ctrl_ack = n.advertise<std_msgs::String>("conveyor_lock_ctrl_ack", 1000);
            pub_pho_state = n.advertise<std_msgs::String>("conveyor/pho_state", 1000);

            rfid_info_service = n.advertiseService("/conveyor/rfid_info", &Conveyor::service_rfid_ctrl, this);


            sys_conveyor = &sys_conveyor_ram;
            sys_conveyor->sys_status = 0;

            get_sys_status_vector.clear();
            get_version_vector.clear();

            get_rfid_id_flag = 0;
            write_rfid_info_flag = 0;
            read_rfid_info_flag = 0;
        }
        int conveyorParamInit(void);
        int GetVersion(conveyor_t *sys);
        int GetSysStatus(conveyor_t *sys);
        int set_conveyor_belt_work_mode(uint8_t mode, uint8_t need_lock);
        int set_lock_status(uint8_t status);
        int get_sanwei_rfid_func(void);
        int write_sanwei_rfid_info_func(uint16_t dst_id, uint16_t src_id, uint16_t time);
        int read_sanwei_rfid_info_func(void);

        bool service_rfid_ctrl(mrobot_srvs::JString::Request &ctrl, mrobot_srvs::JString::Response &status);
        void rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg);
        void work_mode_test_callback(const std_msgs::UInt8MultiArray &msg);
        void work_mode_callback(const std_msgs::String::ConstPtr &msg);
        void lock_ctrl_callback(const std_msgs::String::ConstPtr &msg);
        void ack_work_mode_start_result(const std::string &msg, int err_code);
        void ack_work_mode_exec_result(const std::string &msg, int err_code);
        void ack_lock_ctrl(const std::string &msg, int err_code);
        void post_pho_state(uint32_t state);
        int ack_mcu_upload(CAN_ID_UNION id, uint8_t serial_num);
        void pub_json_msg(const nlohmann::json j_msg);

        json j;
        conveyor_t    *sys_conveyor;
        can_long_frame  long_frame;

        uint32_t get_rfid_id_flag;
        uint32_t write_rfid_info_flag;
        uint32_t read_rfid_info_flag;

        vector<get_sys_status_t>        get_sys_status_vector;
        vector<get_sys_status_ack_t>    get_sys_status_ack_vector;

        vector<get_version_t>           get_version_vector;
        vector<get_version_ack_t>       get_version_ack_vector;

        vector<conveyor_belt_t>         set_conveyor_belt_work_mode_vector;
        vector<conveyor_belt_t>         set_conveyor_belt_work_mode_ack_vector;

        vector<lock_ctrl_t>             lock_ctrl_vector;
        vector<lock_ctrl_t>             lock_ctrl_ack_vector;

        vector<sanwei_rfid_id_t>        get_sanwei_rfid_id_vector;
        vector<sanwei_rfid_id_t>        sanwei_rfid_id_ack_vector;

        vector<sanwei_rfid_info_t>        write_sanwei_rfid_info_vector;
        vector<sanwei_rfid_info_t>        sanwei_rfid_info_ack_vector;

        vector<sanwei_rfid_info_t>        read_sanwei_rfid_info_vector;
        vector<sanwei_rfid_info_t>        read_sanwei_rfid_info_ack_vector;


        boost::mutex mtx;
        bool is_log_on;

    private:

        ros::NodeHandle n;
        ros::Publisher pub_to_can_node;//publish to roscan node
        ros::Subscriber sub_from_can_node;
        ros::Subscriber sub_conveyor_work_mode_test;
        ros::Subscriber sub_conveyor_work_mode;
        ros::Subscriber sub_conveyor_lock_ctrl;
        ros::Publisher pub_conveyor_work_mode_ack;
        ros::Publisher pub_conveyor_lock_ctrl_ack;
        ros::Publisher pub_pho_state;

        ros::ServiceServer rfid_info_service;

        conveyor_t    sys_conveyor_ram;

        std::string software_version_param = "mcu_conveyor_version";
        std::string hardware_version_param = "conveyor_hardware_version";
        std::string protocol_version_param = "conveyor_protocol_version";

        std_msgs::String json_to_String(const nlohmann::json j_msg);
        std_msgs::String ack_get_rfid_id_service(int err_code, uint32_t id);
        std_msgs::String ack_write_rfid_info_service(int err_code);
};

void *CanProtocolProcess(void* arg);

#endif
