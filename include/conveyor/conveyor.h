#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include <mrobot_msgs/vci_can.h>
#include <roscan/can_long_frame.h>
#include <boost/thread/mutex.hpp>
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

#define HW_VERSION_SIZE             3
#define SW_VERSION_SIZE             16
#define PROTOCOL_VERSION_SIZE       14


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

typedef struct
{
    uint8_t set_work_mode;
    uint8_t set_result;
    uint8_t ack_work_mode;
#define CONVEYOR_BELT_LOAD_ERROR        0xF0
#define CONVEYOR_BELT_UNLOAD_ERROR      0xF1
#define CONVEYOR_BELT_STATUS_ERROR      0xFF
    uint8_t err_status;
}conveyor_belt_t;

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
}conveyor_t;

extern conveyor_t    *sys_conveyor;

class Conveyor
{
    public:
        Conveyor(bool log_on = false)
        {
            is_log_on = log_on;
            noah_conveyor_pub = n.advertise<std_msgs::String>("tx_noah_conveyor_node",1000);
            noah_conveyor_sub = n.subscribe("rx_noah_conveyor_node",1000,&Conveyor::from_app_rcv_callback,this);

            pub_to_can_node = n.advertise<mrobot_msgs::vci_can>("conveyor_to_can", 1000);

            sub_from_can_node = n.subscribe("can_to_conveyor", 1000, &Conveyor::rcv_from_can_node_callback, this);
            sub_conveyor_work_mode_test = n.subscribe("/conveyor_work_mode_test", 1000, &Conveyor::work_mode_test_callback, this);


            sys_conveyor = &sys_conveyor_ram;
            sys_conveyor->sys_status = 0;

            get_sys_status_vector.clear();
            get_version_vector.clear();
        }
        int conveyorParamInit(void);
        int GetVersion(conveyor_t *sys);
        int GetSysStatus(conveyor_t *sys);
        int get_serials_leds_version(conveyor_t *sys);
        int set_conveyor_belt_work_mode(uint8_t mode);

        int handle_receive_data(conveyor_t *sys);
        void from_app_rcv_callback(const std_msgs::String::ConstPtr &msg);
        void from_navigation_rcv_callback(const std_msgs::String::ConstPtr &msg);
        void power_from_app_rcv_callback(std_msgs::UInt8MultiArray data);
        void PubChargeStatus(uint8_t status);

        void rcv_from_can_node_callback(const mrobot_msgs::vci_can::ConstPtr &c_msg);
        void work_mode_test_callback(const std_msgs::UInt8MultiArray &msg);

        json j;
        void pub_json_msg_to_app(const nlohmann::json j_msg);
        conveyor_t    *sys_conveyor;
        can_long_frame  long_frame;

        void update_sys_status(void);


        vector<get_sys_status_t>        get_sys_status_vector;
        vector<get_sys_status_ack_t>    get_sys_status_ack_vector;

        vector<get_version_t>           get_version_vector;
        vector<get_version_ack_t>       get_version_ack_vector;

        vector<conveyor_belt_t>         set_conveyor_belt_work_mode_vector;
        vector<conveyor_belt_t>         set_conveyor_belt_work_mode_ack_vector;

        boost::mutex mtx;
        bool is_log_on;

    private:

        ros::NodeHandle n;
        ros::Publisher noah_conveyor_pub;
        ros::Subscriber noah_conveyor_sub;
        ros::Publisher pub_to_can_node;//publish to roscan node
        ros::Subscriber sub_from_can_node;
        ros::Subscriber sub_conveyor_work_mode_test;

        conveyor_t    sys_conveyor_ram;

        std::string software_version_param = "mcu_noah_conveyor_version";
        std::string hardware_version_param = "noah_conveyor_hardware_version";
        std::string protocol_version_param = "noah_conveyor_protocol_version";

};

void *CanProtocolProcess(void* arg);

#endif



