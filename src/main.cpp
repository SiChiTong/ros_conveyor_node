#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include <signal.h>
#include <vector>
#include <pthread.h>
#include <conveyor.h>


void sigintHandler(int sig)
{
    ROS_INFO("killing on exit");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conveyor_node");
    bool is_log_on = 0;
    ROS_INFO("creating conveyor node...");
    Conveyor *conveyor = new Conveyor(is_log_on);
    float rate = 100;
    ros::Rate loop_rate(rate);
    uint32_t cnt = 0;
    bool flag = 0;
    pthread_t can_protocol_proc_handle;
    pthread_create(&can_protocol_proc_handle, NULL, CanProtocolProcess,(void*)conveyor);

    get_version_t get_version;
    conveyor_belt_t set_conveyor_belt_work_mode;
    sanwei_rfid_id_t sanwei_rfid_id = {0};
    sanwei_rfid_info_t sanwei_rfid_info = {0};

    get_version.get_version_type = 1;

    set_conveyor_belt_work_mode.set_work_mode = CONVEYOR_BELT_STATUS_STOP;

    while (ros::ok())
    {
        if(flag == 0)
        {
            if(cnt % (uint32_t)(rate * 2) == (uint32_t)rate/2)
            {
                flag = 1;
                do
                {
                    boost::mutex::scoped_lock(conveyor->mtx);

                    get_version.get_version_type = 1;
                    conveyor->get_version_vector.push_back(get_version);

                    get_version.get_version_type = 2;
                    conveyor->get_version_vector.push_back(get_version);

                    get_version.get_version_type = 3;
                    conveyor->get_version_vector.push_back(get_version);
                    conveyor->get_sanwei_rfid_id_vector.push_back(sanwei_rfid_id);
                    //sleep(1);
                    conveyor->read_sanwei_rfid_info_vector.push_back(sanwei_rfid_info);


//                    sanwei_rfid_info.src_id = 0x7788;
//                    sanwei_rfid_info.dst_id = 0x9999;
//                    sanwei_rfid_info.time = 0x3333;
//
//                    conveyor->write_sanwei_rfid_info_vector.push_back(sanwei_rfid_info);

                    //conveyor->set_conveyor_belt_work_mode_vector.push_back(set_conveyor_belt_work_mode);

                }while(0);
            }
        }
        cnt++;
        ros::spinOnce();
        loop_rate.sleep();
    }

}
