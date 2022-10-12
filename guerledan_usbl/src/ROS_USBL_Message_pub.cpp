#include <iostream>
#include <ctime>
#include <fstream>
#include <seatrac_driver/AsyncService.h>
#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "guerledan_usbl/USBL.h"

using namespace std;
using namespace narval::seatrac;

void log_write(ofstream &log, const narval::seatrac::messages::PingResp &lastResponse_ )
{
    log<<lastResponse_.acoFix.position.northing/10.0<<", ";
    log<<lastResponse_.acoFix.position.easting/10.0<<", ";
    log<<lastResponse_.acoFix.position.depth/10.0<<", ";
    log<<lastResponse_.acoFix.usbl.azimuth/10.0<<", ";
    log<<lastResponse_.acoFix.usbl.elevation/10.0<<",";
    log<<lastResponse_.acoFix.range.dist/10.0<<",";
    log<<lastResponse_.acoFix.depthLocal/10.0<<endl;
    
}
std::string current_time()
{
    time_t now = time(0);
   
   // convert now to string form
   char* dt = ctime(&now);

   cout << "The local date and time is: " << dt << endl;

   // convert now to tm struct for UTC
//    tm *gmtm = gmtime(&now);
//    dt = asctime(gmtm);
   char* test= new char[8];
   for (int i=0;i<8;i++)
   {
      test[i]=dt[i+11];
   }
   std::string s(test,8);
   return s;
}
   

void write_message(guerledan_usbl::USBL &data, const narval::seatrac::messages::PingResp &lastResponse_)
{
    data.azimuth = lastResponse_.acoFix.usbl.azimuth/10;
    data.elevation = lastResponse_.acoFix.usbl.elevation/10;
    data.range = lastResponse_.acoFix.range.dist/10;
    data.depth = lastResponse_.acoFix.depthLocal/10;
    data.position.x = lastResponse_.acoFix.position.easting/10;
    data.position.y = lastResponse_.acoFix.position.northing/10;
    data.position.z = lastResponse_.acoFix.position.depth/10;
}
narval::seatrac::messages::Status get_next_status(SeatracDriver& seatrac)
{
    narval::seatrac::messages::Status status;

    if(!seatrac.wait_for_message(CID_STATUS, &status, 1000)) {
        std::cout << "TIMEOUT" << std::endl << std::flush;
        // throw TimeoutReached();
    }

    return status;
}

class MyDriver : public narval::seatrac::SeatracDriver
{
    public:


    narval::seatrac::messages::PingResp lastResponse_;
    bool data_available=false;
    MyDriver(const IoServicePtr& ioService,
             const std::string& port = "/dev/narval_usbl") :
        narval::seatrac::SeatracDriver(ioService, port)
    {}

    void on_message(CID_E msgId, const std::vector<uint8_t>& msgData)
    {
        switch(msgId) {
            default:
                break;
            case CID_PING_ERROR:
                {
                    narval::seatrac::messages::PingError response;
                    response = msgData;
                    std::cout << response << std::endl;
                }
                break;
            case CID_PING_RESP:
                std::cout << "Got a Ping Response" << std::endl << std::flush;
                {
                    lastResponse_ = msgData;
                    data_available=true;
                }
                break;
        }
    }
};

int main(int argc, char** argv)
{
    cout << "Test 1 " << endl << flush;
    ros::init(argc, argv, "USBL_pub_node");
    ros::NodeHandle n;
    ROS_INFO("Test");

    const char* device = "/dev/ttyUSB0";
    if(argc > 1) device = argv[1];

    AsyncService service;
    MyDriver seatrac(service.io_service(), device);
    seatrac.enable_io_dump();

    service.start();
    
    ros::Publisher chatter_pub = n.advertise<guerledan_usbl::USBL>("Informations", 1000);
    guerledan_usbl::USBL USBL_info_message;
    ofstream log;
    log.open("src/guerledan_usbl/logs/October_11_"+current_time()+".dat");
    log<<"LOG: northing, easting, depth, azimith, elevation, range, Local depth"<<endl;
    while (ros::ok()){
        command::ping_send(seatrac, BEACON_ID_1, MSG_REQU);
        if (seatrac.data_available)
        {
            log_write(log,seatrac.lastResponse_);
            write_message(USBL_info_message, seatrac.lastResponse_);
            chatter_pub.publish(USBL_info_message);
            seatrac.data_available =false;
            ros::spinOnce();
        }
    }
    log.close();
    service.stop();
    return 0;
}

