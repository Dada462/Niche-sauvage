#include <iostream>
using namespace std;
#include <fstream>
#include <seatrac_driver/AsyncService.h>
#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "guerledan_usbl/USBL.h"
#include "vibes.h"
using namespace narval::seatrac;

narval::seatrac::messages::Status get_next_status(SeatracDriver& seatrac)
{
    narval::seatrac::messages::Status status;

    if(!seatrac.wait_for_message(CID_STATUS, &status, 1000)) {
        throw TimeoutReached();
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
                // std::cout << "Got message : " << msgId << std::endl << std::flush;
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
                    //messages::PingResp response;
                    lastResponse_ = msgData;
                    data_available=true;
                    // std::cout << lastResponse_ << std::endl;
                    // PingResp& operator=(const std::vector<uint8_t>& other)
                    // {
                    //     if(other[0] != this->msgId) {
                    //         throw std::runtime_error("Wrong message for decoding.");
                    //     }
                    //     acoFix.assign(other.size() - 1, other.data() + 1);
                    //     return *this;
                    // }
                }
                break;
            //case CID_STATUS:
            //    break;
        }
    }
};

int main(int argc, char** argv)
{
    const char* device = "/dev/ttyUSB0";
    if(argc > 1) device = argv[1];

    AsyncService service;
    MyDriver seatrac(service.io_service(), device);
    seatrac.enable_io_dump();

    service.start();
    ros::init(argc, argv, "USBL");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<guerledan_usbl::USBL>("Informations", 1000);
    guerledan_usbl::USBL msg;
    float freq=1; //Frequency (Hz)
    ros::Rate loop_rate(freq);
    //vibes::beginDrawing(); // initialisation de VIBes
    //vibes::newFigure("Road");
	// vibes::clearFigure("Road");
    // vibes::setFigureProperties(vibesParams("x",100,"y",100,"width",600,"height",600));
    // vibes::axisLimits(-10,10,-10,10);
    ofstream outfile;
    outfile.open("src/guelerdan_usbl/src/logs_test_10_October.dat");
    outfile<<"LOG"<<endl;
    while (ros::ok()){
        // loop_rate.sleep();
        // std::cout << "turning"<< std::endl;
        command::ping_send(seatrac, BEACON_ID_1, MSG_REQU);
        if (seatrac.data_available)
        {
            std::cout << (seatrac.lastResponse_.acoFix.attitudeYaw) << std::endl;
            outfile<<seatrac.lastResponse_.acoFix.usbl.azimuth<<", ";
            outfile<<(seatrac.lastResponse_.acoFix.usbl.elevation)<<",";
            outfile<<(seatrac.lastResponse_.acoFix.range.dist)<<endl;
            outfile<<(seatrac.lastResponse_.acoFix.depthLocal)<<endl;
            msg.azimuth = seatrac.lastResponse_.acoFix.usbl.azimuth;
            msg.elevation = seatrac.lastResponse_.acoFix.usbl.elevation;
            msg.range = seatrac.lastResponse_.acoFix.range.dist;
            chatter_pub.publish(msg);
            seatrac.data_available =false;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
        // if (command::ping_send(seatrac, BEACON_ID_2).msgId==CID_PING_RESP)
        // {
        //     std::cout << seatrac.lastResponse_.acoFix.attitudeYaw<< std::endl;
        //     std::cout << "GOT response"<< std::endl;
        //     // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //     // msg.range =seatrac.lastResponse_.acoFix.depthLocal;
        //     // chatter_pub.publish(msg);
        //     // ros::spinOnce();
        // }
        // else{
        //     std::cout << "no response"<< std::endl;

        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    outfile.close();
    service.stop();

    return 0;
}

