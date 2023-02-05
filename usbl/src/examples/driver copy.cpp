#include <iostream>
using namespace std;
#include <chrono>
#include <thread>

#include <seatrac_driver/AsyncService.h>
#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
using namespace narval::seatrac;


#include <seatrac_driver/messages/Messages.h>

namespace narval { namespace seatrac {

SeatracDriver::SeatracDriver(const IoServicePtr& ioService,
                             const std::string& port) :
    SeatracSerial(ioService, port)
{}


void SeatracDriver::on_receive(const std::vector<uint8_t>& data)
{
    // main dispatch function

    CID_E msgId = (CID_E)data[0]; // TODO check validity
    {
        // Iterating on waiters and setting data when msgId match.
        // If there is a match, waiter is to be deleted.
        std::unique_lock<std::mutex> lock(waitersMutex_);
        auto it = waiters_.begin();
        while(it != waiters_.end()) {
            if((*it)->msg_id() == msgId) {
                (*it)->set_data(data);
                waiters_.erase(it++);
            }
            else {
                it++;
            }
        }
    }
    this->on_message(msgId, data);
}

struct usbl_data{
  int16_t altitude;
};

void SeatracDriver::on_message(CID_E msgId, const std::vector<uint8_t>& data)
{
    if (msgId==CID_PING_RESP)
    {
        messages::PingResp response;
        response = data;
        
        // data_stored.attitude=response.acoFix.attitudePitch;
        std::cout << response.acoFix. << std::endl;

        // for (int i=0;i<sizeof(data);i++)
        // {
            // std::cout << data[i] << std::endl;
        // }
    }
    // switch(msgId) {
    //     default:
    //         // std::cout << "Got message : " << msgId << std::endl << std::flush;
    //         break;
    //     case CID_PING_ERROR:
    //         {
    //             messages::PingError response;
    //             response = data;
    //             // std::cout << response<< std::endl;
    //         }
    //         break;
    //     case CID_PING_RESP:
    //         // std::cout << "Got a Ping Response" << std::endl << std::flush;
    //         {
    //             messages::PingResp response;
    //             response = data;
    //             // std::cout << response.acoFix.range << std::endl;
    //         }
    //         break;
    //     //case CID_STATUS:
    //     //    break;
    // }
}

}; //namespace seatrac
}; //namespace narval




int main(int argc, char** argv)
{
    const char* device = "/dev/ttyUSB0";
    if(argc > 1) device = argv[1];

    AsyncService service;
    SeatracDriver seatrac(service.io_service(), device);
    seatrac.enable_io_dump();

    service.start();
    usbl_data data_stored;
    narval::seatrac::messages::PingSend response;
    narval::seatrac::messages::PingSend::Request request;
    AMSGTYPE_E pingType = MSG_REQ;
    request.target   = BEACON_ID_15;
    request.pingType = pingType;
    // while(true) {
    //     seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response1, 1000);
    //     cout<<response;
    //     cout<<response1<<endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }

    ros::init(argc, argv, "USBL");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Informations", 1000);
    std_msgs::String msg;
    int f=1; //Frequency (Hz)
    ros::Rate loop_rate(f);
    while (ros::ok()){
        seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response, 1000);
        cout<<response<<endl;
        msg.data = "to be defined";
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    service.stop();

    return 0;
}
