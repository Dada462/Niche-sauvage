#include <iostream>
#include <ctime>
#include <fstream>
#include <chrono>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "usbl/USBL.h"

using namespace narval::seatrac;

void log_write(std::ofstream &log, messages::PingResp &data_response)
{
    log << float(data_response.acoFix.position.northing) / 10.0 << ", ";
    log << float(data_response.acoFix.position.easting) / 10.0 << ", ";
    log << float(data_response.acoFix.position.depth) / 10.0 << ", ";
    log << float(data_response.acoFix.usbl.azimuth) / 10.0 << ", ";
    log << float(data_response.acoFix.usbl.elevation) / 10.0 << ",";
    log << float(data_response.acoFix.range.dist) / 10.0 << ",";
    log << float(data_response.acoFix.depthLocal) / 10.0 << std::endl;
}

std::string current_time()
{
    time_t now = time(0);

    // convert now to string form
    char *dt = ctime(&now);

    std::cout << "The local date and time is: " << dt << std::endl;

    // convert now to tm struct for UTC
    // tm *gmtm = gmtime(&now);
    // dt = asctime(gmtm);
    char *test = new char[8];
    for (int i = 0; i < 8; i++)
    {
        test[i] = dt[i + 11];
    }
    std::string s(test, 8);
    return s;
}

void write_message(usbl::USBL &data, const narval::seatrac::messages::PingResp &data_response)
{
    data.azimuth = float(data_response.acoFix.usbl.azimuth) / 10;
    data.elevation = float(data_response.acoFix.usbl.elevation) / 10;
    data.range = float(data_response.acoFix.range.dist) / 10;
    data.depth = float(data_response.acoFix.depthLocal) / 10;
    data.position.x = float(data_response.acoFix.position.easting) / 10;
    data.position.y = float(data_response.acoFix.position.northing) / 10;
    data.position.z = float(data_response.acoFix.position.depth) / 10;
}

class MyDriver : public SeatracDriver
{
public:
    MyDriver(const std::string &serialPort = "/dev/ttyUSB0") : SeatracDriver(serialPort)
    {
    }

    void ping_beacon(BID_E target, AMSGTYPE_E pingType = MSG_REQU)
    {
        messages::PingSend::Request req;
        req.target = target;
        req.pingType = pingType;
        this->send(sizeof(req), (const uint8_t *)&req);
    }
    usbl::USBL USBL_info_message;
    messages::PingResp response_data;
    std::ofstream log;
    ros::Publisher pub;

    bool data_available = false;

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t> &data)
    {
        // replace code in this method by your own
        switch (msgId)
        {
        default:
            // std::cout << "Got message BLABLA: " << msgId << std::endl
                    //   << std::flush;
            break;
        case CID_PING_ERROR:
        {
            messages::PingError response;
            response = data;
            // std::cout << response << std::endl;
            this->ping_beacon(response.beaconId, MSG_REQU);
        }
        break;
        case CID_PING_RESP:
            std::cout << "Got a Ping Response" << std::endl << std::flush;
            {
                response_data = data;
                // std::cout << response_data << std::endl;
                log_write(log, response_data);
                write_message(USBL_info_message, response_data);
                this->ping_beacon(response_data.acoFix.srcId, MSG_REQU);
                pub.publish(USBL_info_message);
                ros::spinOnce();
            }
            break;
        case CID_STATUS:
            // too many STATUS messages so bypassing display.
            break;
        }
    }
};

int main(int argc, char **argv)
{
    MyDriver seatrac("/dev/ttyUSB0");
    ros::init(argc, argv, "USBL_pub_node");
    ros::NodeHandle n;
    ros::Publisher usbl_info_pub = n.advertise<usbl::USBL>("USBL", 1000);
    seatrac.pub = usbl_info_pub;
    seatrac.log.open("src/usbl/logs/October_27_" + current_time() + ".dat");
    seatrac.log << "LOG: northing, easting, depth, azimith, elevation, range, Local depth" << std::endl;

    command::ping_send(seatrac, BEACON_ID_1, MSG_REQU);
    getchar();
    seatrac.log.close();

    return 0;
}