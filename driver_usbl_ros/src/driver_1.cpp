#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
// #include "visualization_msgs/Marker.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <vector>
// #include <tf2_ros/transform_broadcaster.h>

// #include <driver_1.h>
#include <seatrac_driver/SeatracDriver.h>

namespace narval { namespace seatrac{

class MySeatracDriver : public SeatracDriver
{
    public:

    MySeatracDriver(const IoServicePtr& ioService,
                    const std::string& serialDevice = "/dev/ttyUSB0") :
        SeatracDriver(ioService, serialDevice)
    {}

    protected:

    virtual void on_message(CID_E msgId, const std::vector<uint8_t>& msgData)
    {
        // your message handlers here.
    }
};
}}