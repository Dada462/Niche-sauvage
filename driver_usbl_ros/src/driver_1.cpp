
#include <seatrac_driver/SeatracDriver.h>

class MySeatracDriver : public narval::seatrac::SeatracDriver
{
    public:

    MySeatracDriver(const IoServicePtr& ioService,
                    const std::string& serialDevice = "/dev/ttyUSB0") :
        seatrac::SeatracDriver(ioService, serialDevice)
    {}

    protected:

    virtual void on_message(CID_E msgId, const std::vector<uint8_t>& msgData)
    {
        // your message handlers here.
    }
};