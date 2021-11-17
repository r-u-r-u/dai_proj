#include <iostream>
#include "ros/ros.h"
#include "depthai/depthai.hpp"

namespace dai{
    struct RawServoControl :public RawBuffer{
        enum class mode:uint8_t{
        A=0x01,
        B=0x02,  
        };
    };
    class ServoControl :public Buffer{
        std::shared_ptr<dai::RawBuffer> serialize() const override;
    public:
        ServoControl();
        explicit ServoControl(std::shared_ptr<dai::RawServoControl> ptr);
        virtual ~ServoControl() = default;

        void setAngle();
        void setReachTime();
        void setMode();
    };
    std::shared_ptr<dai::RawBuffer> ServoControl::serialize() const {
        return raw;
    }
    ServoControl::ServoControl():
        Buffer(std::make_shared<RawServoControl>()){}
    ServoControl::ServoControl(std::shared_ptr<dai::RawServoControl> ptr):
        Buffer(std::move(ptr)){}

    void ServoControl::setAngle(){
        
    };
    void ServoControl::setReachTime(){

    };
    void ServoControl::setMode(){
        
    };
}


dai::Pipeline createPipeline(){
    dai::Pipeline pipeline;
    auto xLinkIn = pipeline.create<dai::node::XLinkIn>();
    auto xLinkOut = pipeline.create<dai::node::XLinkOut>();
    auto xSPIOut = pipeline.create<dai::node::SPIOut>();

    xLinkIn->setStreamName("communication");
    xLinkOut->setStreamName("communication");
    
    xSPIOut->setStreamName("communication");
    xSPIOut->setBusId(0);
    xSPIOut->input.setBlocking(false);

    xLinkIn->out.link(xLinkOut->input);
    xLinkIn->out.link(xSPIOut->input);
    return pipeline;
}

int main(int argc, char *argv[]){
    ros::init(argc,argv,"oak_communication");
    ros::NodeHandle nh;

    dai::Pipeline pipeline = createPipeline();
    dai::Device device(pipeline);

    std::shared_ptr<dai::DataInputQueue> xInQueue = device.getInputQueue("communication");
    std::shared_ptr<dai::DataOutputQueue> xOutQueue = device.getOutputQueue("communication",5,false);
    
    uint16_t angle = 0;
    uint16_t reachTime = 0;
    std::cout<<"hello"<<std::endl;
    while(ros::ok()){
        dai::Buffer srv;
        std::vector<uint8_t> data{
            0x55,
            0x55,
            0x01,
            0x07,
            0x01,
            0x00FF & angle,
            0x00FF & angle>>8,
            0x00FF & reachTime,
            0x00FF & reachTime>>8,
            };
        data.push_back(~std::accumulate(data.begin(),data.end(),0));
        srv.setData(data);
        xInQueue->send(srv);
        auto get_data = xOutQueue->tryGet();
        if(get_data != nullptr){
            std::for_each(get_data->getRaw()->data.begin(),get_data->getRaw()->data.end(),
            [](const int& n){std::cout<<n<<";";});
            std::cout<<std::endl;
        }
    }
    
    ros::spin();
    return 0;
}