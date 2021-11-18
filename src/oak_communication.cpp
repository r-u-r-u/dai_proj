#include <iostream>
#include "ros/ros.h"
#include "depthai/depthai.hpp"


int main(int argc, char *argv[]){
    ros::init(argc,argv,"oak_communication");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    dai::Pipeline pipeline;
    auto xLinkIn = pipeline.create<dai::node::XLinkIn>();
    auto xLinkOut = pipeline.create<dai::node::XLinkOut>();
    auto xSPIOut = pipeline.create<dai::node::SPIOut>();

    xLinkIn->setStreamName("input_stream");
    xLinkOut->setStreamName("servo");
    
    xSPIOut->setStreamName("servo");
    xSPIOut->setBusId(0);
    xSPIOut->input.setBlocking(false);

    xLinkIn->out.link(xLinkOut->input);
    xLinkIn->out.link(xSPIOut->input);
    
    dai::Device device(pipeline);

    std::shared_ptr<dai::DataInputQueue> xInQueue = device.getInputQueue("input_stream");
    std::shared_ptr<dai::DataOutputQueue> xOutQueue = device.getOutputQueue("servo",5,false);
    
    uint16_t angle = 0;
    uint16_t reachTime = 10;
    std::cout<<pipeline.getConnections().size()<<std::endl;
    while(ros::ok()){
        if(angle>1000)
            angle=0;
        else
            angle++;

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
        data.push_back(~std::accumulate(data.begin()+2,data.end(),0));
        srv.setData(data);
        xInQueue->send(srv);

        auto get_data = xOutQueue->tryGet();
        if(get_data != nullptr){
            std::for_each(get_data->getRaw()->data.begin(),get_data->getRaw()->data.end(),
            [](const int& n){std::cout<<n<<";";});
            std::cout<<std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}