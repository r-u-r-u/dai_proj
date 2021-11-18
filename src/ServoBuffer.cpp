namespace::dai{

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