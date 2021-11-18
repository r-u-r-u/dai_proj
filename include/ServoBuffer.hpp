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
}
