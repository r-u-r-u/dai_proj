import depthai as dai
import numpy as np
import time

def main():
    pipeline = dai.Pipeline()

    xIn = pipeline.createXLinkIn()
    xIn.setStreamName("input_stream")

    xSpi = pipeline.createSPIOut()
    xSpi.setStreamName("servo")
    xSpi.setBusId(0)
    xSpi.input.setBlocking(False)

    xOut = pipeline.createXLinkOut()
    xOut.setStreamName("servo")

    xIn.out.link(xOut.input)
    xIn.out.link(xSpi.input)

    with dai.Device(pipeline) as device:
        xIn_queue = device.getInputQueue("input_stream")
        xOut_queue = device.getOutputQueue("servo",maxSize=5,blocking=False)
        reach_time = 10
        Angle = 0
        while True:
            time.sleep(0.1)
            print("angle:",Angle)

            if Angle > 1000:
                Angle = 0
            else:
                Angle += 10
            
            data_out = dai.RawBuffer()
            TxData = [
                    0x55,
                    0x55,
                    0x01,
                    0x07,
                    0x01,
                    0x00FF & (Angle),
                    0x00FF & (Angle>>8),
                    0x00FF & (reach_time),
                    0x00FF & (reach_time>>8)
                    ]
            TxData.append(~np.sum(TxData[2:]))
            data_out.data = TxData
            xIn_queue.send(data_out)


            data_out = dai.RawBuffer()
            TxData = [
                    0x55,
                    0x55,
                    0x02,
                    0x07,
                    0x01,
                    0x00FF & (Angle),
                    0x00FF & (Angle>>8),
                    0x00FF & (reach_time),
                    0x00FF & (reach_time>>8)
                    ]
            TxData.append(~np.sum(TxData[2:]))
            data_out.data = TxData
            xIn_queue.send(data_out)

            data_in = xOut_queue.tryGet()
            if data_in is not None:
                print(data_in.getRaw().data)

if __name__ == "__main__":
    main()

## Cahnge
# TxData = [
#     0x55,
#     0x55,
#     0x01,
#     0x04,
#     0x0D,
#     0x02
#     ]
