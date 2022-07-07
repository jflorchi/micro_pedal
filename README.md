micro pedal is a clone of comma.ai's comma pedal.

It functions as an analog interceptor that is able to send and receive messages over a CAN bus. When openpilot wants to press the gas, it sends a message over the bus to the pedal, and the pedal drives the DAC to the desired level. When it is not receiving messages to press the pedal, it is simply reading the ADC and then outputting that value to the DAC.

![board](https://raw.github.com/jflorchi/micro_pedal/master/board/micro_pedal_gerb.png)
 
