## Basic Micro Roboclaw Motor 
The `roboclaw` model of the motor component supports [standard brushed DC motors](https://en.wikipedia.org/wiki/DC_motor) driven by [Basicmicro's](https://www.basicmicro.com/) [RoboClaw](https://www.basicmicro.com/RoboClaw-2x30A-Motor-Controller_p_9.html) motor controller.

You must set up your RoboClaw before configuring it.

**Some useful default values for a roboclaw:**
| Name | Value |
| ---- | ---- |
| `serial_baud_rate` | `38400` |
| `motor_channel` | `1` or `2`. |
| `address` | `128` |

Follow [this guide](https://resources.basicmicro.com/roboclaw-motor-controllers-getting-started-guide/) for further setup.

Refer to your motor and motor driver data sheets for specifics.
