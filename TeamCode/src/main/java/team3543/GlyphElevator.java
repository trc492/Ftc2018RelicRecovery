/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team3543;

import ftclib.FtcDcMotor;
import ftclib.FtcDigitalInput;
import trclib.TrcLinearActuator;
import trclib.TrcPidController;

public class GlyphElevator implements TrcPidController.PidInput
{
    public FtcDigitalInput elevatorLowerLimitSwitch;
    private FtcDcMotor elevatorMotor;
    public TrcPidController elevatorPidCtrl;
    public TrcLinearActuator elevator;

    /**
     * Constructor: Create an instance of the object.
     */
    public GlyphElevator()
    {
        elevatorLowerLimitSwitch = new FtcDigitalInput("elevatorLowerLimit");
        elevatorMotor = new FtcDcMotor("elevatorMotor", elevatorLowerLimitSwitch);
        elevatorPidCtrl = new TrcPidController(
                "elevatorPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ELEVATOR_KP, RobotInfo.ELEVATOR_KI, RobotInfo.ELEVATOR_KD),
                RobotInfo.ELEVATOR_TOLERANCE, this);
        elevator = new TrcLinearActuator(
                "elevator", elevatorMotor, elevatorLowerLimitSwitch, elevatorPidCtrl);
        elevator.setPositionScale(RobotInfo.ELEVATOR_INCHES_PER_COUNT, 0.0);
        elevator.setPositionRange(RobotInfo.ELEVATOR_MIN_HEIGHT, RobotInfo.ELEVATOR_MAX_HEIGHT);
    }   //GlyphElevator

    //
    // Implements TrcPidController.PidInput.
    //

    /**
     * This method is called by the PID controller to get the current height of the elevator.
     *
     * @param pidCtrl specifies the PID controller who is inquiring.
     *
     * @return current elevator height.
     */
    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == this.elevatorPidCtrl)
        {
            value = elevator.getPosition();
        }

        return value;
    }   //getInput

}   //class GlyphElevator
