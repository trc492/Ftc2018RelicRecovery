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
import ftclib.FtcTouchSensor;
import trclib.TrcLinearActuator;
import trclib.TrcPidController;

public class GlyphElevator extends TrcLinearActuator
{
    private FtcTouchSensor lowerLimitSwitch;
    private FtcDcMotor motor;
    private TrcPidController pidCtrl;

    /**
     * Constructor: Create an instance of the object.
     */
    public GlyphElevator()
    {
        super("glyphElevator");
        lowerLimitSwitch = new FtcTouchSensor("elevatorLowerLimit");
        motor = new FtcDcMotor("elevatorMotor", lowerLimitSwitch);
        pidCtrl = new TrcPidController(
                "elevatorPidCtrl",
                RobotInfo.ELEVATOR_KP, RobotInfo.ELEVATOR_KI, RobotInfo.ELEVATOR_KD, RobotInfo.ELEVATOR_KF,
                RobotInfo.ELEVATOR_TOLERANCE, RobotInfo.ELEVATOR_SETTLING,
                this);
        pidCtrl.setAbsoluteSetPoint(true);
        initialize(motor, lowerLimitSwitch, pidCtrl);
        setPositionScale(RobotInfo.ELEVATOR_INCHES_PER_COUNT);
        setPositionRange(RobotInfo.ELEVATOR_MIN_HEIGHT, RobotInfo.ELEVATOR_MAX_HEIGHT);
    }   //GlyphElevator

}   //class GlyphElevator
