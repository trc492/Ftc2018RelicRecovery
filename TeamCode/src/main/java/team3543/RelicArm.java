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
import ftclib.FtcServo;
import ftclib.FtcTouchSensor;
import trclib.TrcEnhancedServo;

public class RelicArm
{
    private FtcTouchSensor extenderLowerLimitSwitch;
    private FtcServo extenderServo;
    public TrcEnhancedServo extender;
    public FtcServo grabber;
    private FtcTouchSensor elbowLowerLimitSwitch;
    private FtcTouchSensor elbowUpperLimitSwitch;
    public FtcDcMotor elbow;

    /**
     * Constructor: Create an instance of the object  .
     */
    public RelicArm()
    {
        extenderLowerLimitSwitch = new FtcTouchSensor("extenderLowerLimit");
        extenderServo = new FtcServo("extenderServo");
        extender = new TrcEnhancedServo("extender", extenderServo, extenderLowerLimitSwitch, null);

        grabber = new FtcServo("relicGrabber");
        grabber.setPosition(RobotInfo.RELIC_GRABBER_CLOSE);

        elbowLowerLimitSwitch = new FtcTouchSensor("elbowLowerLimit");
        elbowUpperLimitSwitch = new FtcTouchSensor("elbowUpperLimit");
        elbow = new FtcDcMotor("relicArmElbow", elbowLowerLimitSwitch, elbowUpperLimitSwitch);
    }   //RelicArm

}   //class RelicArm
