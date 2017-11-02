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
import ftclib.FtcServo;
import trclib.TrcEnhancedServo;
import trclib.TrcPidController;
import trclib.TrcRotationalActuator;

public class RelicArm implements TrcPidController.PidInput, TrcPidController.PidOutputCompensation
{
    public FtcDigitalInput extenderLowerLimitSwitch;
    public FtcDigitalInput extenderUpperLimitSwitch;
    private FtcServo extenderServo;
    public TrcEnhancedServo extender;

    public FtcDigitalInput elbowLowerLimitSwitch;
    public FtcDigitalInput elbowUpperLimitSwitch;
    private FtcDcMotor elbowMotor;
    public TrcPidController elbowPidCtrl;
    public TrcRotationalActuator elbow;

    public FtcServo grabber;

    /**
     * Constructor: Create an instance of the object  .
     */
    public RelicArm()
    {
        //
        // Relic arm consists of three subsystems:
        // - Extender
        // - Elbow
        // - Grabber
        //
        extenderLowerLimitSwitch = new FtcDigitalInput("extenderLowerLimit");
        extenderUpperLimitSwitch = new FtcDigitalInput("extenderUpperLimit");
        extenderServo = new FtcServo("extenderServo");
        extenderServo.setInverted(true);
        extender = new TrcEnhancedServo(
                "extender", extenderServo, extenderLowerLimitSwitch, extenderUpperLimitSwitch);

        elbowLowerLimitSwitch = new FtcDigitalInput("elbowLowerLimit");
        elbowUpperLimitSwitch = new FtcDigitalInput("elbowUpperLimit");
        elbowMotor = new FtcDcMotor("relicArmElbow", elbowLowerLimitSwitch, elbowUpperLimitSwitch);
        elbowPidCtrl = new TrcPidController(
                "elbowPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.RELIC_ELBOW_KP, RobotInfo.RELIC_ELBOW_KI, RobotInfo.RELIC_ELBOW_KD),
                RobotInfo.RELIC_ELBOW_TOLERANCE, this, this);
        elbow = new TrcRotationalActuator(
                "elbow", elbowMotor, elbowLowerLimitSwitch, elbowPidCtrl, this);
        elbow.setPositionScale(RobotInfo.RELIC_ELBOW_DEGREES_PER_COUNT, RobotInfo.RELIC_ELBOW_POS_OFFSET);
//???        elbow.setPositionRange(RobotInfo.RELIC_ELBOW_MIN_POS, RobotInfo.RELIC_ELBOW_MAX_POS);

        grabber = new FtcServo("relicGrabber");
        grabber.setInverted(true);
    }   //RelicArm

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

        if (pidCtrl == this.elbowPidCtrl)
        {
            value = elbow.getPosition();
        }

        return value;
    }   //getInput

    //
    // Implements TrcPidController.PidOutputCompensation interface
    //

    /**
     * This method is called by the PID controller to get output compensation. The output compensation will be
     * added to the output power calculation.
     *
     * @param pidCtrl specifies this PID controller so the provider can identify what sensor to read if it is
     *                a provider for multiple PID controllers.
     * @return compensation value of the actuator.
     */
    @Override
    public double getOutputCompensation(TrcPidController pidCtrl)
    {
        double compensation = 0.0;

        if (pidCtrl == elbowPidCtrl)
        {
            compensation = Math.cos(Math.toRadians(elbow.getPosition())) * RobotInfo.RELIC_ELBOW_LEVEL_MOTOR_POWER;
        }

        return compensation;
    }   //getOutputCompensation

}   //class RelicArm
