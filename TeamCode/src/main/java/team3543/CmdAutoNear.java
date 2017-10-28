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

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoNear implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = false;
    private static final boolean debugYPid = false;
    private static final boolean debugTurnPid = false;

    private enum State
    {
        DEPLOY_JEWEL_ARM,
        DETECT_JEWEL,
        WHACK_JEWEL,
        MOVE_JEWEL_ARM_UP,
        RESET_JEWEL_ARM,
        DO_DELAY,
        GRAB_LIFT_GLYPH,
        DRIVE_OFF_PLATFORM,
        CRAB_SIDEWAYS,
        MOVE_FORWARD, // 3-4ft
        SET_DOWN_GLYPH,
        RELEASE_GLYPH,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoNear";

    private Robot robot;
    private double delay;
    private FtcAuto.Alliance alliance;
    private boolean doJewel;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private double jewelArmSweepPosition = RobotInfo.JEWEL_ARM_NEUTRAL;

    CmdAutoNear(Robot robot, double delay, FtcAuto.Alliance alliance, boolean doJewel)
    {
        this.robot = robot;
        this.delay = delay;
        this.alliance = alliance;
        this.doJewel = doJewel;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdAutoNear

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null? sm.getState().toString(): "Disabled");

        if (sm.isReady())
        {
            state = sm.getState();

            switch (state)
            {
                //Deploy the jewel arm*
                //Wait for the jewel arm to deploy.*
                //Call grip vision to access jewel info.
                //Wait for jewel info.*
                //Disable grip vision.
                //Determine direction of arm.
                //Whack the jewel in that direction.
                //Wait for jewel to get whacked.*
                //Move jewel arm up.
                //Wait for jewel arm to move up.*
                //Move jewel arm back to zero position.
                //???
                //PROFIT
                case DEPLOY_JEWEL_ARM:
                    robot.jewelArm.setExtended(true);
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.DETECT_JEWEL);
                    break;

                case DETECT_JEWEL:
                    break;

                case WHACK_JEWEL:
                    robot.jewelArm.setSweepPosition(jewelArmSweepPosition);
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.MOVE_JEWEL_ARM_UP);
                    break;

                case MOVE_JEWEL_ARM_UP:
                    robot.jewelArm.setExtended(false);
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.RESET_JEWEL_ARM);
                    break;

                case RESET_JEWEL_ARM:
                    robot.jewelArm.setSweepPosition(RobotInfo.JEWEL_ARM_NEUTRAL);
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.DO_DELAY);
                    break;

                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.GRAB_LIFT_GLYPH);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.GRAB_LIFT_GLYPH);
                    }
                    break;

                case GRAB_LIFT_GLYPH:
                    robot.glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_CLOSE);
                    robot.glyphElevator.setPosition(RobotInfo.ELEVATOR_MID_HEIGHT, event, 2.0);
                    sm.waitForSingleEvent(event, State.DRIVE_OFF_PLATFORM);
                    break;

                case DRIVE_OFF_PLATFORM:
                    robot.targetHeading = 0.0;
                    robot.setPIDDriveTarget(0.0, 30.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.CRAB_SIDEWAYS);
                    break;

                case CRAB_SIDEWAYS:
                    // ...
                    robot.targetHeading = 0.0;
                    double xDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE ? -15.0 : 15.0;
                    robot.setPIDDriveTarget(xDistance, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_FORWARD);
                    break;

                case MOVE_FORWARD:
                    // Move forward
                    robot.targetHeading = 0.0;
                    robot.setPIDDriveTarget(0.0, 4.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.SET_DOWN_GLYPH);
                    break;

                case SET_DOWN_GLYPH:
                    // lower the elevator
                    robot.glyphElevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT, event, 2.0);
                    sm.waitForSingleEvent(event, State.RELEASE_GLYPH);
                    break;

                case RELEASE_GLYPH:
                case DONE:
                default:
                    // open the glyphgrabber servos
                    robot.glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_OPEN);
                    //
                    // We are done.
                    //
                    done = true;
                    sm.stop();
                    break;
            }
//            robot.traceStateInfo(elapsedTime, state.toString(), xDistance, yDistance, heading);
        }

        if (robot.pidDrive.isActive() && (debugXPid || debugYPid || debugTurnPid))
        {
            robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                    robot.battery.getVoltage(), robot.battery.getLowestVoltage());

//            if (debugXPid && xDistance != 0.0)
//            {
//                robot.encoderXPidCtrl.printPidInfo(robot.tracer);
//            }
//
//            if (debugYPid && yDistance != 0.0)
//            {
//                robot.encoderYPidCtrl.printPidInfo(robot.tracer);
//            }
//
//            if (debugTurnPid)
//            {
//                robot.gyroPidCtrl.printPidInfo(robot.tracer);
//            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdAutoNear
