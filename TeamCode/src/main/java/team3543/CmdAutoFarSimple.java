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

class CmdAutoFarSimple implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = false;
    private static final boolean debugYPid = false;
    private static final boolean debugTurnPid = false;

    private enum State
    {
        DO_DELAY,
        GRAB_LIFT_GLYPH,
        DRIVE_OFF_PLATFORM,// 3ft
        TURN_TO_CRYPTOBOX, //
        MOVE_FORWARD,
        SET_DOWN_GLYPH,
        RELEASE_GLYPH,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoFarSimple";

    private Robot robot;
    private double delay;
    private FtcAuto.Alliance alliance;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdAutoFarSimple(Robot robot, double delay, FtcAuto.Alliance alliance)
    {
        this.robot = robot;
        this.delay = delay;
        this.alliance = alliance;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdAutoFarSimple

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
                    robot.setPIDDriveTarget(0.0, 36.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CRYPTOBOX);
                    break;

                case TURN_TO_CRYPTOBOX:
                    // ...
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE ? 90.0 : -90.0;
                    robot.setPIDDriveTarget(0.0, 0.0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_FORWARD);
                    break;

                case MOVE_FORWARD:
                    // Move forward
                    robot.setPIDDriveTarget(0.0, 12.0, robot.targetHeading, false, event);
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

}   //class CmdAutoFarSimple
