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

class CmdRangeDrive implements TrcRobot.RobotCommand
{
    private static final boolean debugRangePid = true;

    private enum State
    {
        DO_RANGE_DRIVE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdRangeDrive";

    private Robot robot;
    private double rangeDistance;
    private TrcEvent event;
    private TrcStateMachine<State> sm;

    CmdRangeDrive(Robot robot, double rangeDistance)
    {
        this.robot = robot;
        this.rangeDistance = rangeDistance;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_RANGE_DRIVE);
    }   //CmdRangeDrive

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
        robot.dashboard.displayPrintf(1, "State: %s", state != null ? sm.getState().toString() : "Disabled");

        if (sm.isReady())
        {
            state = sm.getState();

            switch (state)
            {
                case DO_RANGE_DRIVE:
                    robot.rangeDrive.setTarget(rangeDistance, 0.0, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                default:
                    //
                    // We are done.
                    //
                    done = true;
                    sm.stop();
                    break;
            }
        }

        if (robot.rangeDrive.isActive() && debugRangePid)
        {
            robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                    robot.battery.getVoltage(), robot.battery.getLowestVoltage());

            if (debugRangePid)
            {
                robot.rangePidCtrl.printPidInfo(robot.tracer, elapsedTime);
                robot.gyroPidCtrl.printPidInfo(robot.tracer, elapsedTime);
                robot.rangePidCtrl.displayPidInfo(10);
                robot.gyroPidCtrl.displayPidInfo(12);
            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdRangeDrive
