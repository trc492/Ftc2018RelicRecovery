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

import android.speech.tts.TextToSpeech;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoFull implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private static final double RED_NEAR_LEFT_COL_OFFSET_IN = -11.5;
    private static final double RED_NEAR_CENTER_COL_OFFSET_IN = -4.0;
    private static final double RED_NEAR_RIGHT_COL_OFFSET_IN = 3.5;

    private static final double RED_FAR_LEFT_COL_OFFSET_IN = -18.0;
    private static final double RED_FAR_CENTER_COL_OFFSET_IN = -10.5;
    private static final double RED_FAR_RIGHT_COL_OFFSET_IN = -3.0;

    private static final double BLUE_NEAR_LEFT_COL_OFFSET_IN = 11.5;
    private static final double BLUE_NEAR_CENTER_COL_OFFSET_IN = 16.0;
    private static final double BLUE_NEAR_RIGHT_COL_OFFSET_IN = 23.5;

    private static final double BLUE_FAR_LEFT_COL_OFFSET_IN = 6.0;
    private static final double BLUE_FAR_CENTER_COL_OFFSET_IN = 13.5;
    private static final double BLUE_FAR_RIGHT_COL_OFFSET_IN = 21.0;

    private static final double SONAR_RED_NEAR_LEFT_COL_OFFSET_IN = 88.0;
    private static final double SONAR_RED_NEAR_CENTER_COL_OFFSET_IN =
            SONAR_RED_NEAR_LEFT_COL_OFFSET_IN + 7.5;
    private static final double SONAR_RED_NEAR_RIGHT_COL_OFFSET_IN =
            SONAR_RED_NEAR_CENTER_COL_OFFSET_IN + 7.5;

    private static final double SONAR_RED_FAR_LEFT_COL_OFFSET_IN = 40.0;
    private static final double SONAR_RED_FAR_CENTER_COL_OFFSET_IN =
            SONAR_RED_FAR_LEFT_COL_OFFSET_IN + 7.5;
    private static final double SONAR_RED_FAR_RIGHT_COL_OFFSET_IN =
            SONAR_RED_FAR_CENTER_COL_OFFSET_IN + 7.5;

    private static final double SONAR_BLUE_NEAR_LEFT_COL_OFFSET_IN = 21.0;
    private static final double SONAR_BLUE_NEAR_CENTER_COL_OFFSET_IN =
            SONAR_BLUE_NEAR_LEFT_COL_OFFSET_IN + 7.5;
    private static final double SONAR_BLUE_NEAR_RIGHT_COL_OFFSET_IN =
            SONAR_BLUE_NEAR_CENTER_COL_OFFSET_IN + 7.5;

    private static final double SONAR_BLUE_FAR_LEFT_COL_OFFSET_IN = 40.0;
    private static final double SONAR_BLUE_FAR_CENTER_COL_OFFSET_IN =
            SONAR_BLUE_FAR_LEFT_COL_OFFSET_IN + 7.5;
    private static final double SONAR_BLUE_FAR_RIGHT_COL_OFFSET_IN =
            SONAR_BLUE_FAR_CENTER_COL_OFFSET_IN + 7.5;

    private enum State
    {
        DEPLOY_JEWEL_ARM,
        WHACK_JEWEL,
        MOVE_JEWEL_ARM_UP,
        RESET_JEWEL_ARM,
        DO_DELAY,
        GRAB_LIFT_GLYPH,
        DRIVE_OFF_PLATFORM,
        DRIVE_TO_WALL,
        TURN_TO_CRYPTOBOX,
        ALIGN_CRYPTOBOX,
        MOVE_FORWARD,
        SET_DOWN_GLYPH,
        RELEASE_GLYPH,
        BACK_OFF,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoFull";

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private double delay;
    private FtcAuto.StartPos startPos;
    private boolean doJewel;
    private boolean doCrypto;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private int retryCount = 0;
    private RelicRecoveryVuMark vuMark;

    CmdAutoFull(
            Robot robot, FtcAuto.Alliance alliance, double delay, FtcAuto.StartPos startPos,
            boolean doJewel, boolean doCrypto)
    {
        robot.tracer.traceInfo(
                moduleName, "alliance=%s, delay=%.0f, startPos=%s, doJewel=%s, doCrypto=%s",
                alliance, delay, startPos, doJewel, doCrypto);
        this.robot = robot;
        this.alliance = alliance;
        this.delay = delay;
        this.startPos = startPos;
        this.doJewel = doJewel;
        this.doCrypto = doCrypto;

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(doJewel? State.DEPLOY_JEWEL_ARM: State.DO_DELAY);
    }   //CmdAutoFull

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
                case DEPLOY_JEWEL_ARM:
                    if (robot.jewelColorTrigger != null)
                    {
                        // Activate text to speech to tell us what color of the jewel was found.
                        robot.jewelColorTrigger.setEnabled(true);
                    }
                    //
                    // Lower jewel arm and wait for it to be done.
                    //
                    retryCount = 0;
                    robot.jewelArm.setExtended(true);
                    timer.set(0.5, event);
                    sm.waitForSingleEvent(event, State.WHACK_JEWEL);
                    break;

                case WHACK_JEWEL:
                    //
                    // Determine crypto column using Vuforia.
                    //
                    vuMark = robot.vuforiaVision.getVuMark();
                    robot.tracer.traceInfo(state.toString(), "VuMark: %s", vuMark.toString());
                    if (robot.textToSpeech != null)
                    {
                        robot.textToSpeech.speak(
                                String.format("%s is in view.", vuMark), TextToSpeech.QUEUE_ADD, null);
                    }

                    // determine the jewel color and whack the correct one.
                    Robot.ObjectColor jewelColor = robot.getObjectColor(robot.jewelColorSensor);

                    robot.tracer.traceInfo(
                            state.toString(), "%d: Color=%s, HSV=[%f/%f/%f]",
                            retryCount, jewelColor.toString(),
                            robot.getObjectHsvHue(robot.jewelColorSensor),
                            robot.getObjectHsvSaturation(robot.jewelColorSensor),
                            robot.getObjectHsvValue(robot.jewelColorSensor));
                    if (jewelColor == Robot.ObjectColor.NO && retryCount < 10)
                    {
                        //
                        // Failed to determine jewel color, try again up to 10 times.
                        //
                        retryCount++;
                        break;
                    }

                    if (robot.jewelColorTrigger != null)
                    {
                        //
                        // Done with jewel color sensor, disable it.
                        //
                        robot.jewelColorTrigger.setEnabled(false);
                    }

                    double sweepPosition =
                            jewelColor == Robot.ObjectColor.RED && alliance == FtcAuto.Alliance.RED_ALLIANCE ||
                            jewelColor == Robot.ObjectColor.BLUE && alliance == FtcAuto.Alliance.BLUE_ALLIANCE ?
                                    RobotInfo.JEWEL_ARM_FORWARD :
                            jewelColor == Robot.ObjectColor.BLUE && alliance == FtcAuto.Alliance.RED_ALLIANCE ||
                            jewelColor == Robot.ObjectColor.RED && alliance == FtcAuto.Alliance.BLUE_ALLIANCE ?
                                    RobotInfo.JEWEL_ARM_BACKWARD :
                                    RobotInfo.JEWEL_ARM_NEUTRAL;
                    robot.jewelArm.setSweepPosition(sweepPosition);
                    timer.set(0.5, event);
                    sm.waitForSingleEvent(event, State.MOVE_JEWEL_ARM_UP);
                    break;

                case MOVE_JEWEL_ARM_UP:
                    //
                    // Done whacking jewel, bring jewel arm back up.
                    //
                    robot.jewelArm.setExtended(false);
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.RESET_JEWEL_ARM);
                    break;

                case RESET_JEWEL_ARM:
                    //
                    // Return jewel arm back to neutral position only after jewel arm is up. This is to prevent
                    // accidental whacking the other jewel while bringing it back up.
                    //
                    robot.jewelArm.setSweepPosition(RobotInfo.JEWEL_ARM_NEUTRAL);
                    timer.set(0.3, event);
                    sm.waitForSingleEvent(event, State.DO_DELAY);
                    break;

                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    robot.tracer.traceInfo(state.toString(), "Delay=%.0f", delay);
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
                    //
                    // Grab the glyph block in front of us and lift it off the platform.
                    //
                    robot.glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_CLOSE);
                    robot.glyphElevator.setPosition(RobotInfo.ELEVATOR_MID_HEIGHT, event, 2.0);
                    sm.waitForSingleEvent(event, State.DRIVE_OFF_PLATFORM);
                    break;

                case DRIVE_OFF_PLATFORM:
                    //
                    // Carefully drive off the platform with only half power.
                    //
                    if (robot.sonarArray != null)
                    {
                        robot.sonarArray.startRanging(true);
                    }
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    targetX = 0.0;
                    targetY = alliance == FtcAuto.Alliance.RED_ALLIANCE ? -22.0 : 25.0;
                    robot.targetHeading = 0.0;

                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 2.0);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_WALL);
                    break;

                case DRIVE_TO_WALL:
                    if (robot.sonarArray != null)
                    {
                        //
                        // Drive forward to the wall using sonar array.
                        //
                        targetY = 12.0;
                        robot.sonarYPidDrive.setTarget(targetY, robot.targetHeading, false, event, 2.0);
                    }
                    else if (robot.rangeDrive != null)
                    {
                        //
                        // Drive forward to the wall using Modern Robotics Range sensor.
                        //
                        targetY = 12.0;
                        robot.rangeDrive.setTarget(targetY, robot.targetHeading, false, event, 2.0);
                    }
                    else
                    {
                        //
                        // Drive forward to the wall using encoders.
                        //
                        targetY = 16.0;
                        robot.pidDrive.setTarget(targetY, robot.targetHeading, false, event, 2.0);
                    }
                    sm.waitForSingleEvent(event, State.TURN_TO_CRYPTOBOX);
                    break;

                case TURN_TO_CRYPTOBOX:
                    //
                    // Turn towards the crypto box if necessary.
                    //
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    targetX = 0.0;
                    targetY = 0.0;
                    robot.targetHeading =
                            startPos == FtcAuto.StartPos.FAR ? -90.0 :
                            alliance == FtcAuto.Alliance.RED_ALLIANCE && startPos == FtcAuto.StartPos.NEAR ? 180.0 : 0.0;

                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 3.0);
                    sm.waitForSingleEvent(event, State.ALIGN_CRYPTOBOX);
                    break;

                case ALIGN_CRYPTOBOX:
                    //
                    // Crab sideways to the correct crypto box column according to the VuMark.
                    //
                    if (robot.cryptoColorTrigger != null)
                    {
                        //
                        // Using color sensor to count the crypto column bars.
                        //
                        robot.redCryptoBarCount = 0;
                        robot.blueCryptoBarCount = 0;
                        robot.cryptoColorTrigger.setEnabled(true);
                    }

                    targetY = 0.0;
                    if (!doCrypto)
                    {
                        //
                        // In case vision is malfunctioning, we are not doing crypto box using vision, so just crab
                        // to the parking zone and stop. While doing this, might as well drop the glyph to wherever
                        // we landed hoping to score something.
                        //
                        targetX = 12.0;
                        robot.pidDrive.setTarget(
                                targetX, targetY, robot.targetHeading, false, event, 2.0);
                    }
                    else if (robot.sonarArray != null)
                    {
                        //
                        // Use the sonar array to guide us to the correct crypto column.
                        //
                        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            robot.useRightSonarForX = true;
                            if (startPos == FtcAuto.StartPos.NEAR)
                            {
                                if (vuMark == RelicRecoveryVuMark.LEFT)
                                {
                                    targetX = SONAR_RED_NEAR_LEFT_COL_OFFSET_IN;
                                }
                                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                                {
                                    targetX = SONAR_RED_NEAR_RIGHT_COL_OFFSET_IN;
                                }
                                else
                                {
                                    targetX = SONAR_RED_NEAR_CENTER_COL_OFFSET_IN;
                                }
                            }
                            else
                            {
                                if (vuMark == RelicRecoveryVuMark.LEFT)
                                {
                                    targetX = SONAR_RED_FAR_LEFT_COL_OFFSET_IN;
                                }
                                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                                {
                                    targetX = SONAR_RED_FAR_RIGHT_COL_OFFSET_IN;
                                }
                                else
                                {
                                    targetX = SONAR_RED_FAR_CENTER_COL_OFFSET_IN;
                                }
                            }
                        }
                        else
                        {
                            robot.useRightSonarForX = false;
                            if (startPos == FtcAuto.StartPos.NEAR)
                            {
                                if (vuMark == RelicRecoveryVuMark.LEFT)
                                {
                                    targetX = SONAR_BLUE_NEAR_LEFT_COL_OFFSET_IN;
                                }
                                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                                {
                                    targetX = SONAR_BLUE_NEAR_RIGHT_COL_OFFSET_IN;
                                }
                                else
                                {
                                    targetX = SONAR_BLUE_NEAR_CENTER_COL_OFFSET_IN;
                                }
                            }
                            else
                            {
                                if (vuMark == RelicRecoveryVuMark.LEFT)
                                {
                                    targetX = SONAR_BLUE_FAR_LEFT_COL_OFFSET_IN;
                                }
                                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                                {
                                    targetX = SONAR_BLUE_FAR_RIGHT_COL_OFFSET_IN;
                                }
                                else
                                {
                                    targetX = SONAR_BLUE_FAR_CENTER_COL_OFFSET_IN;
                                }
                            }
                        }
                        robot.sonarXPidDrive.setTarget(
                                targetX, targetY, robot.targetHeading, false, event, 2.0);
                    }
                    else
                    {
                        //
                        // Using encoder dead reckoning to guide us to the correct crypto column.
                        //
                        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
                        {
                            if (startPos == FtcAuto.StartPos.NEAR)
                            {
                                if (vuMark == RelicRecoveryVuMark.LEFT)
                                {
                                    targetX = RED_NEAR_LEFT_COL_OFFSET_IN;
                                }
                                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                                {
                                    targetX = RED_NEAR_RIGHT_COL_OFFSET_IN;
                                }
                                else
                                {
                                    targetX = RED_NEAR_CENTER_COL_OFFSET_IN;
                                }
                            }
                            else
                            {
                                if (vuMark == RelicRecoveryVuMark.LEFT)
                                {
                                    targetX = RED_FAR_LEFT_COL_OFFSET_IN;
                                }
                                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                                {
                                    targetX = RED_FAR_RIGHT_COL_OFFSET_IN;
                                }
                                else
                                {
                                    targetX = RED_FAR_CENTER_COL_OFFSET_IN;
                                }
                            }
                        }
                        else
                        {
                            if (startPos == FtcAuto.StartPos.NEAR)
                            {
                                if (vuMark == RelicRecoveryVuMark.LEFT)
                                {
                                    targetX = BLUE_NEAR_LEFT_COL_OFFSET_IN;
                                }
                                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                                {
                                    targetX = BLUE_NEAR_RIGHT_COL_OFFSET_IN;
                                }
                                else
                                {
                                    targetX = BLUE_NEAR_CENTER_COL_OFFSET_IN;
                                }
                            }
                            else
                            {
                                if (vuMark == RelicRecoveryVuMark.LEFT)
                                {
                                    targetX = BLUE_FAR_LEFT_COL_OFFSET_IN;
                                }
                                else if (vuMark == RelicRecoveryVuMark.RIGHT)
                                {
                                    targetX = BLUE_FAR_RIGHT_COL_OFFSET_IN;
                                }
                                else
                                {
                                    targetX = BLUE_FAR_CENTER_COL_OFFSET_IN;
                                }
                            }
                        }
                        robot.pidDrive.setTarget(
                                targetX, targetY, robot.targetHeading, false, event, 2.0);
                    }
                    sm.waitForSingleEvent(event, State.MOVE_FORWARD);
                    break;

                case MOVE_FORWARD:
                    if (robot.cryptoColorTrigger != null)
                    {
                        //
                        // We are done with crypto box navigation, turn of crypto color sensor.
                        //
                        robot.cryptoColorTrigger.setEnabled(false);
                    }
                    //
                    // Move forward a little to push the glyph block into the crypto column.
                    //
                    targetX = 0.0;
                    if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        targetY = 15.0;
                    }
                    else
                    {
                        targetY = 9.0;
                    }

                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 1.0);
                    sm.waitForSingleEvent(event, State.SET_DOWN_GLYPH);
                    break;

                case SET_DOWN_GLYPH:
                    if (robot.sonarArray != null)
                    {
                        //
                        // We are done with crypto navigation, turn off sonar array.
                        //
                        robot.sonarArray.stopRanging();
                    }
                    // lower the elevator
                    robot.glyphElevator.setPosition(RobotInfo.ELEVATOR_MIN_HEIGHT, event, 2.0);
                    sm.waitForSingleEvent(event, State.RELEASE_GLYPH);
                    break;

                case RELEASE_GLYPH:
                    robot.glyphGrabber.setPosition(RobotInfo.GLYPH_GRABBER_OPEN);
                    timer.set(0.5, event);
                    sm.waitForSingleEvent(event, State.BACK_OFF);
                    break;

                case BACK_OFF:
                    //
                    // Back off a little to make sure we are not touching the glyph block.
                    //
                    targetX = 0.0;
                    targetY = -4.5;

                    robot.pidDrive.setTarget(targetX, targetY, robot.targetHeading, false, event, 1.0);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    done = true;
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), targetX, targetY, robot.targetHeading);
        }

        if (robot.pidDrive.isActive() || robot.sonarXPidDrive.isActive() || robot.sonarYPidDrive.isActive())
        {
            robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                    robot.battery.getVoltage(), robot.battery.getLowestVoltage());
        }

        if (robot.pidDrive.isActive())
        {
            if (debugXPid && targetX != 0.0)
            {
                robot.encoderXPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugYPid && targetY != 0.0)
            {
                robot.encoderYPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugTurnPid)
            {
                robot.gyroPidCtrl.printPidInfo(robot.tracer);
            }
        }
        else if (robot.sonarXPidDrive.isActive())
        {
            if (debugXPid)
            {
                robot.sonarXPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugTurnPid)
            {
                robot.gyroPidCtrl.printPidInfo(robot.tracer);
            }
        }
        else if (robot.sonarYPidDrive.isActive())
        {
            if (debugYPid)
            {
                robot.sonarYPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugTurnPid)
            {
                robot.gyroPidCtrl.printPidInfo(robot.tracer);
            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdAutoFull
