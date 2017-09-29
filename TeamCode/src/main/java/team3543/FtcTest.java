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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import ftclib.FtcChoiceMenu;
import ftclib.FtcGamepad;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

@TeleOp(name="Test", group="3543Test")
public class FtcTest extends FtcTeleOp implements FtcMenu.MenuButtons, FtcGamepad.ButtonHandler
{
    private static final String moduleName = "FtcTest";

    private enum Test
    {
        SENSORS_TEST,
        MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        X_DISTANCE_DRIVE,
        Y_DISTANCE_DRIVE,
        GYRO_TURN,
        VISION_TEST
    }   //enum Test

    private enum State
    {
        START,
        DONE
    }   //enum State

    //
    // State machine.
    //
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    //
    // Menu choices.
    //
    private Test test = Test.SENSORS_TEST;
    private double driveTime = 0.0;
    private double driveDistance = 0.0;
    private double turnDegrees = 0.0;

    private CmdTimedDrive timedDriveCommand = null;
    private CmdPidDrive pidDriveCommand = null;

    private int motorIndex = 0;

    private VuforiaVision vuforiaVision = null;
    private RelicRecoveryVuMark prevVuMark = null;

    //
    // Implements FtcOpMode interface.
    //

    @Override
    public void initRobot()
    {
        //
        // FtcTest inherits from FtcTeleOp so it can do everything that FtcTeleOp can do and more.
        //
        super.initRobot();
        //
        // Initialize additional objects.
        //
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        //
        // Test menus.
        //
        doMenus();

        switch (test)
        {
            case X_TIMED_DRIVE:
                timedDriveCommand = new CmdTimedDrive(robot, 0.0, driveTime, 1.0, 0.0, 0.0);
                break;

            case Y_TIMED_DRIVE:
                timedDriveCommand = new CmdTimedDrive(robot, 0.0, driveTime, 0.0, 0.2, 0.0);
                break;

            case X_DISTANCE_DRIVE:
                pidDriveCommand = new CmdPidDrive(robot, 0.0, driveDistance*12.0, 0.0, 0.0);
                break;

            case Y_DISTANCE_DRIVE:
                pidDriveCommand = new CmdPidDrive(robot, 0.0, 0.0, driveDistance*12.0, 0.0);
                break;

            case GYRO_TURN:
                pidDriveCommand = new CmdPidDrive(robot, 0.0, 0.0, 0.0, turnDegrees);
                break;

            case VISION_TEST:
                int cameraViewId = hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                vuforiaVision = new VuforiaVision(robot, cameraViewId);
                break;
        }

        sm.start(State.START);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        super.startMode();
        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(true);
        }
    }   //startMode

    @Override
    public void stopMode()
    {
        super.stopMode();
        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(false);
        }
    }   //stopMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //
    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // Allow TeleOp to run so we can control the robot in sensors test mode.
        //
        switch (test)
        {
            case SENSORS_TEST:
            case VISION_TEST:
                super.runPeriodic(elapsedTime);
                doSensorsTest();
                if (vuforiaVision != null)
                {
                    vuforiaVision.getVuMarkPosition();
                    vuforiaVision.getVuMarkOrientation();
                    if (robot.textToSpeech != null)
                    {
                        RelicRecoveryVuMark vuMark = vuforiaVision.getVuMark();
                        if (vuMark != prevVuMark)
                        {
                            String sentence = null;
                            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
                            {
                                sentence = String.format("%s is %s.", vuMark.toString(), "in view");
                            }
                            else if (prevVuMark != null)
                            {
                                sentence = String.format("%s is %s.", prevVuMark.toString(), "out of view");
                            }

                            if (sentence != null)
                            {
                                robot.textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
                                robot.dashboard.displayPrintf(11, sentence);
                            }
                        }
                        prevVuMark = vuMark;
                    }
                }
                break;

            case MOTORS_TEST:
                doMotorsTest();
                break;
        }
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        State state = sm.getState();
        dashboard.displayPrintf(8, "%s: %s", test.toString(), state != null? state.toString(): "STOPPED!");

        switch (test)
        {
            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                double lfEnc = robot.leftFrontWheel.getPosition();
                double rfEnc = robot.rightFrontWheel.getPosition();
                double lrEnc = robot.leftRearWheel.getPosition();
                double rrEnc = robot.rightRearWheel.getPosition();
                dashboard.displayPrintf(9, "Timed Drive: %.0f sec", time);
                dashboard.displayPrintf(10, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
                dashboard.displayPrintf(11, "Enc:lr=%.0f,rr=%.0f", lrEnc, rrEnc);
                dashboard.displayPrintf(12, "average=%f", (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
                dashboard.displayPrintf(13, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                        robot.driveBase.getXPosition(),
                                        robot.driveBase.getYPosition(),
                                        robot.driveBase.getHeading());
                timedDriveCommand.cmdPeriodic(elapsedTime);
                break;

            case X_DISTANCE_DRIVE:
            case Y_DISTANCE_DRIVE:
            case GYRO_TURN:
                dashboard.displayPrintf(9, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                        robot.getInput(robot.encoderXPidCtrl),
                                        robot.getInput(robot.encoderYPidCtrl),
                                        robot.getInput(robot.gyroPidCtrl));
                robot.encoderXPidCtrl.displayPidInfo(10);
                robot.encoderYPidCtrl.displayPidInfo(12);
                robot.gyroPidCtrl.displayPidInfo(14);

                if (!pidDriveCommand.cmdPeriodic(elapsedTime))
                {
                    if (test == Test.X_DISTANCE_DRIVE)
                    {
                        robot.encoderXPidCtrl.printPidInfo(robot.tracer);
                    }
                    else if (test == Test.Y_DISTANCE_DRIVE)
                    {
                        robot.encoderYPidCtrl.printPidInfo(robot.tracer);
                    }
                    else if (test == Test.GYRO_TURN)
                    {
                        robot.gyroPidCtrl.printPidInfo(robot.tracer);
                    }
                }
                break;
        }
    }   //runContinuous

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }   //isMenuBackButton

    private void doMenus()
    {
        //
        // Create menus.
        //
        FtcChoiceMenu<Test> testMenu = new FtcChoiceMenu<>("Tests:", null, this);
        FtcValueMenu driveTimeMenu = new FtcValueMenu("Drive time:", testMenu, this, 1.0, 10.0, 1.0, 4.0, " %.0f sec");
        FtcValueMenu driveDistanceMenu = new FtcValueMenu(
                "Drive distance:", testMenu, this, -10.0, 10.0, 0.5, 4.0, " %.1f ft");
        FtcValueMenu rangeDistanceMenu = new FtcValueMenu(
                "Range distance:", testMenu, this, 0.5, 12.0, 0.5, 6.0, " %.0f in");
        FtcValueMenu turnDegreesMenu = new FtcValueMenu(
                "Turn degrees:", testMenu, this, -360.0, 360.0, 5.0, 45.0, " %.0f deg");
        //
        // Populate menus.
        //
        testMenu.addChoice("Sensors test", Test.SENSORS_TEST, true);
        testMenu.addChoice("Motors test", Test.MOTORS_TEST, false);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("X Distance drive", Test.X_DISTANCE_DRIVE, false, driveDistanceMenu);
        testMenu.addChoice("Y Distance drive", Test.Y_DISTANCE_DRIVE, false, driveDistanceMenu);
        testMenu.addChoice("Degrees turn", Test.GYRO_TURN, false, turnDegreesMenu);
        testMenu.addChoice("Vision test", Test.VISION_TEST, false);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(testMenu, this);
        //
        // Fetch choices.
        //
        test = testMenu.getCurrentChoiceObject();
        driveTime = driveTimeMenu.getCurrentValue();
        driveDistance = driveDistanceMenu.getCurrentValue();
        turnDegrees = turnDegreesMenu.getCurrentValue();
        //
        // Show choices.
        //
        dashboard.displayPrintf(0, "Test: %s", testMenu.getCurrentChoiceText());
    }   //doMenus

    /**
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     */
    private void doSensorsTest()
    {
        final int LABEL_WIDTH = 100;
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        dashboard.displayPrintf(3, LABEL_WIDTH, "FrontEnc: ", "l=%.0f,r=%.0f",
                                robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition());
        dashboard.displayPrintf(4, LABEL_WIDTH, "RearEnc: ", "l=%.0f,r=%.0f",
                                robot.leftRearWheel.getPosition(), robot.rightRearWheel.getPosition());
        dashboard.displayPrintf(5, LABEL_WIDTH, "Gyro: ", "Rate=%.3f,Heading=%.1f",
                                robot.gyro.getZRotationRate().value,
                                robot.gyro.getZHeading().value);
    }   //doSensorsTest

    /**
     * This method runs each of the four wheels in sequence for a fixed number of seconds. It is for diagnosing
     * problems with the drive train. At the end of the run, you should check the amount of encoder counts each
     * wheel has accumulated. They should be about the same. If not, you need to check the problem wheel for
     * friction or chain tension etc. You can also use this test to check if a motor needs to be "inverted"
     * (i.e. turning in the wrong direction).
     */
    private void doMotorsTest()
    {
        dashboard.displayPrintf(9, "Motors Test: index=%d", motorIndex);
        dashboard.displayPrintf(10, "Enc: lf=%.0f, rf=%.0f",
                robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition());
        dashboard.displayPrintf(11, "Enc: lr=%.0f, rr=%.0f",
                robot.leftRearWheel.getPosition(), robot.rightRearWheel.getPosition());

        if (sm.isReady())
        {
            State state = sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Spin a wheel for 5 seconds.
                    //
                    switch (motorIndex)
                    {
                        case 0:
                            //
                            // Run the left front wheel.
                            //
                            robot.leftFrontWheel.setPower(0.5);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 1:
                            //
                            // Run the right front wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.5);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 2:
                            //
                            // Run the left rear wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.5);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 3:
                            //
                            // Run the right rear wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.5);
                            break;
                    }
                    motorIndex = motorIndex + 1;
                    timer.set(5.0, event);
                    sm.waitForSingleEvent(event, motorIndex < 4? State.START: State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, stop all wheels.
                    //
                    robot.leftFrontWheel.setPower(0.0);
                    robot.rightFrontWheel.setPower(0.0);
                    robot.leftRearWheel.setPower(0.0);
                    robot.rightRearWheel.setPower(0.0);
                    sm.stop();
                    break;
            }
        }
    }   //doMotorsTest

    //
    // Overrides FtcGamepad.ButtonHandler in FtcTeleOp.
    //

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed)
    {
        boolean processed = false;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
        // FtcTeleOp gamepad actions.
        //
        dashboard.displayPrintf(7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }
        }
        //
        // If the control was not processed by this method, pass it back to FtcTeleOp.
        //
        if (!processed)
        {
            super.gamepadButtonEvent(gamepad, button, pressed);
        }
    }   //gamepadButtonEvent

}   //class FtcTest
