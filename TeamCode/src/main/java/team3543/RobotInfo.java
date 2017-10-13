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

import com.qualcomm.robotcore.hardware.DcMotor;

class RobotInfo
{
    static final float MM_PER_INCH                      = 25.4f;
    //
    // DriveBase subsystem.
    //
    static final DcMotor.RunMode DRIVE_MOTOR_MODE       = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final double TURN_POWER_LIMIT                = 0.5;

    static final double ENCODER_X_KP                    = 0.2;
    static final double ENCODER_X_KI                    = 0.0;
    static final double ENCODER_X_KD                    = 0.0;
    static final double ENCODER_X_TOLERANCE             = 2.0;
    static final double ENCODER_X_INCHES_PER_COUNT      = 0.0132166817227156;

    static final double SMALL_X_THRESHOLD               = 8.0;
    static final double ENCODER_SMALL_X_KP              = 0.2;
    static final double ENCODER_SMALL_X_KI              = 0.0;
    static final double ENCODER_SMALL_X_KD              = 0.0;

    static final double ENCODER_Y_KP                    = 0.02;
    static final double ENCODER_Y_KI                    = 0.0;
    static final double ENCODER_Y_KD                    = 0.0022;
    static final double ENCODER_Y_TOLERANCE             = 2.0;
    static final double ENCODER_Y_INCHES_PER_COUNT      = 0.01667;

    static final double SMALL_Y_THRESHOLD               = 8.0;
    static final double ENCODER_SMALL_Y_KP              = 0.045;
    static final double ENCODER_SMALL_Y_KI              = 0.0;
    static final double ENCODER_SMALL_Y_KD              = 0.001;

    static final double ANALOG_GYRO_SCALE               = 1.0136;
    static final double ANALOG_GYRO_VOLT_PER_DEG_PER_SEC= 0.007;
    static final double GYRO_KP                         = 0.024;
    static final double GYRO_KI                         = 0.0;
    static final double GYRO_KD                         = 0.0024;
    static final double GYRO_TOLERANCE                  = 2.0;

    static final double SMALL_TURN_THRESHOLD            = 15.0;
    static final double GYRO_SMALL_TURN_KP              = 0.03;
    static final double GYRO_SMALL_TURN_KI              = 0.0;
    static final double GYRO_SMALL_TURN_KD              = 0.001;

    static final double PIDDRIVE_STALL_TIMEOUT          = 0.25;     //in msec.

    /**
     * @author TRC Programming Team, Elaine Z., Victor D.
     *
     * === VARIABLE VISION_KP CHANGELOG BELOW ===
     * Tune 1: 1.0
     * Tune 4: Useless stuff here
     * Tune 5: 0.0125 (Best of Thursday 10/5/2017, 20:53:16)
     * === END VARIABLE VISION_KP CHANGELOG.. ===
     */
    static final double VISION_KP                       = 0.0125;
    static final double VISION_KI                       = 0.0;
    static final double VISION_KD                       = 0.0;
    static final double VISION_TOLERANCE                = 1.0;

    //
    // GlyphGrabber subsystem.
    //
    static final double GLYPH_GRABBER_OPEN              = 0.2;
    static final double GLYPH_GRABBER_CLOSE             = 0.8;

    //
    // JewelBar subsystem.
    //
    static final double JEWEL_ARM_EXTENDED              = 0.8;
    static final double JEWEL_ARM_RETRACTED             = 0.2;

    //
    // GlyphElevator subsystem.
    //
    static final double ELEVATOR_INCHES_PER_COUNT       = 1.0;
    static final double ELEVATOR_KP                     = 0.5;
    static final double ELEVATOR_KI                     = 0.0;
    static final double ELEVATOR_KD                     = 0.0;
    static final double ELEVATOR_TOLERANCE              = 0.2;
    static final double ELEVATOR_MIN_HEIGHT             = 0.0;
    static final double ELEVATOR_MAX_HEIGHT             = 30.0;
    static final double ELEVATOR_CAL_POWER              = 0.3;

    //
    // RelicArm subsystem.
    //
    static final double EXTENDER_INCHES_PER_COUNT       = 1.0;
    static final double EXTENDER_KP                     = 0.5;
    static final double EXTENDER_KI                     = 0.0;
    static final double EXTENDER_KD                     = 0.0;
    static final double EXTENDER_TOLERANCE              = 0.2;
    static final double EXTENDER_MIN_POSITION           = 0.0;
    static final double EXTENDER_MAX_POSITION           = 30.0;
    static final double EXTENDER_CAL_POWER              = 0.3;

    static final double RELIC_GRABBER_OPEN              = 0.2;
    static final double RELIC_GRABBER_CLOSE             = 0.8;
    static final double RELIC_ELBOW_DOWN_POWER          = 0.3;
    static final double RELIC_ELBOW_UP_POWER            = 0.8;

}   //class RobotInfo
