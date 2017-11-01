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

import android.graphics.Color;

import ftclib.FtcColorSensor;
import ftclib.FtcServo;

public class JewelArm
{
    public enum JewelColor
    {
        NONE,
        RED,
        BLUE
    }

    private String instanceName;
    private Robot robot;
    private FtcServo verticalServo;
    private FtcServo horizontalServo;

    /**
     * Constructor: Create an instance of the object  .
     */
    public JewelArm(String instanceName, Robot robot)
    {
        this.instanceName = instanceName;
        this.robot = robot;
        verticalServo = new FtcServo(instanceName + "VerticalServo");
        // verticalServo.setInverted(true);
        horizontalServo = new FtcServo(instanceName + "HorizontalServo");
        // horizontalServo.setInverted(true);
    }   //JewelArm

    public String toString()
    {
        return instanceName;
    }

    public void setExtended(boolean extended)
    {
        verticalServo.setPosition(extended? RobotInfo.JEWEL_ARM_EXTENDED: RobotInfo.JEWEL_ARM_RETRACTED);
    }

    public void setSweepPosition(double pos)
    {
        horizontalServo.setPosition(pos);
    }

    public JewelColor getJewelColor()
    {
        JewelColor color = JewelColor.NONE;

        if (robot.jewelColorSensor != null)
        {
            float hsvValues[] = {0.0f, 0.0f, 0.0f};
            int red = robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.RED).value;
            int green = robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.GREEN).value;
            int blue = robot.jewelColorSensor.getRawData(0, FtcColorSensor.DataType.BLUE).value;
            Color.RGBToHSV(red, green, blue, hsvValues);

            color = hsvValues[0] >= 120.0f && hsvValues[0] <= 220.0f ? JewelColor.BLUE :
                    hsvValues[0] >= 0.0f && hsvValues[0] <= 30.0 || hsvValues[0] >= 350.0 ? JewelColor.RED :
                           JewelColor.NONE;
        }

        return color;
    }

}   //class JewelArm
