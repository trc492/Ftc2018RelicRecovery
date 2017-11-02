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

import ftclib.FtcColorSensor;
import ftclib.FtcServo;
import trclib.TrcAnalogTrigger;

import static team3543.JewelArm.JewelColor.NONE;

public class JewelArm implements TrcAnalogTrigger.TriggerHandler
{
    public enum JewelColor
    {
        NONE,
        RED,
        BLUE
    }

    private static final boolean USE_JEWEL_COLOR_SENSOR = true;
    private static final double[] triggerPoints = {20.0, 120.0, 220.0, 350.0};

    private String instanceName;
    private Robot robot;
    private FtcServo verticalServo;
    private FtcServo horizontalServo;
    private FtcColorSensor jewelColorSensor = null;
    private TrcAnalogTrigger<FtcColorSensor.DataType> jewelColorTrigger = null;
    private JewelColor jewelColor = JewelColor.NONE;

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
        if (USE_JEWEL_COLOR_SENSOR)
        {
            jewelColorSensor = new FtcColorSensor("jewelColorRangeSensor");
            jewelColorTrigger = new TrcAnalogTrigger(
                    "jewelColorTrigger", jewelColorSensor, 0, FtcColorSensor.DataType.HUE,
                    triggerPoints, this);
            jewelColorTrigger.setEnabled(true);
        }
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

        if (jewelColorSensor != null)
        {
            color = jewelColor;
        }

        return color;
    }

    public double getJewelHsvHue()
    {
        double value = 0.0;

        if (jewelColorSensor != null)
        {
            value = jewelColorSensor.getRawData(0, FtcColorSensor.DataType.HUE).value;
        }

        return value;
    }

    public double getJewelHsvSaturation()
    {
        double value = 0.0;

        if (jewelColorSensor != null)
        {
            value = jewelColorSensor.getRawData(0, FtcColorSensor.DataType.SATURATION).value;
        }

        return value;
    }

    public double getJewelHsvValue()
    {
        double value = 0.0;

        if (jewelColorSensor != null)
        {
            value = jewelColorSensor.getRawData(0, FtcColorSensor.DataType.VALUE).value;
        }

        return value;
    }

    //
    // Implements TrcAnalogTrigger.TriggerHandler interface.
    //

    /**
     * This method is called when a threshold has been crossed.
     *
     * @param analogTrigger specifies the TrcAnalogTrigger object that detected the trigger.
     * @param zoneIndex specifies the zone index it is crossing into.
     * @param zoneValue specifies the actual sensor value.
     */
    @Override
    public void triggerEvent(TrcAnalogTrigger<?> analogTrigger, int zoneIndex, double zoneValue)
    {
        if (analogTrigger == jewelColorTrigger)
        {
            if (jewelColorSensor.getRawData(0, FtcColorSensor.DataType.VALUE).value < 10.0)
            {
                jewelColor = JewelColor.NONE;
                robot.textToSpeech.speak("No jewel found.", TextToSpeech.QUEUE_FLUSH, null);
            }
            else
            {
                switch (zoneIndex)
                {
                    case 0:
                    case 4:
                        //
                        // RED jewel found.
                        //
                        jewelColor = JewelColor.RED;
                        robot.textToSpeech.speak("Red jewel found.", TextToSpeech.QUEUE_FLUSH, null);
                        break;

                    case 1:
                    case 3:
                        //
                        // No jewel found.
                        //
                        jewelColor = JewelColor.NONE;
                        robot.textToSpeech.speak("No jewel found.", TextToSpeech.QUEUE_FLUSH, null);
                        break;

                    case 2:
                        //
                        // BLUE jewel found.
                        //
                        jewelColor = JewelColor.BLUE;
                        robot.textToSpeech.speak("Blue jewel found.", TextToSpeech.QUEUE_FLUSH, null);
                        break;
                }
            }
        }
    }   //triggerEvent

}   //class JewelArm
