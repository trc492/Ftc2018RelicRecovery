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

public class JewelArm implements TrcAnalogTrigger.TriggerHandler
{
    public enum JewelColor
    {
        NO,
        RED,
        BLUE
    }

    private static final boolean USE_JEWEL_COLOR_SENSOR = true;
    private static final double[] triggerPoints = {20.0, 120.0, 220.0, 350.0};

    private String instanceName;
    private TextToSpeech textToSpeech = null;
    private FtcServo verticalServo;
    private FtcServo horizontalServo;
    private FtcColorSensor jewelColorSensor = null;
    private TrcAnalogTrigger<FtcColorSensor.DataType> jewelColorTrigger = null;
    private JewelColor jewelColor = JewelColor.NO;
    private boolean armExtended = false;

    /**
     * Constructor: Create an instance of the object  .
     */
    public JewelArm(String instanceName, Robot robot, TextToSpeech textToSpeech)
    {
        this.instanceName = instanceName;
        this.textToSpeech = textToSpeech;
        verticalServo = new FtcServo(instanceName + "VerticalServo");
        horizontalServo = new FtcServo(instanceName + "HorizontalServo");
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
        armExtended = extended;
        verticalServo.setPosition(extended? RobotInfo.JEWEL_ARM_EXTENDED: RobotInfo.JEWEL_ARM_RETRACTED);
    }

    public void setSweepPosition(double pos)
    {
        horizontalServo.setPosition(pos);
    }

    public JewelColor getJewelColor()
    {
        JewelColor color = JewelColor.NO;

        if (jewelColorSensor != null)
        {
            double hue = jewelColorSensor.getRawData(0, FtcColorSensor.DataType.HUE).value;
            if (hue <= 20.0 || hue >= 350.0)
                color = JewelColor.RED;
            else if (hue >= 120.0 && hue <= 220.0)
                color = JewelColor.BLUE;
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
            if (armExtended)
            {
                switch (zoneIndex)
                {
                    case 0:
                    case 4:
                        //
                        // RED jewel found.
                        //
                        jewelColor = JewelColor.RED;
                        break;

                    case 1:
                    case 3:
                        //
                        // No jewel found.
                        //
                        jewelColor = JewelColor.NO;
                        break;

                    case 2:
                        //
                        // BLUE jewel found.
                        //
                        jewelColor = JewelColor.BLUE;
                        break;
                }

                if (textToSpeech != null)
                {
                    textToSpeech.speak(
                            String.format("%s jewel found.", jewelColor.toString()),
                            TextToSpeech.QUEUE_FLUSH, null);
                }
            }
            else
            {
                jewelColor = JewelColor.NO;
            }
        }
    }   //triggerEvent

}   //class JewelArm
