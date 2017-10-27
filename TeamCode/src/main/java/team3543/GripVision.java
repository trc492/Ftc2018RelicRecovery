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

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import ftclib.FtcOpMode;
import hallib.HalDashboard;
import hallib.HalVideoSource;
import trclib.TrcDbgTrace;
import trclib.TrcOpenCvDetector;

public class GripVision extends TrcOpenCvDetector<Rect[]>
{
    private static final String moduleName = "GripVision";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;
    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private static final int NUM_IMAGE_BUFFERS = 2;

    private static final double[] RED_JEWEL_HUE = {0.0, 100.0};
    private static final double[] BLUE_JEWEL_HUE = {40.0, 160.0};

    private GripPipelineJewel gripRedJewel = null;
    private GripPipelineJewel gripBlueJewel = null;
    private volatile Rect[] jewelRects = null;
    private volatile Mat currImage = null;
    private boolean videoOutEnabled = false;

    public GripVision(final String instanceName, HalVideoSource videoSource)
    {
        super(instanceName, videoSource, NUM_IMAGE_BUFFERS, null);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        gripRedJewel = new GripPipelineJewel("RedJewel", RED_JEWEL_HUE);
        gripBlueJewel = new GripPipelineJewel("BlueJewel", BLUE_JEWEL_HUE);
        jewelRects = new Rect[2];
    }   //GripVision

    /**
     * This method update the video stream with the detected targets overlay on the image as rectangles.
     *
     * @param color specifies the color of the rectangle outline overlay onto the detected targets.
     * @param thickness specifies the thickness of the rectangle outline.
     */
    public void putFrame(Scalar color, int thickness)
    {
        if (currImage != null)
        {
//            drawRectangles(currImage, objectRects, color, thickness);
        }
    }   //putFrame

    /**
     * This method update the video stream with the detected targets overlay on the image as rectangles.
     */
    public void putFrame()
    {
        if (currImage != null)
        {
//            drawRectangles(currImage, objectRects, new Scalar(0, 255, 0), 0);
        }
    }   //putFrame

    /**
     * This method enables/disables the video out stream.
     *
     * @param enabled specifies true to enable video out stream, false to disable.
     */
    public void setVideoOutEnabled(boolean enabled)
    {
        final String funcName = "setVideoOutEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        videoOutEnabled = enabled;
    }   //setVideoOutEnabled

//    public void setEnabled(boolean eanabled)
//    {
//
//    }

    public void getJewelRects(Rect[] rects)
    {
        synchronized (this.jewelRects)
        {
            rects[0] = jewelRects[0];
            jewelRects[0] = null;
            rects[1] = jewelRects[1];
            jewelRects[1] = null;
        }
    }   //getJewelRects

    private Rect getTargetRect(GripPipelineJewel pipeline, Mat image)
    {
        Rect targetRect = null;
        MatOfKeyPoint detectedTargets;

        pipeline.process(image);
        detectedTargets = pipeline.findBlobsOutput();
        if (detectedTargets != null)
        {
            KeyPoint[] targets = detectedTargets.toArray();
            if (targets.length > 1)
            {
                HalDashboard.getInstance().displayPrintf(15, "%s: %s", pipeline, targets[0]);
                double radius = targets[0].size/2;
                targetRect = new Rect(
                        (int)(targets[0].pt.x - radius), (int)(targets[0].pt.y - radius),
                        (int)targets[0].size, (int)targets[0].size);
            }
            detectedTargets.release();
        }

        return targetRect;
    }   //getTargetRect

    /**
     * This method is called to detect objects in the acquired image frame.
     *
     * @param image specifies the image to be processed.
     * @param buffers specifies the preallocated buffer to hold the detected targets (not used since no
     *        preallocated buffer required).
     * @return detected objects, null if none detected.
     */
//    @Override
    public Rect[] detectObjects(Mat image, Rect[] buffers)
    {
        //
        // Process the image to detect the jewels we are looking for and put them into jewelRects.
        //
        gripRedJewel.process(image);
        gripBlueJewel.process(image);
        synchronized (jewelRects)
        {
            jewelRects[0] = getTargetRect(gripRedJewel, image);
            jewelRects[1] = getTargetRect(gripBlueJewel, image);
        }

        if (videoOutEnabled)
        {
//            putFrame();
        }

        return jewelRects;
    }   //detectObjects

}   //class GripVision
