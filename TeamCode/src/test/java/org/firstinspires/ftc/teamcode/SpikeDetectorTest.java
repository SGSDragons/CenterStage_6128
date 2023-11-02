package org.firstinspires.ftc.teamcode;

import org.junit.Assert;
import org.junit.Test;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class SpikeDetectorTest {

    static { System.loadLibrary("opencv_java480"); }
    @Test
    public void processFrame() {
        // Images are read in RGB
        // [ 200, 128, 129, 205, 120, 125, ... ]
        int EXPECTATION = 2;
        Mat rgb = Imgcodecs.imread("spike" + EXPECTATION + ".png");
        Mat bgr = new Mat();
        Imgproc.cvtColor(rgb, bgr, Imgproc.COLOR_RGB2BGR);

        Assert.assertEquals(640, bgr.cols()); // Should be 640x480 pixels

        // Image is now how VisionProcessors expect them in BGR.
        // [ 129, 128, 200, 125, 120, 205, ... ]

        // These should be determined during the competition to account for
        // lighting differences. One set of values will be needed for each
        // prop.
        // These are for the test pictures taken on the blue prop.
        double hue = 10;
        double sat = 207;
        double lum = 112;

        Spike_Detector det = new Spike_Detector(hue, sat, lum);
        Mat binary = (Mat)det.processFrame(bgr, 0);

        Imgcodecs.imwrite("filtered50-"+EXPECTATION+".png", binary);
        Assert.assertEquals(EXPECTATION, det.getCorrectSpike());
    }
}
