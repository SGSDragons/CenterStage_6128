package org.firstinspires.ftc.teamcode;

import junit.framework.TestCase;

import org.junit.Assert;
import org.junit.Test;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;


import java.io.File;

public class Spike_DetectorTest extends TestCase {
    static { System.loadLibrary("opencv_java480");}
    @Test
    public void testpic() {

        int spike = 1;
        String file = "red"+spike+".png";

        Mat rgb = Imgcodecs.imread(file);
        Mat bgr = new Mat();

        Imgproc.cvtColor(rgb, bgr, Imgproc.COLOR_RGB2BGR);

        SpikeDetector detector = new SpikeDetector();
        detector.processFrame(bgr, 1);

        Assert.assertEquals(spike, detector.getSpikecolumn());

    }

}