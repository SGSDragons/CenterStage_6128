package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import android.util.Log;

import java.io.File;

@TeleOp(name = "Test")
public class Spike_DetectorTest extends TestCase {
    static { System.loadLibrary("opencv_java480");}
    @Test
    public void testpic() {

        int num = 1;
        String color = "red";
        String file = color+num+".png";

        double h = 0;
        double s = 0;
        double v = 0;

        if (color == "blue"){
            h = 19;
            s = 200;
            v = 150;
        }else if (color == "red"){
            h = 120;
            s = 196;
            v = 176;
        }

        Mat rgb = Imgcodecs.imread(file);
        Mat hsv = new Mat();
        Mat blackwhite = new Mat();
        Mat box = new Mat();


        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar min = new Scalar(Math.max(0, h - 10), s - 50, v - 50);
        Scalar max = new Scalar(h + 10, 255, 255);

        Core.inRange(hsv, min, max, blackwhite);
        Imgproc.boxFilter(blackwhite, box, -1, new Size(1, 1));
        Imgcodecs.imwrite("new.png", blackwhite);

        double xhigh = 0;
        int highcolumn = 0;
        int pixelcount = 0;

        for (int c = 0; c < box.cols(); c += 1) {
            for (int r = 0; r < box.rows(); r += 1) {
                if (box.get(r, c)[0] > xhigh) {
                    xhigh = box.get(r, c)[0];
                    highcolumn = c;
                }if (box.get(r, c)[0] > 0) {
                    pixelcount += 1;
                }
            }
        }

        if (highcolumn <= box.cols() * 1/3) {
            highcolumn = 1;
        } else if (highcolumn <= box.cols() * 2/3) {
            highcolumn = 2;
        } else {
            highcolumn = 3;
        }

        Assert.assertEquals(num,highcolumn);
    }

}