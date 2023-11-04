package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class SpikeDetector implements VisionProcessor {

    int column1 = 0;
    int column2 = 0;
    int column3 = 0;

    int r = 158;
    int g = 50;
    int b = 41;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    double[] getTargetHsv() {
        Mat sampledRgb = new Mat(1, 1, CvType.CV_8UC3);

        byte[] rgb = new byte[] { (byte)r, (byte)g, (byte)b };
        sampledRgb.put(0, 0, rgb);

        Mat sampledHsv = new Mat();
        Imgproc.cvtColor(sampledRgb, sampledHsv, Imgproc.COLOR_RGB2HSV);
        double[] hsv = sampledHsv.get(0,0);
        return hsv;

    }
    @Override
    public Object processFrame(Mat bgr, long captureTimeNanos) {

        Mat cropbgr = bgr.rowRange(100, 250);

        Mat hsv = new Mat();
        Mat blackwhite = new Mat();
        Mat box = new Mat();

        Imgproc.cvtColor(cropbgr, hsv, Imgproc.COLOR_BGR2HSV);

        double[] target = getTargetHsv();
        double h = 120;
        double s = 196;
        double v = 176;

        Scalar min = new Scalar(Math.max(0, h - 10), s - 50, v - 50);
        Scalar max = new Scalar(h + 10, 255, 255);

        Core.inRange(hsv, min, max, blackwhite);
        Imgproc.boxFilter(blackwhite, box, -1, new Size(31, 31));

        double xhigh = 0;
        int highcolumn = 0;

        for (int c = 0; c < box.cols(); c += 1) {
            for (int r = 0; r < box.rows(); r += 1) {
                if (box.get(r, c)[0] > xhigh) {
                    xhigh = box.get(r, c)[0];
                    highcolumn = c;
                }
            }
        }

        if (highcolumn <= box.cols() * 1 / 3) {
            column1 += 1;
        } else if (highcolumn <= box.cols() * 2 / 3) {
            column2 += 1;
        } else {
            column3 += 1;
        }

        return null;

    }

    public int getSpikecolumn() {
        int bestspikecolumn = 0;
        if (column1 > 3 && column1 > column2 && column1 > column3) {
            bestspikecolumn = 1;
        } else if (column2 > 3 && column2 > column1 && column2 > column3) {
            bestspikecolumn = 2;
        } else if (column3 > 3 && column3 > column1 && column3 > column2) {
            bestspikecolumn = 3;
        }
        return bestspikecolumn;
    }

    void reset() {
        column1 = 0;
        column2 = 0;
        column3 = 0;
    }

    @Override
    public void onDrawFrame (Canvas canvas,int onscreenWidth, int onscreenHeight,
    float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){

    }
}