package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Rect;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class SpikeDetector implements VisionProcessor {

    int column1 = 0;
    int column2 = 0;
    int column3 = 0;


    public static double hue;
    public static double saturation;
    public static double value;

    public static int cropH = 150;
    public static int cropL = 270;
    final double rowCount = 480;

    public SpikeDetector(boolean isBlue) {
        if (isBlue) {
            // Fill in numbers for blue
            hue = 18;
            saturation = 180;
            value = 120;

            //hue = hue
            //saturation = saturation
            //volume = volume
        } else {
            hue = 120;
            saturation = 196;
            value = 176;
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }


    @Override
    public Object processFrame(Mat bgr, long captureTimeNanos) {

        Mat cropbgr = bgr.rowRange(cropH, cropL);

        Mat hsv = new Mat();
        Mat blackwhite = new Mat();
        Mat box = new Mat();

        Imgproc.cvtColor(cropbgr, hsv, Imgproc.COLOR_BGR2HSV);


        Scalar min = new Scalar(Math.max(0, hue - 10), saturation - 50, value - 50);
        Scalar max = new Scalar(hue + 10, 255, 255);

        Core.inRange(hsv, min, max, blackwhite); //turns the picture to black or white
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

        return blackwhite;

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
    public void onDrawFrame (
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext)  // userContext is the return value from processFrame
    {

        Mat blackwhite = (Mat)userContext;
        if (blackwhite.empty()) {
            return;
        }

        Bitmap bitmap = Bitmap.createBitmap(blackwhite.cols(), blackwhite.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(blackwhite, bitmap);

        //all of this makes the streak in the middle not stretch a lot to make the thing a little bit cleaner

        double canvasTop = cropH/rowCount * onscreenHeight;
        double canvasBottom = cropL/rowCount * onscreenHeight;

        Rect src = new Rect(0, 0, blackwhite.cols(), blackwhite.rows());
        Rect dst = new Rect(0, (int)canvasTop,onscreenWidth,(int)canvasBottom);
        canvas.drawBitmap(bitmap, src, dst, null);
    }
}