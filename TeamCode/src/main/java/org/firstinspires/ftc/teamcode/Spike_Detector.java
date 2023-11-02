package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class Spike_Detector implements VisionProcessor {
    private final double hue;
    private final double saturation;
    private final double luminosity;

    private  int correctSpike = 0;

    public Spike_Detector(double hue, double saturation, double luminosity) {
        this.hue = hue;
        this.saturation = saturation;
        this.luminosity = luminosity;
    }

    public int getCorrectSpike() {
        return correctSpike;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Do initialization if needed. Swap cameras, multiple cameras....?
    }

    @Override
    public Object processFrame(Mat bgrimage, long captureTimeNanos) {
        if (correctSpike > 0) return null; // A selection has already been made

        Mat hsvimage = new Mat();

        // Convert frame from BGR to HSV to improve color finding.
        Imgproc.cvtColor(bgrimage, hsvimage, Imgproc.COLOR_BGR2HSV);

        // Create a binary image based on a color range filter. A one is
        // a color match. A zero is a mismatch.
        // Hue is color so use tight bounds on it.
        // saturation and luminosity are lowered a bit to be more tolerant
        // of poor lighting. There's no upper bound for them because the
        // higher the better.
        Scalar lower = new Scalar(hue-5, saturation*3/4, luminosity*3/4);
        Scalar upper = new Scalar(hue+5, 255, 255);
        Mat binaryMat = new Mat();
        Core.inRange(hsvimage, lower, upper, binaryMat);

        // For every pixel in the binary image, change it's value to the
        // of the 50x50 pixels in its neighborhood. A single matching
        // color surrounded by mismatches will get a score of 1/2500.
        // If there's a large patch of matching color, its middle pixels
        // will have scores close to 1. The pixel with the highest score
        // will be the pixel with the most matching colors in its 50x50
        // neighborhood.
        Mat filtered = new Mat();
        Imgproc.boxFilter(binaryMat, filtered, -1, new Size(50, 50));

        // Find the X coordinate for the best pixel. If it's in the
        // left most quarter of the image, then the beacon is probably
        // on spike 1. If it's in the second or third quarters, then
        // assume spike 2. Otherwise, it must be spike 3.
        int bestX = 0;
        double bestPixel = 0.0;
        for (int i=0; i < filtered.cols(); ++i) {
            for (int j=0; j < filtered.rows(); ++j) {
                double p = filtered.get(j, i)[0];
                if (p > bestPixel) {
                    bestPixel = p;
                    bestX = i;
                }
            }
        }

        if (bestX < filtered.rows() / 4) {
            correctSpike = 1;
        } else if (bestX < filtered.rows() * 3 / 4) {
            correctSpike = 2;
        } else {
            correctSpike = 3;
        }

//        // Save the filtered image for post-game analysis.
//        // This way if there's an issue, we can see why.
//        String originalPath = Environment.getExternalStorageDirectory().getPath() + "/FIRST/data/spike-original.png";
//        String filteredPath = Environment.getExternalStorageDirectory().getPath() + "/FIRST/data/spike-selection.png";
//
//        Mat rgb = new Mat();
//        Imgproc.cvtColor(bgrimage, rgb, Imgproc.COLOR_BGR2RGB);
//        Imgcodecs.imwrite(originalPath, rgb);
//        Imgcodecs.imwrite(filteredPath, filtered);
        return filtered;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}