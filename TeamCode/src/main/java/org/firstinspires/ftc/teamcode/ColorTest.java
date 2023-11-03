package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
@TeleOp(name = "ColorTest", group = "TEST")
public class ColorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        ColorProcessor colorprocesser = new ColorProcessor();

        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"),colorprocesser);
        visionPortal.resumeStreaming();

        waitForStart();


        while (opModeIsActive()){
            if (gamepad1.a) {
                colorprocesser.captureFrame = "frame_a.png";
            }
            if (gamepad1.b) {
                colorprocesser.captureFrame = "frame_b.png";
            }
            if (gamepad1.x) {
                colorprocesser.captureFrame = "frame_x.png";
            }
            if (gamepad1.y) {
                colorprocesser.captureFrame = "frame_y.png";
            }
            if (gamepad1.dpad_down) {
                colorprocesser.captureFrame = "frame_down.png";
            }
            if (gamepad1.dpad_up) {
                colorprocesser.captureFrame = "frame_up.png";
            }
            if (gamepad1.dpad_left) {
                colorprocesser.captureFrame = "frame_left.png";
            }
            if (gamepad1.dpad_right) {
                colorprocesser.captureFrame = "frame_right.png";
            }
        }
    }

    class ColorProcessor implements VisionProcessor{

        String captureFrame = null;
        @Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            if (captureFrame == null) {
                return null;
            }

            Mat rgb = new Mat();

            String file = Environment.getExternalStorageDirectory().getPath()+"/FIRST/data/" + captureFrame;


            Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGB2BGR);

            Imgcodecs.imwrite(file, rgb);

            captureFrame = null;
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            //Paint p = new Paint();
            //p.setColor(Color.rgb(255, 0, 0));
            //p.setStrokeWidth(5);

            //canvas.drawLine(0, 0, 100, 100, p);
        }
    }
}
