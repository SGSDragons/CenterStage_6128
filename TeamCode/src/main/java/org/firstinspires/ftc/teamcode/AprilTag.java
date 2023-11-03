package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name=   "TagStrafe", group="TEST")
@Config
public class AprilTag extends LinearOpMode {

    public static double gain = 0.0;
    @Override
    public void runOpMode() {

        AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal vision = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class,"webcam"), aprilTag);
        vision.resumeStreaming();
        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);
        driver.imu.resetYaw();

        waitForStart();
        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection d : detections) {
                if (d.id == 1) {
                    TelemetryPacket p = new TelemetryPacket();
                    if (d.ftcPose == null) {
                        p.put("pose", -50);
                    } else {
                        double xshift = d.ftcPose.x;
                        driver.strafe(xshift * gain, 0);
                        p.put("pose", xshift);
                    }
                    FtcDashboard.getInstance().sendTelemetryPacket(p);
                }

            }

            sleep(20);
        }
    }
}

