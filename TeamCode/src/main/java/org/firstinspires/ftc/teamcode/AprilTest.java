package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.ftccommon.SoundPlayer;
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
import java.util.Optional;

@TeleOp(name = "AprilTest")
@Config
public class AprilTest extends LinearOpMode{
    @Override
    public void runOpMode() {
        lockOn();
    }


    AprilTagProcessor april;
    public Autodrive driver;
    final int SOUTH = 0;

    public void lockOn() {

        waitForStart();

        driver = new Autodrive(hardwareMap, this::opModeIsActive);

        april = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal vision = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), april);
        vision.resumeStreaming();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                target(1);
                driver.turn(0);
            }
            if (gamepad1.y) {
                target(2);
                driver.turn(0);
            }
            if (gamepad1.b){
                target(3);
                driver.turn(0);
            }
            if (gamepad1.dpad_up) {
                driver.drive(30, SOUTH);
            }
            if (gamepad1.dpad_down) {
                driver.drive(-30, SOUTH);
            }
        }
    }

    void target(int tagId) {

        int tries = 0;

        while (opModeIsActive() && tries < 20) {
            Optional<AprilTagDetection> someDetection = april.getDetections()
                    .stream()
                    .filter(d -> d.id == (tagId+3) && d.ftcPose != null)
                    .findFirst();

            double target = 0;
            if (someDetection.isPresent()) {
                AprilTagDetection d = someDetection.get();
                double error = target - d.ftcPose.x;
                if (Math.abs(error) < 1.0) {
                    driver.drive(5, SOUTH);
                    driver.drive(-5, SOUTH);
                    //playSound("ss_r2d2_up");

                    return;
                }
                driver.strafe(-error, SOUTH);
                sleep(100);
            } else {
                driver.strafe(5, SOUTH);
            }
            sleep(20);

            ++tries;
        }

        driver.drive(-5, SOUTH);
        //playSound("ss_wookie");
    }
}

