package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Optional;

@Autonomous(name = "backstage_blue")
@Config
public class backstage_blue extends LinearOpMode {


    final int SOUTH = 90;
    final int NORTH = -90;

    public Autodrive driver;
    DcMotor intake;
    DcMotor conveyor;

    AprilTagProcessor april;

    @Config
    public static class BLUE_S1 {

        public static int first_drive = 40;
        public static int second_drive = 35;
    }

    @Config
    public static class BLUE_S2 {
        public static int first_drive = 63;
        public static int second_drive = 10;
    }

    public static int BACKDROP_STRAFE = -25;
    public static int BACKDROP_DRIVE = 32;

    int targetSpike = 0;

    @Override
    public void runOpMode() {
        driveSpike();
        driveBackdrop();
    }

    public void driveSpike() {

        waitForStart();

        driver = new Autodrive(hardwareMap, this::opModeIsActive);
        intake = hardwareMap.get(DcMotor.class, "intake");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");

        SpikeDetector spike = new SpikeDetector(true);
        april = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal vision = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), spike, april);
        vision.setProcessorEnabled(april, false);
        vision.resumeStreaming();

        targetSpike = spike.getSpikecolumn();
        while (targetSpike == 0) {
            targetSpike = spike.getSpikecolumn();
        }

        TelemetryPacket tp = new TelemetryPacket();
        tp.put("spike", targetSpike);
        FtcDashboard.getInstance().sendTelemetryPacket(tp);

        vision.setProcessorEnabled(spike, false);
        vision.setProcessorEnabled(april, true);

        if (targetSpike == 1) {
            spike1();
        }
        // For middle spike autonomous instructions.
        if (targetSpike == 2) {
            spike2();
        }
        // For right spike autonomous instructions.
        if (targetSpike == 3) {
            spike3();
        }

        driver.turn(SOUTH);
    }

    void spike1() {
        driver.drive(BLUE_S1.first_drive, 0); //drives to middle section
        driver.turn(NORTH); //turns to left spike
        driver.drive(-9, NORTH); //backs closer to spike (maybe not needed)
        intake.setPower(-1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(3 , NORTH); //goes back to previous area
        driver.turn(0); //turns to the middle of field
        driver.drive(BLUE_S1.second_drive, 0); //drives up to the raising area
        driver.turn(SOUTH); //drives forward, closer to backdrop
        driver.drive(32,SOUTH);
    }

    void spike2() {
        driver.drive(BLUE_S2.first_drive, 10); //driving through the middle spike
        intake.setPower(-1);//spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(BLUE_S2.second_drive, 0); //drive out of the way of the bar
        driver.turn(SOUTH); //drives forward, closer to backdrop
        driver.drive(32,SOUTH);

    }

    void spike3() {
        driver.drive(BLUE_S1.first_drive, 0); //drives to middle section
        driver.turn(SOUTH); //turns to right spike
        driver.drive(-6, SOUTH); //backs closer to spike (maybe not needed)
        intake.setPower(-1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(12, SOUTH); //goes back to previous area
        driver.turn(0); //turns to the middle of field
        driver.drive(BLUE_S1.second_drive, 0); //drives up to the raising area
        driver.turn(SOUTH); //drives forward, closer to backdrop
        driver.drive(30,SOUTH);

    }

    void driveBackdrop() {
        driver.strafe(BACKDROP_STRAFE, SOUTH);
        alignWithTag(targetSpike);
        driver.drive(BACKDROP_DRIVE, SOUTH);
        conveyor.setPower(0.5);
        sleep(3000);
        conveyor.setPower(0);
        driver.strafe(35, SOUTH);
    }

    void alignWithTag(int tagId) {

        int tries = 0;

        while (opModeIsActive() && tries < 20) {
            Optional<AprilTagDetection> someDetection = april.getDetections()
                    .stream()
                    .filter(d -> d.id == tagId && d.ftcPose != null)
                    .findFirst();

            if (someDetection.isPresent()) {
                AprilTagDetection d = someDetection.get();
                double xshift = d.ftcPose.x;
                if (Math.abs(xshift) < 1) return;
                driver.strafe(xshift, SOUTH);
            } else {
                driver.strafe(-5, SOUTH);
            }
            sleep(200);

            ++tries;
        }

    }
}
