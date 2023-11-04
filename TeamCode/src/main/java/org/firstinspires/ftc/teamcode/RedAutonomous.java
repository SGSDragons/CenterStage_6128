package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Optional;

@Config
abstract class RedAutonomous extends LinearOpMode {
    final int SOUTH = -90;
    final int NORTH = 90;

    public Autodrive driver;
    DcMotor intake;
    DcMotor conveyor;

    AprilTagProcessor april;

    @Config
    public static class RED_S1 {

        public static int first_drive = 40;
        public static int second_drive = 35;
    }

    @Config
    public static class RED_S2 {
        public static int first_drive = 65;
        public static int second_drive = 10;
    }

    public static int BACKDROP_STRAFE = 40;
    public static int BACKDROP_DRIVE = 30;

    public int driveSpike() {

        waitForStart();

        driver = new Autodrive(hardwareMap, this::opModeIsActive);
        intake = hardwareMap.get(DcMotor.class,"intake");
        conveyor = hardwareMap.get(DcMotor.class,"conveyor");

        SpikeDetector spike = new SpikeDetector();
        april = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal vision = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class,"webcam"), spike, april);
        vision.setProcessorEnabled(april, false);
        vision.resumeStreaming();

        int result = spike.getSpikecolumn();
        while (result == 0) {
            result = spike.getSpikecolumn();
        }

        TelemetryPacket tp = new TelemetryPacket();
        tp.put("spike", result);
        FtcDashboard.getInstance().sendTelemetryPacket(tp);

        vision.setProcessorEnabled(spike, false);
        vision.setProcessorEnabled(april, true);

        if (result == 1) {
            spike1();
        }
        // For middle spike autonomous instructions.
        if (result == 2){
            spike2();
        }
        // For right spike autonomous instructions.
        if (result == 3) {
            spike3();
        }

        driver.turn(SOUTH);

        return result;
    }

    void spike1() {
        driver.drive(RED_S1.first_drive, 0); //drives to middle section
        driver.turn(SOUTH); //turns to left spike
        driver.drive(-8, SOUTH); //backs closer to spike (maybe not needed)
        intake.setPower(-1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(5, SOUTH); //goes back to previous area
        driver.turn(0); //turns to the middle of field
        driver.drive(RED_S1.second_drive, 0); //drives up to the raising area
        driver.turn(-90);
    }

    void spike2() {
        driver.drive(RED_S2.first_drive, 0); //driving through the middle spike
        intake.setPower(-1);//spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(RED_S2.second_drive, 0); //drive out of the way of the bar
        driver.turn(-90);
    }

    void spike3() {
        driver.drive(RED_S1.first_drive, 0); //drives to middle section
        driver.turn(NORTH); //turns to right spike
        driver.drive(-8, NORTH); //backs closer to spike (maybe not needed)
        intake.setPower(-1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(10, NORTH); //goes back to previous area
        driver.turn(0); //turns to the middle of field
        driver.drive(RED_S1.second_drive, 0); //drives up to the raising area
    }

    void driveBackdrop(int tagId) {
        driver.strafe(BACKDROP_STRAFE, SOUTH);
        alignWithTag(tagId);
        driver.drive(BACKDROP_DRIVE, SOUTH);
        conveyor.setPower(0.7);
        sleep(2000);
        conveyor.setPower(0);
    }

    void alignWithTag(int tagId) {
        while (opModeIsActive()) {
            Optional<AprilTagDetection> someDetection = april.getDetections()
                    .stream()
                    .filter(d -> d.id == tagId && d.ftcPose != null)
                    .findFirst();

            if (someDetection.isPresent()) {
                AprilTagDetection d = someDetection.get();
                double xshift = d.ftcPose.x;
                if (Math.abs(xshift) < 0.5) return;
                driver.strafe(xshift, SOUTH);
            }

            sleep(20);
        }
    }
}
