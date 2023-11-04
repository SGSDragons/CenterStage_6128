package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Optional;

@Autonomous(name = "backstage_red")
//@TeleOp(name = "backstage_red")
@Config

public class backstage_red extends LinearOpMode {

    final int SOUTH = -90;
    final int NORTH = 90;

    @Config
    public static class BR_S1 {

        public static int first_drive = 40;
        public static int second_drive = 35;
        public static int third_drive = 30;
        public static int strafe_s1 = 22;
        public static int fourth_drive = 15;
    }
    @Config
    public static class BR_S2 {
        public static int first_drive = 65;
        public static int second_drive = 10;
        public static int third_drive = 30;
        public static int strafe_s2 = 40;
        public static int fourth_drive = 30;
    }

    DcMotor intake;
    DcMotor conveyor;


    @Override
    public void runOpMode() {

        waitForStart();

        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);
        intake = hardwareMap.get(DcMotor.class,"intake");
        conveyor = hardwareMap.get(DcMotor.class,"conveyor");

        SpikeDetector spike = new SpikeDetector();
        AprilTagProcessor april = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal vision = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class,"webcam"), spike, april);
        vision.setProcessorEnabled(april, false);
        vision.resumeStreaming();

        int result = spike.getSpikecolumn();
        while (result == 0)
            result = spike.getSpikecolumn();

        vision.setProcessorEnabled(spike, false);
        vision.setProcessorEnabled(april, true);
        final int tagId = result + 3;

        TelemetryPacket tp = new TelemetryPacket();
        tp.put("spike", result);
        FtcDashboard.getInstance().sendTelemetryPacket(tp);


        if (result == 1) {
            //if (gamepad1.x) {
            spike1(driver);
        }
        // For middle spike autonomous instructions.
        if (result == 2){
        //if (gamepad1.y) {
            spike2(driver);
        }
        // For right spike autonomous instructions.
        if (result == 3) {
            //if (gamepad1.b) {
            spike3(driver);
        }

        driver.strafe(BR_S2.strafe_s2, SOUTH);
        alignWithTag(driver, april, tagId);
        driver.drive(BR_S2.fourth_drive, SOUTH);
        conveyor.setPower(0.7);
        sleep(2000);
        conveyor.setPower(0);

    }

    private void alignWithTag(Autodrive driver, AprilTagProcessor april, int tagId) {
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

    private void spike1(Autodrive driver) {
        driver.drive(BR_S1.first_drive, 0); //drives to middle section
        driver.turn(SOUTH); //turns to left spike
        driver.drive(-8, SOUTH); //backs closer to spike (maybe not needed)
        intake.setPower(-1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(5, SOUTH); //goes back to previous area
        driver.turn(0); //turns to the middle of field
        driver.drive(BR_S1.second_drive, 0); //drives up to the raising area
        driver.turn(-90);
        driver.drive(BR_S1.third_drive, SOUTH);
//        driver.strafe(BR_S1.strafe_s1, -90);
//        driver.drive(BR_S1.fourth_drive, -90);
//        conveyor.setPower(0.7);
//        sleep(2000);
//        conveyor.setPower(0);
        //stop - already in backstage now

    }
    private void spike2(Autodrive driver) {
        driver.drive(BR_S2.first_drive,0); //driving through the middle spike
        intake.setPower(-1);//spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(BR_S2.second_drive, 0); //drive out of the way of the bar
        driver.turn(-90);
        driver.drive(BR_S2.third_drive, SOUTH);
//        driver.strafe(BR_S2.strafe_s2, -90);
//        driver.drive(BR_S2.fourth_drive, -90);
//        conveyor.setPower(0.7);
//        sleep(2000);
//        conveyor.setPower(0);
        //stop - already in backstage now

    }

    private void spike3(Autodrive driver) {
        driver.drive(BR_S1.first_drive, 0); //drives to middle section
        driver.turn(NORTH); //turns to right spike
        driver.drive(-8, NORTH); //backs closer to spike (maybe not needed)
        intake.setPower(-1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(10, NORTH); //goes back to previous area
        driver.turn(0); //turns to the middle of field
        driver.drive(BR_S1.second_drive, 0); //drives up to the raising area
        driver.turn(SOUTH);
        driver.drive(BR_S1.third_drive, SOUTH);
//        driver.strafe(strafe_s3, -90);
//        driver.drive(BR_S1.fourth_drive, -90);
//        conveyor.setPower(0.7);
//        sleep(2000);
//        conveyor.setPower(0);
        //stop - already in backstage now

    }

}
