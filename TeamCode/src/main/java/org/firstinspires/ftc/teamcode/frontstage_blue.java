package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

//@Autonomous(name = "frontstage_blue")
@TeleOp(name = "frontstage_blue")
@Config

public class frontstage_blue extends LinearOpMode {

    public static int LEG_ONE = 50;
    public static int LEG_TWO = 3;
    public static int LEG_THREE = 87;
    public static int LEG_FOUR = 0;
    public static int LEG_FIVE = 0;
    public static int strafe_s3 = 28;

    @Config
    public static class S1 {
        public static int first_drive = 40;
        public static int second_drive = 35;
        public static int third_drive = 100;
        public static int strafe_s1 = 29;
        public static int fourth_drive = 5;
    }
    @Config
    public static class S2 {
        public static int first_drive = 50;
        public static int second_drive = 10;
        public static int third_drive = 100;
        public static int strafe_s2 = 32;
        public static int fourth_drive = 5;
    }
    DcMotor intake;
    DcMotor conveyor;


    @Override
    public void runOpMode() {

        waitForStart();

        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);
        intake = hardwareMap.get(DcMotor.class,"intake");
        conveyor = hardwareMap.get(DcMotor.class,"conveyor");

        //boolean isBlue = true;

        //Spike_Det detector = new Spike_Det(isBlue);

        //VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(
                //hardwareMap.get(WebcamName.class, "webcam"),
                //detector);

            //visionPortal.resumeStreaming();
            //int correctSpike = 0;

            //while(correctSpike == 0) {
            //correctSpike = detector.getCorrectSpike();
            //}

            //visionPortal.stopStreaming();

            //sleep(2000);

        // For left spike autonomous instructions. For strafe, positive values make it go right
        //if (correctSpike == 1)
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
            driver.turn(90);
        }
            if (gamepad1.dpad_up) {
                driver.turn(0);
            }
            if (gamepad1.dpad_right) {
                driver.turn(-90);
            }
            if (gamepad1.dpad_down) {
                driver.turn(180);
            }
            if (gamepad1.x) {
                spike1(driver, -90, -3);
            }

            // For middle spike autonomous instructions.
            //if (correctSpike == 2)
            if (gamepad1.y) {
                spike2(driver);
            }
            // For right spike autonomous instructions.
            //if (correctSpike == 3)
            if (gamepad1.b) {
                spike3(driver, 90, 3);
            }
        }
        // Now we continue on with the autonomous driving instructions to get the robot
        // through the middle racks, then over to the correct backdrop to read April Tags.

        //if (correctSpike == 1) {
            //driver.turn(90);
            //driver.drive(18);
       //}

    }
    private void spike1(Autodrive driver, int degrees, int degrees1) {
        driver.drive(S1.first_drive); //drives to middle section
        driver.turn(degrees); //turns to left spike
        driver.drive(-6); //backs closer to spike (maybe not needed)
        intake.setPower(1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(6); //goes back to previous area
        driver.turn(degrees1); //turns to the middle of field
        driver.drive(S1.second_drive); //drives up to the raising area
        driver.turn(90);
        driver.drive(S1.third_drive);
        driver.strafe(S1.strafe_s1);
        driver.drive(S1.fourth_drive);
        conveyor.setPower(.2);
        sleep(2000);
        conveyor.setPower(0);
       //stop - already in backstage now

    }
    private void spike2(Autodrive driver) {
        driver.drive(S2.first_drive); //driving through the middle spike
        intake.setPower(1);//spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(S2.second_drive); //drive out of the way of the bar
        driver.turn(90);
        driver.drive(S2.third_drive);
        driver.strafe(S2.strafe_s2);
        driver.drive(S2.fourth_drive);
        conveyor.setPower(.2);
        sleep(2000);
        conveyor.setPower(0);
        //stop - already in backstage now

    }

    private void spike3(Autodrive driver, int degrees, int degrees1) {
        driver.drive(S1.first_drive); //drives to middle section
        driver.turn(degrees); //turns to right spike
        driver.drive(-6); //backs closer to spike (maybe not needed)
        intake.setPower(1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(5); //goes back to previous area
        driver.turn(degrees1); //turns to the middle of field
        driver.drive(S1.second_drive); //drives up to the raising area
        driver.turn(90);
        driver.drive(S1.third_drive);
        driver.strafe(strafe_s3);
        driver.drive(S1.fourth_drive);
        conveyor.setPower(.2);
        sleep(2000);
        conveyor.setPower(0);
        //stop - already in backstage now

    }

}
