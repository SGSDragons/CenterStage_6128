package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

//@Autonomous(name = "frontstage_red")
@TeleOp(name = "frontstage_red")
@Config

public class frontstage_red extends LinearOpMode {

    public static int LEG_ONE = 50;
    public static int LEG_TWO = 3;
    public static int LEG_THREE = 87;
    public static int LEG_FOUR = 0;
    public static int LEG_FIVE = 0;
    public static int strafe_s3 = 50;

    @Config
    public static class FR_S1 {
        public static int first_drive = 40;
        public static int second_drive = 35;
        public static int third_drive = 110;
        public static int strafe_s1 = 22;
        public static int fourth_drive = 5;
    }
    @Config
    public static class FR_S2 {
        public static int first_drive = 65;
        public static int second_drive = 10;
        public static int third_drive = 110;
        public static int strafe_s2 = 30;
        public static int fourth_drive = 12;
    }

    DcMotor intake;
    DcMotor conveyor;


    @Override
    public void runOpMode() {

        waitForStart();

        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);
        intake = hardwareMap.get(DcMotor.class,"intake");
        conveyor = hardwareMap.get(DcMotor.class,"conveyor");

        int correctSpike = 0;


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
        if (correctSpike == 1) {
        //if (gamepad1.x) {
            spike1(driver, -90, -3);
        }

        // For middle spike autonomous instructions.
        if (correctSpike == 2) {
        //if (gamepad1.y) {
            spike2(driver);
        }
        // For right spike autonomous instructions.
        if (correctSpike == 3){
        //if (gamepad1.b)
            spike3(driver, 90, 3);
        }

    }
    private void spike1(Autodrive driver, int degrees, int degrees1) {
        driver.drive(FR_S1.first_drive, 0); //drives to middle section
        driver.turn(degrees); //turns to left spike
        driver.drive(-8, -90); //backs closer to spike (maybe not needed)
        intake.setPower(-1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(5, -90); //goes back to previous area
        driver.turn(degrees1); //turns to the middle of field
        driver.drive(FR_S1.second_drive, 0); //drives up to the raising area
        driver.turn(-90);
        driver.drive(FR_S1.third_drive, -90);
        driver.strafe(FR_S1.strafe_s1, -90);
        driver.drive(FR_S1.fourth_drive, -90);
        conveyor.setPower(0.7);
        sleep(2000);
        conveyor.setPower(0);
        //stop - already in backstage now

    }
    private void spike2(Autodrive driver) {
        driver.drive(FR_S2.first_drive,0); //driving through the middle spike
        intake.setPower(-1);//spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(FR_S2.second_drive, 0); //drive out of the way of the bar
        driver.turn(-90);
        driver.drive(FR_S2.third_drive, -90);
        driver.strafe(FR_S2.strafe_s2, -90);
        driver.drive(FR_S2.fourth_drive, -90);
        conveyor.setPower(0.7);
        sleep(2000);
        conveyor.setPower(0);
        //stop - already in backstage now

    }

    private void spike3(Autodrive driver, int degrees, int degrees1) {
        driver.drive(FR_S1.first_drive, 0); //drives to middle section
        driver.turn(degrees); //turns to right spike
        driver.drive(-8, 90); //backs closer to spike (maybe not needed)
        intake.setPower(-1);//feeder spits out pixel
        sleep(1000); //waits so that pixel comes out smoothly
        intake.setPower(0);
        driver.drive(10, 90); //goes back to previous area
        driver.turn(degrees1); //turns to the middle of field
        driver.drive(FR_S1.second_drive, 0); //drives up to the raising area
        driver.turn(-90);
        driver.drive(FR_S1.third_drive, -90);
        driver.strafe(strafe_s3, -90);
        driver.drive(FR_S1.fourth_drive, -90);
        conveyor.setPower(0.7);
        sleep(2000);
        conveyor.setPower(0);
        //stop - already in backstage now

    }

}
