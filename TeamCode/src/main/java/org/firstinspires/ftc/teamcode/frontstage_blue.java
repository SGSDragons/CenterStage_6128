package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "frontstage_blue")
@Config


public class frontstage_blue extends LinearOpMode {

    public static int LEG_ONE = 50;
    public static int LEG_TWO = 3;
    public static int LEG_THREE = 87;
    public static int LEG_FOUR = 0;
    public static int LEG_FIVE = 0;


    @Override
    public void runOpMode() {

        waitForStart();

        DcMotor Feeder = hardwareMap.get(DcMotor.class, "rightarm");
        Feeder.setDirection(DcMotor.Direction.REVERSE);

        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);
        driver.drive(LEG_ONE);
        Feeder.setPower(-2);
        sleep(1000);
        driver.drive(LEG_TWO);
        driver.turn(78);
        driver.drive(LEG_THREE);
        driver.strafe(LEG_FOUR);
    }
}
