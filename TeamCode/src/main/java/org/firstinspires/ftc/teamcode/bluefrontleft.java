package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "blue front left")
public class bluefrontleft extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);
        driver.drive(-20);
        driver.turn(90);
        driver.drive(10);
    }
}