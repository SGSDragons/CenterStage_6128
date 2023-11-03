package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "StrafeTest", group="TEST")
public class StrafeTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) driver.strafe(-50);
            if (gamepad1.dpad_right) driver.strafe(50);
        }
    }
}
