package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "StrafeTest", group="TEST")
@Disabled
public class StrafeTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Autodrive driver = new Autodrive(hardwareMap, this::opModeIsActive);

        waitForStart();
        while (opModeIsActive()) {
            double heading = driver.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (gamepad1.dpad_left) driver.strafe(-50, (int)heading);
            if (gamepad1.dpad_right) driver.strafe(50, (int)heading);
        }
    }
}
