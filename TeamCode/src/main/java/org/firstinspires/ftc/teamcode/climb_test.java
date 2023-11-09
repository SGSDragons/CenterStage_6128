package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "climb_test")
public class climb_test extends LinearOpMode{

    private DcMotor climb = null;
    public void runOpMode(){


        waitForStart();

        while (opModeIsActive()){

        double climbPower;

      climbPower = 1;

        if (gamepad1.dpad_up){
            climb.setPower(climbPower);


        }
    }
    }
}

