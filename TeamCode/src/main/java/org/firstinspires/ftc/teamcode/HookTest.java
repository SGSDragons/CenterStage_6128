package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="11HookTest")
@Config
public class HookTest extends LinearOpMode {

    public static double pos = 0.75;
    @Override
    public void runOpMode() {
        DcMotor LiftRight = hardwareMap.get(DcMotor.class,"liftright");
        Servo Hook = hardwareMap.get(Servo.class, "hook");
        DcMotor LiftLeft = hardwareMap.get(DcMotor.class, "liftleft");
        Servo Launcher = hardwareMap.get(Servo.class, "launcher");

//        Hook.scaleRange(LOWERED_POSITION, RAISED_POSITION);

        waitForStart();
        Hook.setPosition(0);

        while (opModeIsActive()) {
//            while (gamepad1.a){
//                LiftRight.setPower(1);
//                LiftLeft.setPower(-1);
//                driver.drive(3,180);
//                sleep(1500);
//                LiftRight.getZeroPowerBehavior();
//                LiftLeft.getZeroPowerBehavior();
//                LiftRight.setPower(0); //necessary?
//                LiftLeft.setPower(0); //necessary?
//            }

            Hook.setPosition(pos);

            TelemetryPacket tp = new TelemetryPacket();
            tp.put("hook-pos", Hook.getPosition());
            FtcDashboard.getInstance().sendTelemetryPacket(tp);
//            if (gamepad1.a) {
//                Launcher.setPosition(0.25);
//            }
            if (gamepad1.y) {
                LiftRight.setPower(-1);
                LiftLeft.setPower(1);
                sleep(190);
                LiftLeft.setPower(0);
                LiftRight.setPower(0);
            }

        }

    }
}
