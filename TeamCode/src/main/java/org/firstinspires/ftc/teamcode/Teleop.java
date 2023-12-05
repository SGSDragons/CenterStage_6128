/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="6128Teleop", group="Linear OpMode")
@Config
public class Teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Feeder = null;
    private DcMotor Conveyor = null;
    private Servo Launcher = null;
    private DcMotor LiftRight = null;
    private Servo Hook = null;
    private DcMotor LiftLeft = null;

    public static double BRAKING_FORCE = 0.75;

    @Override
    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backleft");
        Feeder = hardwareMap.get(DcMotor.class, "intake");
        Conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        Launcher = hardwareMap.get(Servo.class, "launcher");
        LiftRight = hardwareMap.get(DcMotor.class,"liftright");
        Hook = hardwareMap.get(Servo.class, "hook");
        LiftLeft = hardwareMap.get(DcMotor.class, "liftleft");
        IMU imu = hardwareMap.get(IMU.class, "imu");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Feeder.setDirection(DcMotor.Direction.FORWARD);
        Conveyor.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setDirection(Servo.Direction.REVERSE);
        Hook.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        final double hookDownPosition = 0;
        final double hookUpPosition = 0.7;

        imu.resetYaw();

        Hook.setPosition(hookDownPosition);
        while (opModeIsActive()) {

            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw  =  gamepad1.right_stick_x;
            double feedPower = gamepad2.right_stick_y;
            double conveyorPower = -gamepad2.left_stick_y;

            if(gamepad1.dpad_down){
                imu.resetYaw();
            }

            if(gamepad1.right_bumper){
                double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                yaw = -angle/90;
            }

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (gamepad1.a){
                LiftRight.setPower(1);
                LiftLeft.setPower(-1);
                sleep(1500);
                LiftRight.getZeroPowerBehavior();
                LiftLeft.getZeroPowerBehavior();
                LiftRight.setPower(0); //necessary?
                LiftLeft.setPower(0); //necessary?
            }

            if (gamepad1.b){
                Hook.setPosition(hookUpPosition);
            }
            if (gamepad1.x) {
                Hook.setPosition(hookDownPosition);
            }
            if (gamepad1.y) {
                LiftRight.setPower(-1);
                LiftLeft.setPower(1);
                sleep(190);
                LiftLeft.setPower(0);
                LiftRight.setPower(0);
            }

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels. Make the trigger act like a brake.
            double breakingForce = (1 - gamepad1.right_trigger*BRAKING_FORCE);
            leftFrontDrive.setPower(leftFrontPower * breakingForce);
            leftBackDrive.setPower(leftBackPower * breakingForce);
            rightFrontDrive.setPower(rightFrontPower * breakingForce);
            rightBackDrive.setPower(rightBackPower * breakingForce);
            Feeder.setPower(feedPower);
            Conveyor.setPower(conveyorPower);

            if(gamepad2.y) {
                Launcher.setPosition(0);
            }else if(gamepad2.x) {
                Launcher.setPosition(1);
            }

            // Show the elapsed game time and wheel power.
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftBackPower, rightBackPower);
//            telemetry.addData("Servo Position", Launcher.getPosition());
//            telemetry.update();

            TelemetryPacket p = new TelemetryPacket();
            p.put("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }
}
