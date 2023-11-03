/* Copyright (c) 2021 FIRST. All rights reserved.
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
@Config
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private IMU imu = null;

    public static int TICKS_PER_INCH = 40;

    public static double MIN_POWER_TO_MOVE = 0.0;

    public static int go = 0;

    public static double turnGain =-0.02;

    public static double minturnpower = 0.45;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        imu = hardwareMap.get(IMU.class, "imu");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int leftFrontStart = leftFrontDrive.getCurrentPosition();
        int leftBackStart = leftBackDrive.getCurrentPosition();
        int rightFrontStart = rightFrontDrive.getCurrentPosition();
        int rightBackStart = rightBackDrive.getCurrentPosition();
        imu.resetYaw();

        Autodrive driver = new Autodrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, imu);
        driver.drive(36);
        driver.turn(90);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            stuff(axial, lateral, yaw);

            TelemetryPacket positions = new TelemetryPacket();
            positions.put("FrontLeft", leftFrontDrive.getCurrentPosition() - leftFrontStart);
            positions.put("BackLeft", leftBackDrive.getCurrentPosition() - leftBackStart);
            positions.put("FrontRight", rightFrontDrive.getCurrentPosition() - rightFrontStart);
            positions.put("BackRight", rightBackDrive.getCurrentPosition() - rightBackStart);
            positions.put("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            positions.put("GO", go);
            FtcDashboard.getInstance().sendTelemetryPacket(positions);
        }
    }

    public void driveForward(int distanceInches) {
        int ticksDistance = 4* (distanceInches * TICKS_PER_INCH);

        final int startingPosition =
            leftBackDrive.getCurrentPosition() +
            rightBackDrive.getCurrentPosition() +
            leftFrontDrive.getCurrentPosition() +
            rightFrontDrive.getCurrentPosition();

        int ticksTravelled = 0;

        while (ticksTravelled < ticksDistance && opModeIsActive()) {
            int currentPos = leftBackDrive.getCurrentPosition() +
                    rightBackDrive.getCurrentPosition() +
                    leftFrontDrive.getCurrentPosition() +
                    rightFrontDrive.getCurrentPosition();

            ticksTravelled = currentPos - startingPosition;
            int remainingInches = (ticksDistance - ticksTravelled)/TICKS_PER_INCH;

            if (remainingInches > 10) {
                stuff(1.0, 0, 0); // Full power when more than 10 inches away
            } else {
                double axial = Math.max(remainingInches/10.0, MIN_POWER_TO_MOVE);
                stuff(axial, 0, 0);
            }
        }
    }

    public void turn(final double degrees) {
        imu.resetYaw();

        double error = degrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);



        while ( opModeIsActive() && Math.abs(error)>3  ) {
            double yaw = error * turnGain;

            // If the magnitude of yaw power is less than the min turn power,
            // then adjust will be greater than 1.0. Scale yaw without changing
            // it's sign to ensure it's strong enough.
            // If it's less than 1, then don't make it any weaker.
            double adjust = minturnpower / Math.abs(yaw);
            if (adjust > 1.0) {
                yaw *= adjust;
            }

            stuff(0, 0, yaw);
            error = degrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            TelemetryPacket stats = new TelemetryPacket();
            stats.put("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            FtcDashboard.getInstance().sendTelemetryPacket(stats);

        }
    }

    public void stuff(double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower   = leftBackPower / max;
            rightBackPower  = rightBackPower / max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

}
