package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Supplier;

/*
 * Basic autonomous movement "blueprints" for driving and turning.
 * Used mainly to be able to get used by other files instead of writing
 * the drive and turn codes again.
 */
@Config
public class Autodrive {

    private final DcMotor leftFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor rightBackDrive;

    public final IMU imu;

    public static int TICKS_PER_INCH = 40;

    public static double MIN_POWER_TO_MOVE = 0.2;

    public static double turnGain =-0.01;

    public static double DriveGain = 0.0005;

    public static double minturnpower = 0.45;

    // By default, keep running
    private Supplier<Boolean> keepRunning = () -> true;

    public Autodrive(DcMotor leftFrontDrive, DcMotor leftBackDrive, DcMotor rightFrontDrive, DcMotor rightBackDrive, IMU imu) {
        this.leftFrontDrive = leftFrontDrive;
        this.leftBackDrive = leftBackDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.rightBackDrive = rightBackDrive;
        this.imu = imu;
    }

    public Autodrive(HardwareMap hardwareMap, Supplier<Boolean> keepRunning) {
        imu = hardwareMap.get(IMU.class, "imu");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backright");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        this.keepRunning = keepRunning;
        imu.resetYaw();
    }

    public void drive(float distanceInches, int direction) {
        float ticksDistance = 4* (distanceInches * TICKS_PER_INCH);

        final int startingPosition =
                        leftBackDrive.getCurrentPosition() +
                        rightBackDrive.getCurrentPosition() +
                        leftFrontDrive.getCurrentPosition() +
                        rightFrontDrive.getCurrentPosition();

        float targetPosition = startingPosition + ticksDistance;

        double error = targetPosition - startingPosition;

        // Stop when roughly within one quarter of an inch.
        while (keepRunning.get() && Math.abs(error) > TICKS_PER_INCH ) {
            double axial = error * DriveGain;

            // If the magnitude of axial power is less than the min drive power,
            // then adjust will be greater than 1.0. Scale without changing
            // it's sign to ensure it's strong enough.
            // If scale is less than 1, then don't make the power any weaker.
            double adjust = MIN_POWER_TO_MOVE / Math.abs(axial);
            if (adjust > 1.0) {
                axial *= adjust;
            }
            axial = Math.min(0.7, axial);

            double yawCorrection = turnGain*(direction-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            stuff(axial, 0, yawCorrection);

            int currentPos =
                            leftBackDrive.getCurrentPosition() +
                            rightBackDrive.getCurrentPosition() +
                            leftFrontDrive.getCurrentPosition() +
                            rightFrontDrive.getCurrentPosition();

            error = targetPosition - currentPos;

            TelemetryPacket stats = new TelemetryPacket();
            stats.put("Axial", error);
            FtcDashboard.getInstance().sendTelemetryPacket(stats);

        }

        // Stop the motors. We made it.
        stuff(0, 0, 0);
    }

    public void turn(final double degrees) {

        double error = degrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while ( Math.abs(error)>3  ) {
            double yaw = error * turnGain;

            // If the magnitude of yaw power is less than the min turn power,
            // then adjust will be greater than 1.0. Scale yaw without changing
            // it's sign to ensure it's strong enough.
            // If scale is less than 1, then don't make the power any weaker.
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

        // Stop the motors. We made it.
        stuff(0, 0, 0);
    }

    public void strafe(double distanceInches, int direction) {
        int ticksDistance = (int)(4* (distanceInches * TICKS_PER_INCH));

        final int startingPosition =
                        -leftBackDrive.getCurrentPosition() +
                        rightBackDrive.getCurrentPosition() +
                        leftFrontDrive.getCurrentPosition() +
                        -rightFrontDrive.getCurrentPosition();

//        imu.resetYaw();
        int targetPosition = startingPosition + ticksDistance;

        float error = targetPosition - startingPosition;

        // Stop when roughly within one quarter of an inch.
        while (keepRunning.get() && Math.abs(error) > TICKS_PER_INCH ) {
            double lateral = error * DriveGain;

            // If the magnitude of lateral power is less than the min drive power,
            // then adjust will be greater than 1.0. Scale without changing
            // it's sign to ensure it's strong enough.
            // If scale is less than 1, then don't make the power any weaker.
            double adjust = MIN_POWER_TO_MOVE / Math.abs(lateral);
            if (adjust > 1.0) {
                lateral = lateral * adjust;
            }

            adjust = 0.5 / Math.abs(lateral);
            if (adjust < 1.0) {
                lateral = lateral * adjust;
            }

            double yawCorrection = turnGain*(direction-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            stuff(0, lateral, yawCorrection);

            int currentPos =
                            -leftBackDrive.getCurrentPosition() +
                            rightBackDrive.getCurrentPosition() +
                            leftFrontDrive.getCurrentPosition() +
                            -rightFrontDrive.getCurrentPosition();

            error = targetPosition - currentPos;

            TelemetryPacket stats = new TelemetryPacket();
            stats.put("Target", targetPosition);
            stats.put("Current", currentPos);
            stats.put("Lateral", error);
            FtcDashboard.getInstance().sendTelemetryPacket(stats);
            //positive value goes right

        }

        // Stop the motors. We made it.
        stuff(0, 0, 0);
    }


    public void stuff(double axial, double lateral, double yaw) {
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