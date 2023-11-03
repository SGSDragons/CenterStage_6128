package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "YawDemo")
public class YawDemo extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        while (opModeIsActive()) {
            TelemetryPacket p = new TelemetryPacket();
            p.put("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }
}
