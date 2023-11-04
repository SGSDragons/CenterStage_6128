package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "backstage_red")
@Config
public class backstage_red extends RedAutonomous {
    public static int DRIVE_SOUTH = 30;

    @Override
    public void runOpMode() {
        final int spike = driveSpike();
        final int tagId = spike + 3;

        driver.drive(DRIVE_SOUTH, SOUTH);

        driveBackdrop(tagId);
    }
}
