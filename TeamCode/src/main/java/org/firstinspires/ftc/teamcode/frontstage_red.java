package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "frontstage_red")
@Config
public class frontstage_red extends RedAutonomous {
    public static int DRIVE_SOUTH = 110;

    @Override
    public void runOpMode() {


        final int spike = driveSpike();
        final int tagId = spike + 3;

        driver.drive(DRIVE_SOUTH, SOUTH);

        driveBackdrop(tagId);
    }
}
