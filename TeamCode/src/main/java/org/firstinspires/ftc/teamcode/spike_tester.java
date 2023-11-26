package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Config
public class spike_tester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SpikeDetector vision = new SpikeDetector(true);

        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"),vision);
        visionPortal.resumeStreaming();

        waitForStart();
        while (opModeIsActive()){
            sleep(2000);
        }
    }
}
