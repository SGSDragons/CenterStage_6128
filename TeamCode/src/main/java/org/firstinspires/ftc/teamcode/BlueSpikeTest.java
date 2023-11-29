package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "BlueSpikeTest")
public class BlueSpikeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SpikeDetector spike = new SpikeDetector(true);

        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"),spike);
        visionPortal.resumeStreaming();

        waitForStart();

        while (opModeIsActive()) {

            sleep(50);

            if (gamepad1.a) {
                spike.reset();
            }

            TelemetryPacket telemetry = new TelemetryPacket();
            telemetry.put("Spike", spike.getSpikecolumn());
            telemetry.put("Spike1", spike.column1);
            telemetry.put("Spike2", spike.column2);
            telemetry.put("Spike3", spike.column3);

            FtcDashboard.getInstance().sendTelemetryPacket(telemetry);

        }
    }
}
