package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Front Stage - Blue")
public class FrontStage_Blue extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        Autodrive driver = new Autodrive(hardwareMap);

        Spike_Detector detector = new Spike_Detector(10, 207, 112);

        int correctSpike = detector.getCorrectSpike();

        if (correctSpike == 1) {
            driver.drive(52);
            driver.turn(-90);
            driver.drive(-8);
            sleep(3000);
          // use feeder to spit out pixel
            driver.drive(8);
            driver.turn(90);
            driver.drive(20);
        }
    }
}
