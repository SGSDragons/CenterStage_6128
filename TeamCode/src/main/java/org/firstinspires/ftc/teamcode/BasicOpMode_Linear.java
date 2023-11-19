package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="6128_Teleop", group="Linear OpMode")

public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Feeder = null;
    private DcMotor Conveyor = null;
    private Servo Launcher = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Feeder.setDirection(DcMotor.Direction.REVERSE);
        Conveyor.setDirection(DcMotor.Direction.REVERSE);
        Launcher.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double max;

            double drive = gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;
            double feed = gamepad2.right_stick_y;
            double convey = gamepad2.left_stick_y;

            double leftFrontPower = -drive + lateral + turn;
            double leftBackPower = drive + lateral - turn;
            double rightFrontPower = drive + lateral + turn;
            double rightBackPower = -drive + lateral - turn;
            double FeedPower  = feed;
            double ConveyorPower = -convey;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower/1.5);
            leftBackDrive.setPower(leftBackPower/1.5);
            rightFrontDrive.setPower(rightFrontPower/1.5);
            rightBackDrive.setPower(rightBackPower/1.5);
            Feeder.setPower(FeedPower);
            Conveyor.setPower(ConveyorPower);

            if(gamepad2.y) {
                Launcher.setPosition(-1);
            }else if(gamepad2.x) {
                Launcher.setPosition(1);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftBackPower, rightBackPower);
            telemetry.addData("Servo Position", Launcher.getPosition());
            telemetry.update();
        }
    }
}