package main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import common.Robot;

@TeleOp(name="Driver")

 public class driver extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);

        // start the drivetrain manual drive thread
        robot.drive.start();

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                robot.lifterUp();
                while (gamepad1.y)
                    sleep(10);
            }
             if (gamepad1.a) {
                 robot.lifterDown();
                 while (gamepad1.a)
                     sleep(10);
             }
             if (gamepad1.x) {
                 robot.bottomGrabberUp();
                 while (gamepad1.x)
                     sleep(10);
             }

             if (gamepad1.b) {
                 robot.bottomGrabberDown();
                    while (gamepad1.b)
                        sleep(10);
             }
        }
    }

}
