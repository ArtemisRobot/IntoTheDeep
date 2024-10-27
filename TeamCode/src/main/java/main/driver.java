package main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import common.Robot;

@TeleOp(name="Driver")

 public class driver extends LinearOpMode {

    Robot   robot;

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
             if (gamepad2.left_bumper) {
                 while (gamepad2.left_bumper)
                     sleep(10);
                 robot.retractArm();
             }
             if (gamepad2.right_bumper) {
                 while (gamepad2.right_bumper)
                     sleep(10);
                 robot.extendingArm();
             }

             if (gamepad2.a) {
                 robot.pickerDown();
                 while (gamepad1.x)
                     sleep(10);
             }

             if (gamepad2.y) {
                 robot.pickerUp();
                    while (gamepad1.b)
                        sleep(10);
             }

             if (gamepad2.x) {
                robot.pickerOpen();
             }

             if (gamepad2.b) {
                 robot.pickerClosed();
             }
         }
    }

}
