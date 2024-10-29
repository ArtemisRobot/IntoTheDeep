package main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import common.Logger;
import common.Robot;

@TeleOp(name="Driver")

 public class driver extends LinearOpMode {

    private enum GAMEPAD_MODE { ARM, LIFTER }
    GAMEPAD_MODE gamepadMode = GAMEPAD_MODE.ARM;

    Robot   robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);

        // start the drivetrain manual drive thread
        robot.drive.start();

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", gamepadMode);
        telemetry.update();

        while (opModeIsActive()) {

             if (gamepadMode == GAMEPAD_MODE.ARM) {
                 if (gamepad2.left_bumper) {
                     robot.moveArm(robot.ARM_IN);
                     while (gamepad2.left_bumper)
                         sleep(10);

                 } else if (gamepad2.right_bumper) {
                     robot.moveArm(robot.ARM_OUT);
                     while (gamepad2.right_bumper)
                         sleep(10);

                 } else if (gamepad2.left_trigger > 0) {
                     robot.amrRetract();
                     while (gamepad2.left_trigger > 0 && robot.armRetractable())
                         sleep(10);
                     robot.armStop();

                 } else if (gamepad2.right_trigger > 0) {
                     robot.armExtend();
                     while (gamepad2.right_trigger > 0 && robot.armExtendable()) {
                         sleep(10);
                     }
                     robot.armStop();

                 } else if (gamepad2.a) {
                     robot.pickerDown();
                     while (gamepad1.a)
                         sleep(10);

                 } else if (gamepad2.y) {
                    robot.pickerUp();
                     while (gamepad1.y)
                         sleep(10);

                 } else if (gamepad2.x) {
                    robot.pickerOpen();
                        while (gamepad1.x)
                            sleep(10);

                 } else if (gamepad2.b) {
                    robot.pickerClosed();
                     while (gamepad1.b)
                         sleep(10);
                 }


        } else if (gamepadMode == GAMEPAD_MODE.LIFTER) {

                 if (gamepad2.left_bumper) {
                     robot.lifterDown();
                     while (gamepad2.left_bumper)
                         sleep(10);

                 } else if (gamepad2.right_bumper) {
                     robot.lifterUp();
                     while (gamepad2.right_bumper)
                         sleep(10);

                 } else if (gamepad2.left_trigger > 0) {
                     robot.lifterRetract();
                     while (gamepad2.left_trigger > 0 && robot.armRetractable())
                         sleep(10);
                     robot.armStop();

                 } else if (gamepad2.right_trigger > 0) {
                     robot.LifterExtend();
                     while (gamepad2.right_trigger > 0 && robot.armExtendable()) {
                         sleep(10);
                     }
                     robot.armStop();

                 } else if (gamepad2.a) {
                     robot.dropperDown();
                     while (gamepad1.a)
                         sleep(10);

                 }else if (gamepad2.y) {
                     robot.dropperUp();
                     while (gamepad1.y)
                         sleep(10);
                 } else if (gamepad2.x) {
                     robot.dropperOpen();
                     while (gamepad1.x)
                         sleep(10);

                 } else if (gamepad2.b) {
                     robot.dropperClosed();
                 }
             }

            else if (gamepad2.dpad_left) {
                robot.pickerUp();
                robot.moveArm(robot.ARM_EXCHANGE);
                robot.dropperOpen();
                robot.dropperDown();
                sleep(1000);
                robot.pickerOpen();
                sleep(1000);
                robot.dropperUp();
            }
         }
    }
}
