package main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import common.Robot;

@TeleOp(name="Driver")

 public class driver extends LinearOpMode {

    private enum GAMEPAD_MODE { ARM, LIFTER, COMPETITION }
    GAMEPAD_MODE gamepadMode = GAMEPAD_MODE.COMPETITION;

    Robot   robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);

        // start the drivetrain manual drive thread
        robot.drive.start();

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        telemetry.addData("\n Robot Controls", "\n" +
                "  y - set robot to dropper position\n" +
                "  a - set robot to picking position\n" +
                "  x - dropper open / close\n" +
                "  b - picker open / close\n" +
                "  dpad up - lifter up\n" +
                "  dpad down - lifer down\n" +
                "  left stick - lifter manual control\n" +
                "  right stick - arm manual control\n" +
                "\n");

        telemetry.update();

        while (opModeIsActive()) {

            if (gamepadMode == GAMEPAD_MODE.ARM) {
                dropperHandleGamepad();
            } else if (gamepadMode == GAMEPAD_MODE.LIFTER) {
                lifterHandleGamepad();
            } else if (gamepadMode == GAMEPAD_MODE.COMPETITION) {
                robotHandleGamepad();
            }
        }
    }

    private void dropperHandleGamepad () {

        if (gamepad2.left_bumper) {
            robot.armMoveTo(robot.ARM_IN);
            while (gamepad2.left_bumper)
                sleep(10);

        } else if (gamepad2.right_bumper) {
            robot.armMoveTo(robot.ARM_OUT);
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
            while (gamepad2.a)
                sleep(10);

        } else if (gamepad2.y) {
            robot.pickerUp();
            while (gamepad2.y)
                sleep(10);

        } else if (gamepad2.x) {
            robot.pickerOpen();
            while (gamepad2.x)
                sleep(10);

        } else if (gamepad2.b) {
            robot.pickerClose();
            while (gamepad2.b)
                sleep(10);
        }
    }

    private void lifterHandleGamepad () {

        if (gamepad2.left_bumper) {
            robot.lifterDown();
            while (gamepad2.left_bumper)
                sleep(10);

        } else if (gamepad2.right_bumper) {
            robot.lifterUp();
            while (gamepad2.right_bumper)
                sleep(10);

        } else if (gamepad2.right_trigger > 0) {
            robot.LifterExtend();
            while (gamepad2.right_trigger > 0 && robot.lifterExtendable())
                sleep(10);
            robot.lifterStop();

        } else if (gamepad2.left_trigger > 0) {
            robot.lifterRetract();
            while (gamepad2.left_trigger > 0 && robot.lifterRetractable())
                sleep(10);
            robot.lifterStop();

        } else if (gamepad2.a) {
            robot.dropperDown();
            while (gamepad2.a)
                sleep(10);

        } else if (gamepad2.y) {
            robot.dropperUp();
            while (gamepad2.y)
                sleep(10);

        } else if (gamepad2.x) {
            robot.dropperOpen();
            while (gamepad2.x)
                sleep(10);

        } else if (gamepad2.b) {
            robot.dropperClose();
            while (gamepad2.b)
                sleep(10);
        }
    }

    private void robotHandleGamepad () {

        Gamepad gamepad = gamepad1;

        if (gamepad.start) {
            robotStart();
            while (gamepad.start) sleep(10);

        } else if (gamepad.x) {
            // toggle the picker open or closed
            if (robot.pickerIsOpen()) {
                robot.pickerClose();
            } else {
                robot.pickerOpen();
            }
            while (gamepad.x) sleep(10);

        } else if (gamepad.b) {
            // toggle the dropper open or closed
            if (robot.dropperIsOpen()) {
                robot.dropperClose();
            } else {
                robot.dropperOpen();
            }
            while (gamepad.b) sleep(10);

        } else if (gamepad.y) {
            // move the game piece to scoring position
            robot.pickerUp();
            robot.dropperDown();
            sleep(1000);         // wait for the dropper to get to the down position
            robot.armMoveTo(robot.ARM_EXCHANGE);
            robot.dropperClose();
            sleep(400);          // wait for the dropper to get to the closed position
            robot.pickerOpen();
            sleep(400);
            robot.pickerUp();
            while (gamepad.y) sleep(10);

        } else if (gamepad.a) {
            robot.dropperOpen();
            robot.dropperDown();
            robot.armMoveTo(robot.AMR_OUT_PART_WAY);
            robot.pickerOpen();
            robot.pickerDown();
            robot.lifterDown();
            while (gamepad.a) sleep(10);

        } else if (gamepad.dpad_up) {
            // raise the lifter
            robot.lifterUp();
            while (gamepad.dpad_up) sleep(10);

        } else if (gamepad.dpad_down) {
            // lower the lifter
            robot.lifterDown();
            while (gamepad.dpad_down) sleep(10);

        } else if (gamepad.right_stick_y < 0) {
            // retract the arm
            robot.amrRetract();
            while (gamepad.right_stick_y < 0 && robot.armRetractable())
                sleep(10);
            robot.armStop();

        } else if (gamepad.right_stick_y > 0) {
            // extend the arm
            robot.armExtend();
            while (gamepad.right_stick_y > 0 && robot.armExtendable()) {
                sleep(10);
            }
            robot.armStop();

        } else if (gamepad.left_stick_y < 0) {
            robot.LifterExtend();
            while (gamepad2.left_stick_y < 0 && robot.lifterExtendable())
                sleep(10);
            robot.lifterStop();

        } else if (gamepad.left_stick_y > 0) {
            robot.lifterRetract();
            while (gamepad2.left_stick_y > 0 && robot.lifterRetractable())
                sleep(10);
            robot.lifterStop();
        }
    }

    private void robotStart() {

        robot.dropperDown();
        robot.dropperOpen();
        robot.armMoveTo(robot.AMR_OUT_PART_WAY);
        robot.pickerDown();
        robot.pickerOpen();
    }
}
