package main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import common.DriveGamepad;
import common.Robot;

@TeleOp(name="MainTeleOp", group = "Main")

 public class MainTeleOp extends LinearOpMode {

    Robot   robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);
        robot.startDriveGamepad();

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Robot Controls", "\n" +
                "  y - set robot to dropper position\n" +
                "  a - set robot to picking position\n" +
                "  x - dropper open / close\n" +
                "  b - picker open / close\n" +
                "  right bumper - picker up / down\n" +
                "  left bumper - picker rotate\n" +
                "  dpad up - lifter up\n" +
                "  dpad down - lifer down / down\n" +
                "  dpad left - dropper to dropping position\n" +
                "  dpad right - drop in bucket\n" +
                "  left stick - arm manual control\n" +
                "  right stick - lifter manual control\n" +
                "\n");

        telemetry.update();

        robot.setToStartPosition();

        while (opModeIsActive()) {
            robotHandleGamepad();
        }

        robot.setToStopPosition();
    }

    private void robotHandleGamepad () {

        Gamepad gamepad = gamepad2;

        handleReset();

        if (gamepad.x) {
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
            // move the sample from the picker to the dropper
            robot.pickUpSample();
            while (robot.isBusy() && opModeIsActive()) sleep(10);
            robot.armMoveTo(robot.ARM_EXCHANGE);
            while (robot.isBusy() && opModeIsActive()) sleep(10);
            robot.moveSampleToDropper();
            while (gamepad.y) sleep(10);

        } else if (gamepad.a) {
            // move the arm to picking position
            robot.armMoveTo(robot.AMR_OUT_PART_WAY);
            robot.lifterDown();
            robot.pickerUp();
            while (gamepad.a) sleep(10);

        } else if (gamepad.dpad_up) {
            // raise the lifter
            robot.lifterUp();
            while (gamepad.dpad_up) sleep(10);

        } else if (gamepad.dpad_down) {
            // lower the lifter
            robot.lifterDown();
            while (gamepad.dpad_down) sleep(10);

        } else if (gamepad.dpad_left) {
            // move the dropper wrist to dropping position
            if (robot.dropperIsUp())
                robot.dropperDropPosition();
            else
                robot.dropperUp();
            while (gamepad.dpad_left ) sleep(10);

        } else if (gamepad.dpad_right) {
            // drop the sample in the bucket
            robot.dropSampleInTopBucket();
            while (gamepad.dpad_right) sleep(10);

        } else if (gamepad.left_stick_y > 0) {
            // retract the arm
            robot.amrRetract();
            while (gamepad.left_stick_y > 0)
                sleep(10);
            robot.armStop();

        } else if (gamepad.left_stick_y < 0) {
            // extend the arm
            robot.armExtend();
            while (gamepad.left_stick_y < 0) {
                sleep(10);
            }
            robot.armStop();

        } else if (gamepad.right_stick_y < 0) {
            // manually control the lifter
            robot.LifterExtend();
            while (gamepad.right_stick_y < 0)
                sleep(10);
            robot.lifterStop();

        } else if (gamepad.right_stick_y > 0) {
            // manually control the arm
            robot.lifterRetract();
            while (gamepad.right_stick_y > 0)
                sleep(10);
            robot.lifterStop();

        } else if (gamepad.right_bumper) {
            // toggle picker up / down
            if (robot.pickerIsUp()) {
                robot.pickerDown();
            } else {
                robot.pickerUp();
            }
            while (gamepad.right_bumper) sleep(10);

        } else if (gamepad.left_bumper) {
            // rotate the picker
            robot.pickerRotate();
            while (gamepad.left_bumper) sleep(10);
        }
    }

    private boolean handleReset() {

        Gamepad gamepad = gamepad1;
        boolean handled = false;
        if (gamepad.right_bumper) {
            while (gamepad.right_bumper)
                if (gamepad.start) {
                    robot.resetEncoders();
                    handled = true;
                    break;
                }
            while (gamepad.left_bumper || gamepad.start) Thread.yield();
        }
        return handled;
    }
}
