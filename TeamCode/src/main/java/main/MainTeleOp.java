package main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import common.Robot;

@TeleOp(name="MainTeleOp", group = "Main")

 public class MainTeleOp extends LinearOpMode {

    Robot   robot;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(this);

        // start the drivetrain manual drive thread
        robot.drive.start();

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        telemetry.addData("Robot Controls", "\n" +
                "  y - set robot to dropper position\n" +
                "  a - set robot to picking position\n" +
                "  x - dropper open / close\n" +
                "  b - picker open / close\n" +
                "  dpad up - lifter up\n" +
                "  dpad down - lifer down\n" +
                "  dpad left or right - drop in bucket\n" +
                "  left stick - arm manual control\n" +
                "  right stick - lifter manual control\n" +
                "\n");

        telemetry.update();

        robot.setToStartPosition();

        while (opModeIsActive()) {

            robotHandleGamepad();
        }
    }

    private void robotHandleGamepad () {

        Gamepad gamepad = gamepad2;

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
            robot.armMoveTo(robot.ARM_EXCHANGE);
            while (robot.isBusy() && opModeIsActive()) sleep(10);
            robot.pickUpSample();
            while (gamepad.y) sleep(10);

        } else if (gamepad.a) {
            robot.setToPickingPosition(robot.AMR_OUT_PART_WAY);
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

        } else if (gamepad.dpad_left || gamepad.dpad_right) {
            robot.dropSampleInTopBucket();
            while (gamepad.dpad_left || gamepad.dpad_right) sleep(10);

        } else if (gamepad.left_stick_y < 0) {
            // retract the arm
            robot.amrRetract();
            while (gamepad.left_stick_y < 0 && robot.armRetractable())
                sleep(10);
            robot.armStop();

        } else if (gamepad.left_stick_y > 0) {
            // extend the arm
            robot.armExtend();
            while (gamepad.left_stick_y > 0 && robot.armExtendable()) {
                sleep(10);
            }
            robot.armStop();

        } else if (gamepad.right_stick_y < 0) {
            robot.LifterExtend();
            while (gamepad2.right_stick_y < 0 && robot.lifterExtendable())
                sleep(10);
            robot.lifterStop();

        } else if (gamepad.right_stick_y > 0) {
            robot.lifterRetract();
            while (gamepad2.right_stick_y > 0 && robot.lifterRetractable())
                sleep(10);
            robot.lifterStop();

        } else if (gamepad.right_bumper) {
            if (robot.pickerIsUp()) {
                robot.pickerDown();
            } else {
                robot.pickerUp();
            }
            while (gamepad.right_bumper) sleep(10);

        } else if (gamepad.left_bumper) {
            robot.pickerRotate();
            while (gamepad.left_bumper) sleep(10);
        }
    }
}
