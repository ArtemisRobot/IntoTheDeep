package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import common.Logger;
import common.Robot;

@TeleOp(name="RobotTest", group="Test")
//@Disabled
 public class RobotTest extends LinearOpMode {

    private enum GAMEPAD_MODE { PICKER, DROPPER, ARM, LIFTER, ROBOT, AUTO, SPECIMEN }
    GAMEPAD_MODE gamepadMode = GAMEPAD_MODE.DROPPER;

    Robot   robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        displayControls();

        while (opModeIsActive()) {

            handleModeChange();

            switch (gamepadMode) {
                case PICKER:
                    pickersHandleGamepad();
                    break;
                case DROPPER:
                    dropperHandleGamepad();
                    break;
                case ARM:
                    armHandleGamepad();
                    break;
                case LIFTER:
                    lifterHandleGamepad();
                    break;
                case ROBOT:
                    robotHandleGamepad();
                    break;
                case AUTO:
                    autoHandleGamepad();
                    break;
                case SPECIMEN:
                    specimenHandleGamepad();
                    break;
            }
        }
    }

    private void handleModeChange() {

        if (gamepad1.dpad_left) {
            int index = (gamepadMode.ordinal() - 1);
            if (index < 0)
                index = GAMEPAD_MODE.values().length - 1;
            gamepadMode = GAMEPAD_MODE.values()[index];
            displayControls();
            while (gamepad1.dpad_left) sleep(10);

        } else if (gamepad1.dpad_right) {
            int index = (gamepadMode.ordinal() + 1) % GAMEPAD_MODE.values().length;
            gamepadMode = GAMEPAD_MODE.values()[index];
            displayControls();
            while (gamepad1.dpad_right) sleep(10);
        }
    }

    private void displayControls() {

        telemetry.addData("Mode:", gamepadMode);

        switch (gamepadMode) {
            case PICKER:
                displayPickerControls();
                break;
            case DROPPER:
                displayDropperControls();
                break;
            case ARM:
                displayArmControls();
                break;
            case LIFTER:
                displayLifterControls();
                break;
            case ROBOT:
                displayRobotControls();
                break;
            case AUTO:
                displayAutoControls();
                break;
            case SPECIMEN:
                break;
        }
        telemetry.update();
    }

    private void displayPickerControls() {
        telemetry.addData("\n Grabber Controls", "\n" +
                "  y - picker up\n" +
                "  a - picker down\n" +
                "  x - picker open\n" +
                "  b - picker close\n" +
                "  right bumper - picker rotate\n" +
                "\n");
    }

    private void displayDropperControls() {
        telemetry.addData("\n Dropper Controls", "\n" +
                "  y - up\n" +
                "  a - down\n" +
                "  x - open\n" +
                "  b - close\n" +
                "  right bumper - ascent position\n" +
                "\n");
    }

    private void displayArmControls() {
        telemetry.addData("\n Dropper Controls", "\n" +
                "  y - arm out\n" +
                "  a - arm in\n" +
                "  x - arm exchange position\n" +
                "  b - arm aoto pick position\n" +
                "  left trigger - arm manually retract\n" +
                "  right trigger - arm manually extend\n" +
                "\n");
    }
    private void displayLifterControls() {
        telemetry.addData("\n Dropper Controls", "\n" +
                "  dpad up - lifter up\n" +
                "  dpad down - lifter down\n" +
                "  left trigger - arm manually retract\n" +
                "  right trigger - arm manually extend\n" +
                "\n");
    }

    private void displayRobotControls() {
        telemetry.addData("\n Robot Controls", "\n" +
                "  a - pickup sample\n" +
                "  y - move sample to dropper\n" +
                "  b - drop sample in bucket\n" +
                "  dpad up - lifter up\n" +
                "  dpad down - lifer down\n" +
                "  left stick - lifter manual control\n" +
                "  right stick - arm manual control\n" +
                "\n");
    }

    private void displayAutoControls() {
        telemetry.addData("\n Robot Controls", "\n" +
                "  a - pickup sample\n" +
                "  y - drop sample in bucket\n" +
                "\n");
    }


    private void pickersHandleGamepad () {

        Gamepad gamepad = gamepad1;

        if (gamepad.y) {
            robot.pickerUp();
            while (gamepad.y)
                sleep(10);

        } else if (gamepad.a) {
            robot.pickerDown();
            while (gamepad.a)
                sleep(10);

        } else if (gamepad.x) {
            robot.pickerOpen();
            while (gamepad.x)
                sleep(10);

        } else if (gamepad.b) {
            robot.pickerClose();
            while (gamepad.b)
                sleep(10);

        } else if (gamepad.left_bumper) {
            robot.pickerRotate();
            while (gamepad.left_bumper) sleep(10);

        } else if (gamepad.right_bumper) {
            // toggle the picker open or closed
            if (robot.pickerIsOpen()) {
                robot.pickerClose();
            } else {
                robot.pickerOpen();
            }
            while (gamepad.right_bumper) sleep(10);
        }
    }

    private void dropperHandleGamepad() {

        Gamepad gamepad = gamepad1;

        if (gamepad.a) {
            robot.dropperDown();
            while (gamepad.a)
                sleep(10);

        } else if (gamepad.y) {
            robot.dropperUp();
            while (gamepad.y)
                sleep(10);

        } else if (gamepad.x) {
            robot.dropperOpen();
            while (gamepad.x)
                sleep(10);

        } else if (gamepad.b) {
            robot.dropperClose();
            while (gamepad.b)
                sleep(10);

        } else if (gamepad.left_bumper) {
            // toggle the dropper open or closed
            if (robot.dropperIsOpen()) {
                robot.dropperClose();
            } else {
                robot.dropperOpen();
            }
            while (gamepad.left_bumper) sleep(10);
        }

        else if (gamepad.right_bumper) {
            robot.dropperAscent();
            sleep(50);
            requestOpModeStop();
        }
    }

    private void armHandleGamepad () {
        Gamepad gamepad = gamepad1;

        if (gamepad.y) {
            robot.armMoveTo(robot.ARM_OUT);
            while (gamepad.y)
                sleep(10);

        } else if (gamepad.a) {
            robot.armMoveTo(robot.ARM_IN);
            while (gamepad.a)
                sleep(10);

        } else if (gamepad.x) {
            robot.armMoveTo(robot.ARM_EXCHANGE, robot.ARM_HIGH_SPEED);
            while (gamepad.x)
                sleep(10);

        } else if (gamepad.b) {
            robot.armMoveTo(robot.ARM_AUTO_PICK, robot.ARM_HIGH_SPEED);
            while (gamepad.b)
                sleep(10);

        } else if (gamepad.left_trigger > 0) {
            robot.amrRetract();
            while (gamepad.left_trigger > 0 && robot.armRetractable())
                sleep(10);
            robot.armStop();

        } else if (gamepad.right_trigger > 0) {
            robot.armExtend();
            while (gamepad.right_trigger > 0 && robot.armExtendable()) {
                sleep(10);
            }
            robot.armStop();
        }
    }

    private void lifterHandleGamepad () {

        Gamepad gamepad = gamepad1;

        if (gamepad.dpad_down) {
            robot.lifterDown();
            while (gamepad.dpad_down)
                sleep(10);

        } else if (gamepad.dpad_up) {
            robot.lifterUp();
            while (gamepad.dpad_up)
                sleep(10);

        } else if (gamepad.right_trigger > 0) {
            robot.LifterExtend();
            while (gamepad.right_trigger > 0)
                sleep(10);
            robot.lifterStop();

        } else if (gamepad.left_trigger > 0) {
            robot.lifterRetract();
            while (gamepad.left_trigger > 0)
                sleep(10);
            robot.lifterStop();
        }
    }

    private void robotHandleGamepad () {

        Gamepad gamepad = gamepad1;

        if (gamepad.start) {
            robot.setToStartPosition();
            while (gamepad.start) sleep(10);

        } else if (gamepad.y) {
            // move the game piece to scoring position
            robot.moveSampleToDropper();
            while (gamepad.y) sleep(10);

        } else if (gamepad.a) {
            robot.pickUpSample();
            while (gamepad.a) sleep(10);

        } else if (gamepad.b) {
            robot.dropSampleInTopBucket();
            while (gamepad.b) sleep(10);

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
            while (gamepad.left_stick_y < 0)
                sleep(10);
            robot.lifterStop();

        } else if (gamepad.left_stick_y > 0) {
            robot.lifterRetract();
            while (gamepad.left_stick_y > 0)
                sleep(10);
            robot.lifterStop();
        }
    }

    private void autoHandleGamepad () {

        Gamepad gamepad = gamepad1;
        if (gamepad.a) {
            robot.pickUpSample();
            waitUntilRobotIdIdle();
            robot.armMoveTo(robot.ARM_EXCHANGE, robot.ARM_HIGH_SPEED);
            waitUntilRobotIdIdle();
            robot.moveSampleToDropper();
            while (gamepad.a) sleep(10);
        }

        if (gamepad.y) {
            robot.dropSampleInTopBucket();
            while (gamepad.y) sleep(10);
        }
    }

    private  void specimenHandleGamepad () {

        Gamepad gamepad = gamepad1;

        if (gamepad.a) {
            robot.scoreSpecimen();
            while (gamepad.a) sleep(10);
        }

        if (gamepad.y) {
            robot.dropperSpecimenUp();
            while (gamepad.y) sleep(10);
        }

        if (gamepad.b) {
            robot.dropperSpecimenDown();
            sleep(100);
            robot.dropperOpen();
            while (gamepad.b) sleep(10);
        }

        if (gamepad.dpad_up) {
            robot.lifterToTopBar();
            while (gamepad.dpad_up) sleep(10);
        }

        if (gamepad.dpad_down) {
            robot.lifterDown();
            while (gamepad.dpad_down) sleep(10);
        }
    }

    private void waitUntilRobotIdIdle() {
        Logger.message("waiting");
        long start = System.currentTimeMillis();
        while (robot.isBusy() && opModeIsActive()) {
            if (System.currentTimeMillis()-start > 3000) {
                Logger.warning("robot timed out");
                break;
            }
        }
        Logger.message("done waiting");
    }

}

