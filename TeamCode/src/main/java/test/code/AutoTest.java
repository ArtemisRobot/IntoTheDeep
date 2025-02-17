package test.code;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Logger;
import common.Robot;

@Disabled
@TeleOp(name="AutoTest", group = "Test")

public class AutoTest extends LinearOpMode {

    Robot robot;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        robot = new Robot(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Press start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                robot.dropSampleInTopBucket();
                while (gamepad1.a) sleep(10);

            } else if (gamepad1.x) {
                robot.pickUpSample();
                while (gamepad1.x) sleep(10);

            } else if (gamepad1.b) {
                // toggle the dropper open or closed
                if (robot.dropperIsOpen()) {
                    robot.dropperClose();
                } else {
                    robot.dropperOpen();
                }
                while (gamepad1.b) sleep(10);

            } else if (gamepad1.y) {
                robot.setToStartPosition();
                while (gamepad1.y) sleep(10);

            } else if (gamepad1.right_bumper) {
                if (robot.pickerIsUp()) {
                    robot.pickerDown();
                } else {
                    robot.pickerUp();
                }
                while (gamepad1.right_bumper) sleep(10);

            } else if (gamepad1.left_bumper) {
                if (robot.dropperIsUp()) {
                    robot.dropperDown();
                } else {
                    robot.dropperUp();
                }
                while (gamepad1.left_bumper) sleep(10);

            } else if (gamepad1.dpad_up) {
                long start = System.currentTimeMillis();
                robot.lifterUp();
                while (robot.lifterIsBusy() && opModeIsActive()) {
                    sleep(1);
                }
                Logger.message(String.format("lifter up time: %,d milliseconds", System.currentTimeMillis() - start));
            }

            else if (gamepad1.dpad_down) {
                long start = System.currentTimeMillis();
                robot.lifterDown();
                while (robot.lifterIsBusy() && opModeIsActive()) {
                    sleep(1);
                }
                Logger.message(String.format("lifter down time: %,d milliseconds", System.currentTimeMillis() - start));
            }

            telemetry.addData("Robot is busy", robot.isBusy());
            telemetry.addData("ok to move robot", robot.okToMove());
            telemetry.update();
            sleep(10);
        }
    }
}
