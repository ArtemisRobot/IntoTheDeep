package test.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Robot;

@TeleOp(name="AutoTest", group = "Test")

public class AutoTest extends LinearOpMode {

    Robot robot;

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
            } else if (gamepad1.left_bumper) {
                if (robot.dropperIsUp()) {
                    robot.dropperDown();
                } else {
                    robot.dropperUp();
                }
                while (gamepad1.left_bumper) sleep(10);
            }

            telemetry.addData("Robot is busy", robot.isBusy());
            telemetry.addData("ok to move robot", robot.okToMove());
            telemetry.update();
            sleep(10);
        }
    }
}
