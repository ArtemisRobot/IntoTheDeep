package test.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Robot;

@TeleOp(name=" AutoTest", group = "Test")

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
                robot.setToStopPosition();
                while (gamepad1.a) sleep(10);
            } else if (gamepad1.x) {
                robot.pickUpYellow();
                while (gamepad1.x) sleep(10);
            } else if (gamepad1.b) {
                robot.dropperClose();
                while (gamepad1.b) sleep(10);
            } else if (gamepad1.y) {
                robot.setToStartPosition();
                while (gamepad1.y) sleep(10);
            }

            telemetry.addData("Robot is busy", robot.isBusy());
            telemetry.addData("ok to move robot", robot.okToMove());
            telemetry.update();
            sleep(10);
        }
    }
}
