package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Robot;

@TeleOp(name="Distance Test", group="Test")

public class DistanceTest extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);
        telemetry.addLine("Press start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("distance",  robot.drive.distanceToObject());
            telemetry.update();
            sleep(50);
        }
    }
}
