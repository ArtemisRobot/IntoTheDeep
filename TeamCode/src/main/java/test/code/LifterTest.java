package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Lifter;
import common.Logger;

@com.acmerobotics.dashboard.config.Config
@TeleOp(name="Lifter Test")

 public class LifterTest extends LinearOpMode {

    public static double LIFTER_SPEED = 0.50;
    public static double LIFTER_SPEED_LOW = 0.20;
    public static int    LIFTER_STOP_TICKS = 500;
    public static int    LIFTER_UP_POSITION = 1614;
    public static int    LIFTER_DOWN_POSITION = 0;

    Lifter lifter;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press Start");
        telemetry.update();

        lifter = new Lifter(this);
        lifter.setRange(LIFTER_DOWN_POSITION, LIFTER_UP_POSITION);
        lifter.setLowSpeedThreshold(LIFTER_STOP_TICKS);
        lifter.start();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                long start = System.currentTimeMillis();
                lifter.setPosition(LIFTER_UP_POSITION, LIFTER_SPEED, LIFTER_SPEED);
                while (gamepad1.y)
                    sleep(10);
                telemetry.addData("lifter up time: %6d", System.currentTimeMillis()-start);
                telemetry.update();
            }

             if (gamepad1.a) {
                 long start = System.currentTimeMillis();
                 lifter.setPosition(LIFTER_DOWN_POSITION, LIFTER_SPEED, LIFTER_SPEED_LOW);
                 while (gamepad1.a)
                     sleep(10);
                 telemetry.addData("lifter up time: %6d", System.currentTimeMillis()-start);
                 telemetry.update();
             }

             else if (gamepad1.right_stick_y < 0) {
                 lifter.runLifter(LIFTER_SPEED);
                 while (gamepad1.right_stick_y < 0)
                     sleep(10);
                 lifter.stopLifter();

             } else if (gamepad1.right_stick_y > 0) {
                 lifter.runLifter(-LIFTER_SPEED);
                 while (gamepad1.right_stick_y > 0)
                     sleep(10);
                 lifter.stopLifter();
             }
        }
    }
}
