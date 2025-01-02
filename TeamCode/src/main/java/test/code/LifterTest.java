package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import common.Lifter;

@TeleOp(name="Lifter Test")

 public class LifterTest extends LinearOpMode {

    public static double LIFTER_SPEED = 0.50;
    public static double LIFTER_SPEED_LOW = 0.20;
    public static int    LIFTER_STOP_TICKS = 500;
    public static int    LIFTER_UP_POSITION = 3228;
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
                lifter.setPosition(LIFTER_UP_POSITION, LIFTER_SPEED, LIFTER_SPEED);
                while (gamepad1.y)
                    sleep(10);
            }

             if (gamepad1.a) {
                 lifter.setPosition(LIFTER_DOWN_POSITION, LIFTER_SPEED, LIFTER_SPEED_LOW);
                 while (gamepad1.a)
                     sleep(10);
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
