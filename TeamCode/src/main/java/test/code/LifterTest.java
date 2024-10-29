package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Lifter Test")
@Disabled

 public class LifterTest extends LinearOpMode {

    int LIFTER_UP_POSITION = 1000;
    int LIFTER_DOWN_POSITION = 0;

    double ARM_SERVO_DOWN_POSITION = 0.880;
    double ARM_SERVO_UP_POSITION = .210;

    double LIFTER_SPEED = 0.25;

    DcMotorEx lifter;
    Servo armServo;

    @Override
    public void runOpMode() {
        telemetry.addLine("Press Start");
        telemetry.update();

        lifter = hardwareMap.get(DcMotorEx.class, "lifter");
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armServo = hardwareMap.get(Servo.class, "armServo");

        waitForStart();



        while (opModeIsActive()) {

            if (gamepad1.y) {
                lifterUp();
                while (gamepad1.y)
                    sleep(10);
            }
             if (gamepad1.a) {
                 lifterDown();
                 while (gamepad1.a)
                     sleep(10);
             }
             if (gamepad1.x) {
                 grabberUp();
                 while (gamepad1.x)
                     sleep(10);
             }

             if (gamepad1.b) {
                 grabberDown();
                    while (gamepad1.b)
                        sleep(10);
             }
        }
    }

    private void grabberUp() {
        armServo.setPosition(ARM_SERVO_DOWN_POSITION);
    }

    private void grabberDown() {
        armServo.setPosition(ARM_SERVO_UP_POSITION);
    }

    private void lifterUp() {

        lifter.setTargetPosition(LIFTER_UP_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(LIFTER_SPEED);
        while (opModeIsActive() && lifter.isBusy()) {
            if (gamepad1.back)
                break;
        }
        lifter.setPower(0);
    }

    private void lifterDown() {
        lifter.setTargetPosition(LIFTER_DOWN_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(LIFTER_SPEED);
        while (opModeIsActive() && lifter.isBusy()) {
            if (gamepad1.back)
                break;
        }
        lifter.setPower(0);

    }
}
