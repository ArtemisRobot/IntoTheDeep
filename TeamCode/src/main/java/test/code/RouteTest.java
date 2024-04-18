package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Drive;

@TeleOp(name="Route Test", group="Test")
public class RouteTest extends LinearOpMode {
    Drive drive = null;
    private final ElapsedTime runtime = new ElapsedTime();

    private enum TUNING { DRIVE, STRAFE, TURN }
    private TUNING tuning = TUNING.DRIVE;

    private double factor = Drive.DRIVE_FACTOR;


    private static final double ODOMETER_TICKS_PER_REV = 2000;
    private static final double ODOMETER_WHEEL_DIAMETER = 1.90278;  //  1.92913;       // 0.9863 in inches, 48mm diameter
    private static final double ODOMETER_COUNTS_PER_INCH = ODOMETER_TICKS_PER_REV / (ODOMETER_WHEEL_DIAMETER * Math.PI);

    private final String INCHES_FORMAT = " %6.3f";
    private final String FACTOR_FORMAT = " %6.3f";

    Telemetry.Item toTravelMsg;
    Telemetry.Item traveled1Msg;
    Telemetry.Item traveled2Msg;
    Telemetry.Item factorMsg;

    public void runOpMode() {
        drive = new Drive(this);

        telemetry.addLine("push start");
        telemetry.update();

        waitForStart();

        telemetry.setAutoClear(false);
        telemetry.addData("Controls", "\n" +
                "  y - move forward\n" +
                "  a - move backward\n" +
                "  x - strafe left\n" +
                "  b - strafe right\n" +
                "  dpad_left - turn left\n" +
                "  dpad_right - turn right\n\n" +
                "  left bumper - decrease inches to move\n" +
                "  right bumper - increase inches to move\n" +
                "  left trigger - decrease tuning factor\n" +
                "  right trigger - increase tuning factor\n" +
                "\n");

        double inches = 12;
        factorMsg = telemetry.addData("drive factor", FACTOR_FORMAT, factor);
        toTravelMsg = telemetry.addData("inches to travel", INCHES_FORMAT, inches);
        traveled1Msg = telemetry.addData("inches traveled odometer", 0);
        traveled2Msg = telemetry.addData("inches traveled encoders", 0);
        telemetry.update();


        while (opModeIsActive()) {

            if (gamepad1.y) {
                moveForwardBackward(Drive.DIRECTION.FORWARD, inches);
            }
            if (gamepad1.a) {
                moveForwardBackward(Drive.DIRECTION.BACK, inches);
            }
            if (gamepad1.x) {
                drive.strafeLeft(inches);
            }
            if (gamepad1.b) {
                drive.strafeRight(inches);
            }
            if (gamepad1.dpad_left){
                drive.turn(90);
            }
            if (gamepad1.dpad_right) {
                drive.turn(-90);
            }
            if (gamepad1.left_bumper) {
                runtime.reset();
                while (gamepad1.left_bumper) {
                    inches -= (double) increment(.5, 1, 2);
                    toTravelMsg.setValue( INCHES_FORMAT, inches);
                    telemetry.update();
                }
            }
            if (gamepad1.right_bumper) {
                runtime.reset();
                while (gamepad1.right_bumper) {
                    inches += (double) increment(.5, 1, 2);
                    toTravelMsg.setValue( INCHES_FORMAT, inches);
                    telemetry.update();
                }
            }
            if (gamepad1.left_trigger != 0) {
                runtime.reset();
                while (gamepad1.left_trigger != 0) {
                    factor -= (double) increment(0.001, 0.005, 0.01);
                    SetTuningFactor(factor);
                }
            }
            if (gamepad1.right_trigger != 0) {
                runtime.reset();
                while (gamepad1.right_trigger != 0) {
                    factor += (double) increment(0.001, 0.005, 0.01);
                    SetTuningFactor(factor);
                }
            }
            if (gamepad1.back) {
                SelectTuning();
                while (gamepad1.back) sleep(100);
            }
         }
    }

    private void SelectTuning () {
        if (tuning == TUNING.DRIVE) {
            tuning = TUNING.STRAFE;
            factor = Drive.STRAFE_FACTOR;
            factorMsg.setCaption("strafe factor");
        } else if (tuning == TUNING.STRAFE) {
            tuning = TUNING.TURN;
            factor = Drive.TURN_FACTOR;
            factorMsg.setCaption("turn factor");
        } else if (tuning == TUNING.TURN) {
            tuning = TUNING.DRIVE;
            factor = Drive.DRIVE_FACTOR;
            factorMsg.setCaption("drive factor");
        }
        factorMsg.setValue( FACTOR_FORMAT, factor);
        telemetry.update();
    }

    private void SetTuningFactor (double factor) {
        if (tuning == TUNING.DRIVE) {
            Drive.DRIVE_FACTOR = factor;
        } else if (tuning == TUNING.STRAFE) {
            Drive.STRAFE_FACTOR = factor;
        } else if (tuning == TUNING.TURN) {
            Drive.TURN_FACTOR = factor;
        }
        factorMsg.setValue( FACTOR_FORMAT, factor);
        telemetry.update();
    }


    private void moveForwardBackward (Drive.DIRECTION direction, double inches) {

        //double odometerStart = (double)drive.sideEncoder.getCurrentPosition() / ODOMETER_COUNTS_PER_INCH;

        if (direction == Drive.DIRECTION.FORWARD)
            drive.forward(inches);
        else if (direction == Drive.DIRECTION.BACK)
            drive.back(inches);

        sleep(1000);
        //double odometerEnd = (double)drive.sideEncoder.getCurrentPosition() / ODOMETER_COUNTS_PER_INCH;
        // traveled1Msg.setValue(INCHES_FORMAT, odometerEnd - odometerStart);
        traveled2Msg.setValue(INCHES_FORMAT, drive.getDistanceTraveled());
        telemetry.update();
    }

    private void moveLeftRight (Drive.DIRECTION direction, double inches) {

        if (direction == Drive.DIRECTION.LEFT)
            drive.strafeLeft(inches);
        else if (direction == Drive.DIRECTION.RIGHT)
            drive.strafeRight(inches);
    }

    /* Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    private double increment(double v1, double v2, double v3){
        long sleepTime;
        double delta;
        if (runtime.seconds() < 3){
            delta = v1;
            sleepTime = 500;
        }
        else if (runtime.seconds() < 6){
            delta = v2;
            sleepTime = 250;
        }
        else{
            delta = v3;
            sleepTime = 250;
        }
        sleep(sleepTime);
        return delta;
    }

}
