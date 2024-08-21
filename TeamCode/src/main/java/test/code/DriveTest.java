/*
 * Test code for threads
 */
package test.code;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;

import common.Drive;
import common.Logger;
import common.PIDController;

@TeleOp(name="* Drive Test", group="Test")
@SuppressWarnings("unused")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class DriveTest extends LinearOpMode {

    double MIN_SPEED = 0.25;
    double MAX_SPEED = 0.9;
    double MIN_ROTATE_SPEED = 0.25;
    double MAX_ROTATE_SPEED = 0.50;

    public static double speed = 0.50;
    public static double speedX = 0;
    public static double speedY = 0;
    public static double speedYaw = 0;

    public static double leftFrontPower  = 0;
    public static double rightFrontPower = 0;
    public static double leftRearPower   = 0;
    public static double rightRearPower  = 0;

    public static double inches = 96;
    public static double timeout = 0;

    double velocity;

    Drive drive  = null;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new Drive(this);
        //drive.start();

        telemetry.addData("Status", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.clear();

        //checkAccuracy();

       //runTest();

        newDrive();

        drive.setBraking(false);

        while (opModeIsActive()) {

            validateConfig();
            telemetry.addLine("Controls:");
            telemetry.addLine("  y - move forward");
            telemetry.addLine("  a - move backward\n");
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Inches", "%4.1f", inches);
            telemetry.addData("Drift", "%8.2f", drive.totalDrift);
            telemetry.addData("Yaw:", "%6.2f", drive.getOrientation() );
            telemetry.addData("Odometer:", "%d", drive.odometer.getCurrentPosition());

            telemetry.update();

            if (gamepad1.y) {
                testCorrection(inches, Drive.DIRECTION.FORWARD);
            }

            if (gamepad1.a) {
                testCorrection(inches, Drive.DIRECTION.BACK);
            }

            if (gamepad1.b) {
                testCorrection(inches, Drive.DIRECTION.LEFT);
            }

            if (gamepad1.y) {
                testCorrection(inches, Drive.DIRECTION.RIGHT);
            }
        }
    }

    private double limitAcceleration(double speed, double lastSpeed, double currentTime, double lastTime) {

        double ACCELERATION_TIME = (1000 * 1.5);   // 1.5 second to accelerate to full speed
        double DECELERATION_TIME = (1000 * 1.0);   // 1 second to come to full stop

        double accelerationPerMS = (MAX_SPEED - MIN_SPEED) / ACCELERATION_TIME;
        double decelerationPerMS = (MAX_SPEED - MIN_SPEED) / DECELERATION_TIME;

        double deltaSpeed = speed - lastSpeed;
        double deltaTime = currentTime - lastTime;
        double acceleration = deltaSpeed / deltaTime;

        if ((deltaSpeed > 0) && acceleration > accelerationPerMS)
            return lastSpeed + (accelerationPerMS * deltaTime);

        if ((deltaSpeed < 0) && (Math.abs(acceleration) > (decelerationPerMS * deltaTime)))
            return  lastSpeed - (decelerationPerMS * deltaTime);

        return speed;
    }

    public void newDrive() {

        telemetry.addData("left front power", "%4.2f", leftFrontPower);
        telemetry.addData("Right front power", "%4.2f", rightFrontPower);
        telemetry.addData("left rear power", "%4.2f", leftRearPower);
        telemetry.addData("Right rear power", "%4.2f", rightRearPower);
        telemetry.update();

        while (opModeIsActive()) {

            // ToDo remove, emergency stop for testing
            if (gamepad1.back) {
                requestOpModeStop();
                break;
            }

            // Left stick to go forward back and strafe. Right stick to rotate. Left trigger accelerate.
            Gamepad gamepad = gamepad1;
            double x = gamepad.left_stick_x;
            double y = -gamepad.left_stick_y;
            double turn = gamepad.right_stick_x;
            double noise = 0.01;

            // Is either stick being used
            if (Math.abs(x) > noise || Math.abs(y) > noise || Math.abs(turn) > noise ) {

                double heading = Math.atan2(-x, y);     // same format as the gyro
                double angle = Math.atan2(y, x);
                double sin = Math.sin(angle - (Math.PI / 4));
                double cos = Math.cos(angle - (Math.PI / 4));
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                double power = Math.hypot(x, y);
                if (power != 0)
                    power = Math.pow(Math.abs(Math.min(power, 1)), 3) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
                if (turn != 0)
                    turn = Math.pow(Math.abs(Math.min(turn, 1)), 3) * (MAX_ROTATE_SPEED - MIN_ROTATE_SPEED) + MIN_ROTATE_SPEED;

                leftFrontPower  = power * (cos/max) + turn;
                rightFrontPower = power * (sin/max) - turn;
                leftRearPower   = power * (sin/max) + turn;
                rightRearPower  = power * (cos/max) - turn;

                double scale = power + Math.abs(turn);
                if (scale > 1) {
                    leftFrontPower  /= scale;
                    rightFrontPower /= scale;
                    leftRearPower   /= scale;
                    rightRearPower  /= scale;
                }

                //drive.leftFrontDrive.setPower(leftFrontPower);
                //drive.rightFrontDrive.setPower(rightFrontPower);
                //drive.leftBackDrive.setPower(leftRearPower);
                //drive.rightBackDrive.setPower(rightRearPower);

                Logger.message("%s",
                    String.format("x: %5.2f  y: %5.2f  turn: %5.2f ", x, y, turn) +
                    String.format("angle: %5.2f (rad)  %4.0f (deg)  ", angle, Math.toDegrees(angle)) +
                    String.format("heading: %4.0f   ", Math.toDegrees(heading)) +
                    String.format("power: %4.2f  sin: %5.2f  cos: %5.2f   ", power, sin, cos) +
                    String.format("power: %5.2f  %5.2f  %5.2f  %5.2f", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower)
                );

                sleep(250);
            } else {
                leftFrontPower  = 0;
                rightFrontPower = 0;
                leftRearPower   = 0;
                rightRearPower  = 0;

                drive.stopRobot();
            }

            telemetry.addData("left front power", "%4.2f", leftFrontPower);
            telemetry.addData("Right front power", "%4.2f", rightFrontPower);
            telemetry.addData("left rear power", "%4.2f", leftRearPower);
            telemetry.addData("Right rear power", "%4.2f", rightRearPower);
            telemetry.update();
        }
    }


    public void runTest() {

        // ToDo remove
        boolean running = true;
        boolean driving = false;
        Drive.DIRECTION lastDirection = Drive.DIRECTION.STOOPED;
        double MIN_ROTATE_SPEED = 0.25;
        double MAX_ROTATE_SPEED = 0.50;
        telemetry.addData("Speed", "%4.2f", speed);
        telemetry.addData("SpeedX", "%4.2f", speedX);
        telemetry.addData("SpeedY", "%4.2f", speedY);
        telemetry.addData("SpeedYaw", "%4.2f", speedYaw);
        telemetry.update();


        while (!isStarted()) Thread.yield();
        Logger.message("robot drive thread started");

        ElapsedTime driveTime = new ElapsedTime();
        double lastTime = driveTime.milliseconds();
        double lastSpeed = 0;

        double maxStick = 0;
        while (running && opModeIsActive()) {

            // ToDo remove, emergency stop for testing
            if (gamepad1.back) {
                requestOpModeStop();
                break;
            }

            // Left stick to go forward back and strafe. Right stick to rotate. Left trigger accelerate.
            Gamepad gamepad = gamepad1;
            double x = -gamepad.left_stick_y;
            double y = -gamepad.left_stick_x;
            double yaw = -gamepad.right_stick_x;
            double noise = 0.01;
            speedX = 0;
            speedY = 0;
            speed = 0;

            // Is either stick being used
            if (Math.abs(x) > noise || Math.abs(y) > noise || Math.abs(yaw) > noise ) {
                Drive.DIRECTION direction;

                // Moving forward or backward
                if (Math.abs(x) > noise && Math.abs(y) <= noise && Math.abs(yaw) <= noise) {
                    if (x > 0)
                        direction = Drive.DIRECTION.FORWARD;
                    else
                        direction = Drive.DIRECTION.BACK;
                    speed = Math.pow(Math.abs(x), 3) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;

                // Moving left or right
                } else if (Math.abs(y) > noise && Math.abs(x) <= noise && Math.abs(yaw) <= noise) {
                    if (y > 0)
                        direction = Drive.DIRECTION.LEFT;
                    else
                        direction = Drive.DIRECTION.RIGHT;
                    speed = Math.pow(Math.abs(y), 3) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;

                // Turning
                } else if (Math.abs(yaw) > noise && Math.abs(x) <= noise && Math.abs(y) <= noise) {
                    if (yaw > 0)
                        direction =  Drive.DIRECTION.TURN_LEFT;
                    else
                        direction = Drive.DIRECTION.TURN_RIGHT;
                    speed = Math.pow(Math.abs(yaw), 3) * (MAX_ROTATE_SPEED - MIN_ROTATE_SPEED) + MIN_ROTATE_SPEED;

                } else {
                    direction =  Drive.DIRECTION.DRIVER;
                    int count = 0;
                    if (Math.abs(x) > noise) {
                        speedX = Math.pow(Math.abs(x), 3) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
                        count++;
                    }
                    if (Math.abs(y) > noise) {
                        speedY = Math.pow(Math.abs(y), 3) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
                        count++;
                    }
                    if (Math.abs(yaw) > noise) {
                        speedYaw = Math.pow(Math.abs(yaw), 3) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED;
                        count++;
                    }
                    speedX /= count;
                    speedY /= count;
                    speedYaw /= count;
                    if (speedX > 0  && x < 0) speedX = -speedX;
                    if (speedY > 0  && y < 0) speedY = -speedY;
                    if (speedYaw > 0  && yaw < 0) speedYaw = -speedY;
                }


                double angle = Math.atan2(x, y);
                double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
                double power = Math.sin(angle - (Math.PI/4));
                Logger.message("x: %6.2ff  y: %6.2f  angle: %f  %f   magnitude: %4.2f  power: %f",
                        x, y, angle, Math.toDegrees(angle), magnitude, power);


                double currentTime = driveTime.milliseconds();

                if (direction == Drive.DIRECTION.DRIVER) {
                    //drive.moveRobot(speedX, speedY, speedYaw);
                } else {
                    // limit acceleration and deceleration to prevent skidding.
                    speed = limitAcceleration(speed, lastSpeed, currentTime, lastTime);
                    lastSpeed = speed;
                    //drive.moveRobot(direction, speed);
                }
                lastTime = currentTime;

                driving = true;
                lastDirection = direction;
                /*
                Logger.message("%-12s   x: %6.2f  y: %6.2f  yaw: %6.2f  speed: %6.2f  speedX: %6.2f  speedY: %6.2f  speedYaw: %6.2f",
                        direction, x , y, yaw, speed, speedX, speedY, speedYaw);

                 */

            } else if (driving) {
                drive.stopRobot();
                lastDirection = Drive.DIRECTION.STOOPED;
                driving = false;

            } else {
                Thread.yield();
            }

            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("SpeedX", "%4.2f", speedX);
            telemetry.addData("SpeedY", "%4.2f", speedY);
            telemetry.addData("SpeedYaw", "%4.2f", speedYaw);
            telemetry.update();
        }
        Logger.message("robot drive thread stopped");
    }




    private void testCorrection(double inches, Drive.DIRECTION direction) {

        // Disable drift correction and test how straight the drivetrain drives.
        /*
        double coefficient = drive.getDriftCoefficient();
        drive.setDriftCoefficient(0);
        drive.resetOrientation();
        drive.moveDistance(Drive.DIRECTION.FORWARD, speed, inches, 0 );
        drive.setDriftCoefficient(coefficient);
        telemetry.addData("Yaw:", "%6.2f", drive.getOrientation() );

        telemetry.update();
        sleep(5000);
        drive.moveDistance(Drive.DIRECTION.BACK, speed, inches, 0 );
        */

        drive.resetOrientation();
        drive.moveDistanceWithPIDControl (direction, speed, inches, 0);


        telemetry.addData("Yaw:", "%6.2f", drive.getOrientation() );
        telemetry.update();
    }

    private void validateConfig () {
        // Check if the values input in the FTC Dashboard are valid.
        speed = Math.min(Math.max(speed, 0), 1);
        inches = Math.min(Math.max(inches, 0), 120);
        timeout = Math.max(timeout, 0);
    }

    private void testPID() {

        double PID_DRIVE_KP = 0.02;
        double PID_DRIVE_KI = 0;
        double PID_DRIVE_KD = 0;

        PIDController pidDrive = new PIDController(PID_DRIVE_KP, PID_DRIVE_KI, PID_DRIVE_KD);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(PID_DRIVE_KP*3, PID_DRIVE_KP*3);   //
        pidDrive.setInputRange(-12, 12);
        pidDrive.enable();

        double correction = pidDrive.performPID(-0.5);

        Logger.message("correction: %6.3f", correction);

    }

    private void checkAccuracy () {

        ElapsedTime runtime = new ElapsedTime();

        for (double power=0.2; power<=0.801; power+=0.05) {
            if (! opModeIsActive()) break;

            drive.resetEncoders();
            drive.moveRobot(power, 0, 0);

            runtime.reset();
            while (runtime.milliseconds() < 10000 && opModeIsActive()) {
                sleep(1);
            }

            //while (gamepad1.a) sleep(100);
            drive.stopRobot();
            sleep(1000);

            int leftFrontPos = Math.abs(drive.leftFrontDrive.getCurrentPosition());
            int rightFrontPos = Math.abs(drive.rightFrontDrive.getCurrentPosition());
            int leftBackPos = Math.abs(drive.leftBackDrive.getCurrentPosition());
            int rightBackPos = Math.abs(drive.rightBackDrive.getCurrentPosition());
            int maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

            double COUNTS_PER_MOTOR_REV = 384.5;           // Gobilda Yellow Jacket Motor 5203-2402-0001
            double WHEEL_DIAMETER_INCHES = (96 / 25.4);    // 96 mm wheels converted to inches
            double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

            Logger.message("\n");
            Logger.message("power %5.2f", power);
            Logger.message("left front  %6d    %5.2f    %6.3f", leftFrontPos,  (leftFrontPos / COUNTS_PER_INCH),  (((double)leftFrontPos  / maxPos) - 1) * 100);
            Logger.message("right front %6d    %5.2f    %6.3f", rightFrontPos, (rightFrontPos / COUNTS_PER_INCH), (((double)rightFrontPos / maxPos) - 1) * 100);
            Logger.message("left rear   %6d    %5.2f    %6.3f", leftBackPos,   (leftBackPos / COUNTS_PER_INCH),   (((double)leftBackPos   / maxPos) - 1) * 100);
            Logger.message("right rear  %6d    %5.2f    %6.3f", rightBackPos,  (rightBackPos / COUNTS_PER_INCH),  (((double)rightBackPos  / maxPos) - 1) * 100);
        }
    }

    private void checkVelocity() {

        for (double power = 0.20; power <= 0.25; power += 0.001) {
            drive.moveRobot(power, 0, 0);
            Logger.message("\n");
            for (int i = 0; i<10; i++) {
                sleep(100);
                Logger.message("power: %5.3f   velocity: %6.1f  %6.1f",
                        power ,
                        drive.leftFrontDrive.getVelocity(),
                        drive.rightFrontDrive.getVelocity());
            }
        }
        drive.stopRobot();
    }

    private void characterizeVelocity() {

        for (double power = 0.20; power <= 0.9; power += 0.01)
        {
            drive.leftFrontDrive.setPower(power);
            velocity = drive.leftFrontDrive.getVelocity();
            telemetry.addData("Velocity", "%8.2f", velocity);
            telemetry.update();
            sleep(100);
        }
        drive.leftFrontDrive.setPower(0);
    }

    private void recordEncoders() {
        FileOutputStream fos;
        DataOutputStream dos;
        double[] dbuf = {65.56,66.89,67.98,68.82,69.55,70.37};

        try {
            String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), "/temp/drive.txt");
            Logger.message (path);

            fos = new FileOutputStream(path);
            dos = new DataOutputStream(fos);

            // for each byte in the buffer
            for (double d:dbuf) {
                // write double to the data output stream
                dos.writeDouble(d);
            }

            // force bytes to the underlying stream
            dos.flush();
            dos.close();

            fos.close();

        } catch(Exception e) {
            e.printStackTrace();

        }
    }

    private void readEncoders() {
        InputStream is;
        DataInputStream dis;

        try {
             // create file input stream
            String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), "/temp/drive.txt");

            is = new FileInputStream(path);

            // create new data input stream
            dis = new DataInputStream(is);

            // read till end of the stream
            while(dis.available()>0) {

                // read double
                double c = dis.readDouble();

                // print
                Logger.message("%f", c);
            }

            is.close();
            dis.close();

        } catch(Exception e) {
            // if any I/O error occurs
            e.printStackTrace();
        }
    }
}
