/*
 * Test code for threads
 */
package test.code;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;

import common.Drive;
import common.Logger;

@TeleOp(name="* Drive Test", group="Test")
@SuppressWarnings("unused")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class DriveTest extends LinearOpMode {

    public static double speed = 0.50;
    public static double inches = 96;
    public static double timeout = 0;

    double velocity;

    Drive drive  = null;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new Drive(this);
        drive.start();

        telemetry.addData("Status", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.clear();

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

            telemetry.update();

            if (gamepad1.y) {
                //drive.moveDistanceWithGyro(Drive.DIRECTION.FORWARD, speed, inches, timeout);
                while (gamepad1.y) {
                    drive.moveRobotWithPIDControl(Drive.DIRECTION.FORWARD, speed);
                    telemetry.addData("Drift", "%8.2f", drive.totalDrift);
                    telemetry.addData("Yaw:", "%6.2f", drive.getOrientation() );
                    telemetry.update();

                }
                drive.stopRobot();
            }

            if (gamepad1.a) {
                while (gamepad1.a) {
                    drive.moveRobotWithPIDControl(Drive.DIRECTION.BACK, speed);
                    telemetry.addData("Drift", "%8.2f", drive.totalDrift);
                    telemetry.addData("Yaw:", "%6.2f", drive.getOrientation() );
                    telemetry.update();
                }
                drive.stopRobot();
            }

            if (gamepad1.b) {
                testCorrection(inches);
            }
        }
    }

    private void testCorrection(double inches) {

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
        drive.moveDistanceWithPIDControl (Drive.DIRECTION.FORWARD, speed, inches, 0);
        telemetry.addData("Yaw:", "%6.2f", drive.getOrientation() );
        telemetry.update();
    }

    private void validateConfig () {
        // Check if the values input in the FTC Dashboard are valid.
        speed = Math.min(Math.max(speed, 0), 1);
        inches = Math.min(Math.max(inches, 0), 120);
        timeout = Math.max(timeout, 0);
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
