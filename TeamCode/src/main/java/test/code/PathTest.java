/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package test.code;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

import common.Drive;
import common.Gyro;
import common.Logger;
import utils.Increment;

@TeleOp(name="Path Test", group="Test")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class PathTest extends LinearOpMode {

    public static double TARGET_X = 20;
    public static double TARGET_Y = 0;

    public static double DISTANCE_TOLERANCE = 0.5;

    final static double MAX_SPEED       = 0.9;
    final static double TURN_SPEED      = 0.25;
    final double TURN_MIN_SPEED         = 0.2;
    final double TURN_MAX_SPEED         = 0.9;

    final double TURN_RAMP_UP_TIME      = 1000;                       // turn ramp up time in milliseconds
    final double TURN_RAMP_DOWN_DEGREES = 5;

    public static double speed = TURN_SPEED;
    private double heading;

    private Telemetry.Item speedMsg;

    private Drive drive;
    Gyro gyro;
    Localizer localizer;

    // PID control fields
    public static double kP = 0.04;
    public static double kI = 0;
    public static double kD = 0;
    public static double minOutput = -1;
    public static double maxOutput = 1;

    private double lastError = 0;
    private double integral = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // Heading error PIDF coefficients
    public static CustomPIDFCoefficients headingPIDFCoefficients = new CustomPIDFCoefficients(
            0.6,
            0,
            0,
            0);

    public static CustomPIDFCoefficients drivePIDFCoefficients = new CustomPIDFCoefficients(
            0.6,
            0,
            0,
            0);

    PIDFController headingPID = new PIDFController(headingPIDFCoefficients);
    PIDFController drivePID = new PIDFController(drivePIDFCoefficients);


    @Override
    public void runOpMode() {

        try {
            initialize();
            waitForStart();
            runDriveToCoordinateTest();
            runTurnTest();
        } catch (Exception e)  {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    private void initialize () {
        gyro = new Gyro(hardwareMap, "imu");
        drive = new Drive(this);
        localizer = new OTOSLocalizer(hardwareMap);

        heading = 0;

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);
    }


    private void runDriveToCoordinateTest() {
        moveToCoordinate(TARGET_X, TARGET_Y, 5000);
        sleep(3000);
    }


    private void runTurnTest() {

        telemetry.clear();

        speedMsg =  telemetry.addData("Speed", "%4.2f", speed);
        Telemetry.Item yawGyroMsg =  telemetry.addData("yaw gyro", 0);

        telemetry.addData("\nControls", "\r\n" +
                "  dpad up - increase speed\n" +
                "  dpad down - decrease speed\n" +
                "  y - turn north\n" +
                "  a - turn east\n" +
                "  x - turn south\n" +
                "  b - turn west\n" +
                "\n");

        Increment speedIncrement = new Increment(0.01, 0.05, 0.1);

        while (opModeIsActive()) {
            if (gamepad1.y) {
                heading = 0;
                turnToWithPID(heading);
            }

            else if (gamepad1.b) {
                heading = 90;
                turnToWithPID(heading);
            }

            else if (gamepad1.a) {
                heading = 180;
                turnToWithPID(heading);
            }

            else if (gamepad1.x) {
                heading = -90;
                turnToWithPID(heading);
            }

            else if (gamepad1.dpad_up) {
                // increase the speed to travel
                speedIncrement.reset();
                while (gamepad1.dpad_up) {
                    speed = Math.min(speed + speedIncrement.get(), TURN_MAX_SPEED);
                    setDisplaySpeed();
                    telemetry.update();
                }

            } else if (gamepad1.dpad_down) {
                // decrease the speed the travel
                speedIncrement.reset();
                while (gamepad1.dpad_down) {
                    speed = Math.max(speed - speedIncrement.get(), TURN_MIN_SPEED);
                    setDisplaySpeed();
                    telemetry.update();
                }
            }

            setDisplaySpeed();
            yawGyroMsg.setValue("%6.2f (error: %6.2f)", gyro.getYaw(), heading);
            telemetry.update();

            displayDashboardTelemetry();

        }
    }

    private void displayDashboardTelemetry() {

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        dashboardTelemetry.addLine("  ");
        dashboardTelemetry.addLine("Controls");
        dashboardTelemetry.addLine("  a - turn east");
        dashboardTelemetry.addLine("  b - turn west");
        dashboardTelemetry.addLine("  x - turn south");
        dashboardTelemetry.addLine("  y - turn north");

        dashboardTelemetry.addData("1. speed", "%4.2f", speed);
        dashboardTelemetry.addData("3. yaw navx", "%6.2f (error: %6.2f)", gyro.getYaw(), heading - gyro.getYaw());

        dashboardTelemetry.update();
    }

    private void setDisplaySpeed () {
        speedMsg.setValue("%4.2f", speed);
    }

    /**
     * Calculate to turn speed. The turn speed to ramped up based on time, and ramped down based on
     * degrees remaining to turn.
     *
     * @param startTime time in millisecond of the start of the turn
     * @param speed speed of the turn (0-1)
     * @param degreesToGo degrees remaining to turn
     * @return motor power (0-1)
     */
    private double getRampedPower (double startTime, double speed, double degreesToGo) {

        double speedRange = Math.max(Math.abs(speed) - TURN_MIN_SPEED, 0);
        double ramUp = (startTime / TURN_RAMP_UP_TIME) * speedRange + TURN_MIN_SPEED;
        double ramDown = (Math.pow(degreesToGo, 2) / Math.pow(TURN_RAMP_DOWN_DEGREES, 2)) * speedRange + TURN_MIN_SPEED;
        return Math.min(Math.min(ramUp, ramDown), speed);
    }

    public void turnToWithPID(double targetHeading) {

        double startHeading = localizer.getPose().getHeading();
        double lastHeading = startHeading;
        double currentHeading;
        double toTurn;
        double velocity;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double lastTime = 0;
        double currentTime = 0;

        pidReset();

        while (opModeIsActive()) {

            currentHeading = localizer.getPose().getHeading();
            toTurn = (targetHeading - currentHeading);

            Drive.DIRECTION direction;
            if (toTurn > 0) {
                direction = Drive.DIRECTION.TURN_LEFT;
            } else if (toTurn < 0) {
                direction = Drive.DIRECTION.TURN_RIGHT;
            } else {
                return;
            }

            double power = pidControl(Math.toRadians(toTurn));
            double rampSpeed = getRampedPower(currentTime, power, Math.abs(toTurn));
            drive.moveRobot(direction, rampSpeed);

            currentTime = timer.milliseconds();
            velocity = angleWrap(currentHeading - lastHeading) / (currentTime - lastTime);
            lastTime = currentTime;
            lastHeading = currentHeading;

            double degreesTurned = angleWrap(currentHeading - startHeading);
            Logger.message(
                    String.format("%s", direction) +
                    String.format("  to: %-6.1f", targetHeading) +
                            String.format("  start: %-6.1f", startHeading) +
                            String.format("  target: %-6.1f", startHeading) +
                            String.format("  to turn: %-6.2f", toTurn) +
                            String.format("  turned: %-6.2f", degreesTurned) +
                            String.format("  velocity: %-6.4f", velocity) +
                            String.format("  curr: %-6.4f", currentHeading) +
                            String.format("  last: %-6.4f", lastHeading));

            if (toTurn > 0.5)
                break;

            if (timer.seconds() > 5)
                break;
        }

        drive.stopRobot();
    }

    /**
     *  Method to
     *
     * @param timeout timeout in milliseconds, 0 for no timeout
     */
    public void moveToCoordinate(double targetX, double targetY, double timeout) {

        Pose pose;
        double power = 0;
        double x, y;
        double currentX, currentY;
        ElapsedTime timer = new ElapsedTime();

        // Looping until we move the desired distance
        //double startHeading = localizer.getPose().getHeading();
        double startHeading = getPose(power).getHeading();
        timer.reset();
        drivePID.reset();
        headingPID.reset();
        drive.accelerationReset();
        while (opModeIsActive()) {

            //pose = localizer.getPose();
            pose = getPose(power);
            currentX = pose.getX();
            currentY = pose.getY();
            x = targetX - currentX;
            y = targetY - currentY;
            double angle = Math.atan2(x, y);
            double distance = Math.hypot(x, y);

            drivePID.updateError(distance);
            power = drivePID.runPIDF();

            double currentHeading = pose.getHeading();
            headingPID.updateError(startHeading - currentHeading);
            double turn = headingPID.runPIDF();

            drive.accelerationLimit(power);

            double sin = Math.sin(angle - (Math.PI / 4));
            double cos = Math.cos(angle - (Math.PI / 4));
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double scale = 1;
            if (power != 0 &&(power + Math.abs(turn) > MAX_SPEED))
                scale = (power + Math.abs(turn)) / MAX_SPEED;

            double leftFrontPower  = (power * (cos/max) + turn) / scale;
            double rightFrontPower = (power * (sin/max) - turn) / scale;
            double leftRearPower   = (power * (sin/max) + turn) / scale;
            double rightRearPower  = (power * (cos/max) - turn) / scale;

            drive.leftFrontDrive.setPower(leftFrontPower);
            drive.rightFrontDrive.setPower(rightFrontPower);
            drive.leftBackDrive.setPower(leftRearPower);
            drive.rightBackDrive.setPower(rightRearPower);

            Logger.message("%s",
                    String.format("x: %5.2f y: %5.2f  distance: %5.2f  turn: %5.2f  ", x, y, distance, turn) +
                    String.format("angle: %5.2f (rad)  %4.0f (deg)  ", angle, Math.toDegrees(angle)) +
                    String.format("heading: %6.2f %5.2f  ", startHeading, currentHeading) +
                    String.format("power: %4.2f  sin: %5.2f  cos: %5.2f  ", power, sin, cos) +
                    String.format("power: %4.2f  %4.2f  %4.2f  %4.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower));


            if (Math.abs(distance) < DISTANCE_TOLERANCE)
                break;

            if (timeout > 0 && timer.milliseconds() >= timeout) {
                Logger.message("moveDistance timed out");
                break;
            }
        }

        drive.stopRobot();
    }

    //private double turnDeceleration(double velocity) {   }
    Pose pose = new Pose(0, 0, 0);
    ElapsedTime poseTimer = new ElapsedTime();
    double posePreviousTime = 0;
    private Pose getPose(double power) {
        if (pose.getX() == 0 && pose.getY() == 0) {
            poseTimer.reset();
            posePreviousTime = 0;
        }
        double currentTime = poseTimer.seconds();
        double delta = (currentTime - posePreviousTime);
        double x = pose.getX() + (delta * power * 10);
        pose.setX(x);
        posePreviousTime = currentTime;
        return pose;
    }

    public void pidReset() {
        pidTimer.reset();
        lastError = 0;
        integral = 0;
    }

    public double pidControl (double error) {

        integral += error * pidTimer.seconds();
        double derivative = (error - lastError) / pidTimer.seconds();
        lastError = error;
        pidTimer.reset();

        double output = (error * kP) + (integral * kI) + (derivative * kD);
        output = MathFunctions.clamp(output, minOutput, maxOutput);

        return output;
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < - Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
