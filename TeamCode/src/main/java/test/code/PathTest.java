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
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@TeleOp(name="* Path Test", group="Test")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class PathTest extends LinearOpMode {

    public static double TARGET_X = 20;
    public static double TARGET_Y = 0;

    public static double DISTANCE_TOLERANCE_HIGH_SPEED = 10;
    public static double DISTANCE_TOLERANCE_LOW_SPEED = 0.5;

    public static double HEADING_TOLERANCE_HIGH_SPEED = 10;
    public static double HEADING_TOLERANCE_LOW_SPEED = 0.5;

    public static double TURN_TOLERANCE = 0.5;

    public static double MAX_SPEED = 0.5;

    final static double TURN_SPEED = 0.25;
    final static double TURN_MIN_SPEED = 0.10;
    final static double TURN_MAX_SPEED = 0.95;

    final double TURN_RAMP_UP_TIME = 1000;                       // turn ramp up time in milliseconds
    final double TURN_RAMP_DOWN_DEGREES = 5;

    public static double speed = TURN_SPEED;
    private double heading;

    private Telemetry.Item speedMsg;

    private Drive drive;
    Gyro gyro;
    Localizer localizer;
    private SparkFunOTOS otos;

    public static CustomPIDFCoefficients driveHighSpeedPIDFCoefficients = new CustomPIDFCoefficients(
            0.3, 0, 0, 0);

    public static CustomPIDFCoefficients driveLowSpeedPIDFCoefficients = new CustomPIDFCoefficients(
            0.12, 0, 0, 0);

    public static CustomPIDFCoefficients headingHighSpeedPIDFCoefficients = new CustomPIDFCoefficients(
            1, 0, 0, 0);

    public static CustomPIDFCoefficients headingLowSpeedPIDFCoefficients = new CustomPIDFCoefficients(
            1, 0, 0, 0);

    public static CustomPIDFCoefficients turnPIDFCoefficients = new CustomPIDFCoefficients(
            0.55, 0, 0, 0);

    PIDFController drivePID  = new PIDFController(driveHighSpeedPIDFCoefficients);
    PIDFController headingPID = new PIDFController(headingHighSpeedPIDFCoefficients);
    PIDFController turnPID = new PIDFController(turnPIDFCoefficients);

    private double distanceTolerance;
    private double headingTolerance;

    @Override
    public void runOpMode() {

        try {
            initialize();
            waitForStart();

            runDriveToCoordinateTest();

            //velocityDeceleration();
            //runTurnTest();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    private void initialize() {
        gyro = new Gyro(hardwareMap, "imu");
        drive = new Drive(this);
        drive.start();

        localizer = new OTOSLocalizer(hardwareMap);
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        heading = 0;

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);
    }


    private void runDriveToCoordinateTest() {

        while (opModeIsActive()) {

            if (gamepad1.a) {
                localizer.setPose(new Pose(0, 0, Math.toRadians(90)));
                moveToCoordinate(20, 0,     90,2000);
                while (gamepad1.a) sleep(10);
                sleep(3000);
                displayPose();
            }

            if (gamepad1.b) {
                moveToCoordinate(20, 20, 10,2000);
                while (gamepad1.b) sleep(10);
                displayPose();
            }

            if (gamepad1.y) {
                moveToCoordinate(20, 0, 0,2000);
                while (gamepad1.y) sleep(10);
                displayPose();
            }

            if (gamepad1.x) {
                moveToCoordinate(20, -20, 0,2000);
                sleep(3000);
                displayPose();
                while (gamepad1.x) sleep(10);
            }
        }
    }

    void printAngle (double x, double y) {
        double angle = Math.atan2(x, y);
        double sin = Math.sin(angle - (Math.PI / 4));
        double cos = Math.cos(angle - (Math.PI / 4));

        Logger.message("x: %3.0f  y:%3.0f angle: %5.2f (rad) %5.0f (deg)   sin: %5.2f   cos: %5.2f", x, y, angle, Math.toDegrees(angle), sin, cos);
    }

    private void runTurnTest() {

        telemetry.clear();

        speedMsg = telemetry.addData("Speed", "%4.2f", speed);
        Telemetry.Item yawGyroMsg = telemetry.addData("yaw gyro", 0);

        telemetry.addData("\nControls", "\r\n" +
                "  dpad up - increase speed\n" +
                "  dpad down - decrease speed\n" +
                "  y - turn north\n" +
                "  b - turn east\n" +
                "  a - turn south\n" +
                "  x - turn west\n" +
                "\n");

        Increment speedIncrement = new Increment(0.01, 0.05, 0.1);

        while (opModeIsActive()) {

            if (gamepad1.y) {
                heading = 0;
                turnToWithPID(heading);
                while (gamepad1.y) sleep(10);

            } else if (gamepad1.b) {
                heading = -90;
                turnToWithPID(heading);
                while (gamepad1.b) sleep(10);

            } else if (gamepad1.a) {
                heading = 180;
                turnToWithPID(heading);
                while (gamepad1.a) sleep(10);

            } else if (gamepad1.x) {
                heading = 90;
                turnToWithPID(heading);
                while (gamepad1.x) sleep(10);

            } else if (gamepad1.dpad_up) {
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
            yawGyroMsg.setValue("%6.2f target: %6.2f %5.2f", gyro.getYaw(), heading, Math.toDegrees(localizer.getPose().getHeading()));
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

    private void setDisplaySpeed() {
        speedMsg.setValue("%4.2f", speed);
    }

    /**
     * Calculate to turn speed. The turn speed to ramped up based on time, and ramped down based on
     * degrees remaining to turn.
     *
     * @param startTime   time in millisecond of the start of the turn
     * @param speed       speed of the turn (0-1)
     * @param degreesToGo degrees remaining to turn
     * @return motor power (0-1)
     */
    private double getRampedPower(double startTime, double speed, double degreesToGo) {

        double speedRange = Math.max(Math.abs(speed) - TURN_MIN_SPEED, 0);
        double ramUp = (startTime / TURN_RAMP_UP_TIME) * speedRange + TURN_MIN_SPEED;
        double ramDown = (Math.pow(degreesToGo, 2) / Math.pow(TURN_RAMP_DOWN_DEGREES, 2)) * speedRange + TURN_MIN_SPEED;
        return Math.min(Math.min(ramUp, ramDown), speed);
    }

    public void turnToWithPID(double heading) {

        double startHeading = localizer.getPose().getHeading();
        double lastHeading = startHeading;
        double currentHeading;
        double toTurn;
        double velocity;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double lastTime = 0;
        double currentTime = 0;

        double targetHeading = Math.toRadians(heading);
        double maxVelocity = drive.getMaxVelocity();

        // Run a full speed until we near the deceleration threshold.
        velocity = TURN_MAX_SPEED * maxVelocity;
        double degreesOfDeceleration = velocity / maxVelocity * 35;
        toTurn = angleWrap(targetHeading - startHeading);
        if (Math.abs(Math.toDegrees(toTurn)) > degreesOfDeceleration) {
            drive.setVelocity(-velocity, velocity, -velocity, velocity);
            while (opModeIsActive() && Math.abs(Math.toDegrees(toTurn)) > degreesOfDeceleration) {
                currentHeading = localizer.getPose().getHeading();
                toTurn = angleWrap(targetHeading - currentHeading);
                Logger.message("  to turn: %-6.1f  deceleration: %-6.1f", Math.toDegrees(toTurn),degreesOfDeceleration);
            }
        }

        turnPID.reset();
        while (opModeIsActive()) {

            currentHeading = localizer.getPose().getHeading();
            currentTime = timer.milliseconds();

            toTurn = angleWrap(targetHeading - currentHeading);
            turnPID.updateError(toTurn);
            double error = turnPID.runPIDF();

            double sign = MathFunctions.getSign(error);
            double power = MathFunctions.clamp(Math.abs(error), TURN_MIN_SPEED, TURN_MAX_SPEED);
            velocity = power * maxVelocity;

            drive.leftFrontDrive.setVelocity(-velocity * sign);
            drive.rightFrontDrive.setVelocity(velocity * sign);
            drive.leftBackDrive.setVelocity(-velocity * sign);
            drive.rightBackDrive.setVelocity(velocity * sign);

            double degreesTurned = angleWrap(currentHeading - startHeading);
            degreesOfDeceleration = velocity / maxVelocity * 30;

            // calculate turn velocity in degrees per second
            double velocityInDegrees = Math.toDegrees(angleWrap(currentHeading - lastHeading) / (currentTime - lastTime) * 1000);

            Logger.message(
                            String.format("  target: %-6.1f", Math.toDegrees(targetHeading)) +
                            String.format("  start: %-6.1f", Math.toDegrees(startHeading)) +
                            String.format("  curr: %-6.1f", Math.toDegrees(currentHeading)) +
                            String.format("  to turn: %-6.1f", Math.toDegrees(toTurn)) +
                            String.format("  turned: %-6.1f", Math.toDegrees(degreesTurned)) +
                            String.format("  last: %-6.1f", Math.toDegrees(lastHeading)) +
                            String.format("  velocity: %-6.1f %-6.1f (ticks)", velocity, drive.leftFrontDrive.getVelocity()) +
                            String.format("  %-6.1f (deg)", velocityInDegrees) +
                            String.format("  deceleration: %-6.1f", degreesOfDeceleration) +
                            String.format("  time: %6.2f", currentTime-lastTime) +
                            String.format("  power: %4.2f", power) +
                            String.format("  error: %5.2f", error) +
                            String.format("  time: %4.2f", timer.seconds()));

            if (Math.abs(toTurn) < Math.toRadians(TURN_TOLERANCE) && velocityInDegrees <=  20 ) {
                break;
            }

            if (timer.seconds() > 2) {
                Logger.message("timeout");
                break;
            }
            lastTime = currentTime;
            lastHeading = currentHeading;
        }

        drive.stopRobot();
        drive.leftFrontDrive.setVelocity(0);
        drive.rightFrontDrive.setVelocity(0);
        drive.leftBackDrive.setVelocity(0);
        drive.rightBackDrive.setVelocity(0);

        double seconds = timer.seconds();
        sleep(2000);
        Logger.message("turn ends at %5.1f  time: %5.2f  velocity tolerance %5.0f",
                Math.toDegrees(localizer.getPose().getHeading()),seconds, maxVelocity * TURN_MIN_SPEED );
    }

    public void moveToCoordinate(double targetX, double targetY, double targetHeading, double timeout) {

        long start = System.currentTimeMillis();

        headingPID.setCoefficients(headingHighSpeedPIDFCoefficients);
        drivePID.setCoefficients(driveHighSpeedPIDFCoefficients);
        distanceTolerance = DISTANCE_TOLERANCE_HIGH_SPEED;
        headingTolerance = HEADING_TOLERANCE_HIGH_SPEED;
        moveTo(targetX, targetY, targetHeading, timeout);

        headingPID.setCoefficients(headingLowSpeedPIDFCoefficients);
        drivePID.setCoefficients(driveLowSpeedPIDFCoefficients);
        distanceTolerance = DISTANCE_TOLERANCE_LOW_SPEED;
        headingTolerance = HEADING_TOLERANCE_LOW_SPEED;
        moveTo(targetX, targetY, targetHeading, timeout);

        Logger.message("time: %,d milliseconds", System.currentTimeMillis() - start);
    }

        /**
         * Move to the specified coordinate and heading
         *
         * @param targetX
         * @param targetY
         * @param targetHeading desired heading in degrees (0 - 360)
         * @param timeout timeout in milliseconds, 0 for no timeout
         */
    public void moveTo(double targetX, double targetY, double targetHeading, double timeout) {

        ElapsedTime timer = new ElapsedTime();

        Logger.message("to %3.0f, %3.0f, %4.0f", targetX, targetY, targetHeading);

        double maxVelocity = drive.getMaxVelocity() * MAX_SPEED;

        timer.reset();
        drivePID.reset();
        headingPID.reset();

        // Looping until we move the desired distance
        while (opModeIsActive()) {

            Pose pose = localizer.getPose();
            SparkFunOTOS.Pose2D rawPose = otos.getPosition();

            double currentX = pose.getX();
            double currentY = -pose.getY();
            double currentHeading = pose.getHeading();
            double a = targetX - currentX;
            double b = targetY - currentY;
            double angle = Math.atan2(a, b);
            double distance = Math.hypot(a, b);
            double sin = Math.sin(angle - (Math.PI / 4));
            double cos = Math.cos(angle - (Math.PI / 4));
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double headingError = angleWrap(currentHeading - Math.toRadians(targetHeading));
            headingPID.updateError(headingError);
            double turn = headingPID.runPIDF();

            // If the heading error is greater than 45 degrees then just turn.
            double power = 0;
            if (Math.abs(headingError) < Math.toDegrees(45))  {
                drivePID.updateError(distance);
                power = drivePID.runPIDF();
            }

            double scale = 1;
            if (power != 0 && (power + Math.abs(turn) > MAX_SPEED))
                scale = (power + Math.abs(turn)) / MAX_SPEED;

            double leftFrontPower  = (power * (cos / max) + turn) / scale;
            double rightFrontPower = (power * (sin / max) - turn) / scale;
            double leftRearPower   = (power * (sin / max) + turn) / scale;
            double rightRearPower  = (power * (cos / max) - turn) / scale;

            drive.leftFrontDrive.setVelocity(leftFrontPower * maxVelocity);
            drive.rightFrontDrive.setVelocity(rightFrontPower * maxVelocity);
            drive.leftBackDrive.setVelocity(leftRearPower * maxVelocity);
            drive.rightBackDrive.setVelocity(rightRearPower * maxVelocity);

            Logger.message("%s",
                    String.format("x: %-5.1f y: %-5.1f h: %-5.1f  ", rawPose.x, rawPose.y, Math.toDegrees(rawPose.h)) +
                    String.format("x: %-5.1f y: %-5.1f h: %-5.1f  ", currentX, currentY, Math.toDegrees(currentHeading)) +
                    String.format("a: %5.2f b: %5.2f  distance: %5.2f  turn: %5.2f  ", a, b, distance, turn) +
                    String.format("angle: %5.2f (rad)  %4.0f (deg)  ", angle, Math.toDegrees(angle)) +
                    String.format("heading: %6.1f %6.1f  ", Math.toDegrees(currentHeading), Math.toDegrees(headingError)) +
                    String.format("power: %4.2f  sin: %5.2f  cos: %5.2f  ", power, sin, cos) +
                    String.format("power: %5.2f  %5.2f  %5.2f  %5.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                    String.format("derivative %6.2f", drivePID.errorDerivative));

            if (Math.abs(distance) < distanceTolerance && Math.toDegrees(headingError) < headingTolerance)
                break;

            if (timeout > 0 && timer.milliseconds() >= timeout) {
                Logger.message("moveDistance timed out");
                break;
            }
        }

        drive.stopRobot();
    }
    public void turnTo(double heading) {

        ElapsedTime timer = new ElapsedTime();
        double lastTime = 0;

        double startHeading = localizer.getPose().getHeading();
        double lastHeading = startHeading;
        double targetHeading = Math.toRadians(heading);
        double maxVelocity = drive.getMaxVelocity();

        turnPID.reset();
        timer.reset();

        while (opModeIsActive()) {

            double currentHeading = localizer.getPose().getHeading();
            double currentTime = timer.milliseconds();

            double toTurn = angleWrap(targetHeading - currentHeading);
            turnPID.updateError(toTurn);
            double error = turnPID.runPIDF();

            double sign = MathFunctions.getSign(error);
            double power = MathFunctions.clamp(Math.abs(error), TURN_MIN_SPEED, TURN_MAX_SPEED);
            double velocity = power * maxVelocity;

            drive.leftFrontDrive.setVelocity(-velocity * sign);
            drive.rightFrontDrive.setVelocity(velocity * sign);
            drive.leftBackDrive.setVelocity(-velocity * sign);
            drive.rightBackDrive.setVelocity(velocity * sign);

            double degreesTurned = angleWrap(currentHeading - startHeading);
            double degreesOfDeceleration = velocity / maxVelocity * 30;

            // calculate turn velocity in degrees per second
            double velocityInDegrees = Math.toDegrees(angleWrap(currentHeading - lastHeading) / (currentTime - lastTime) * 1000);

            Logger.message(
                    String.format("  target: %-6.1f", Math.toDegrees(targetHeading)) +
                            String.format("  start: %-6.1f", Math.toDegrees(startHeading)) +
                            String.format("  curr: %-6.1f", Math.toDegrees(currentHeading)) +
                            String.format("  to turn: %-6.1f", Math.toDegrees(toTurn)) +
                            String.format("  turned: %-6.1f", Math.toDegrees(degreesTurned)) +
                            String.format("  last: %-6.1f", Math.toDegrees(lastHeading)) +
                            String.format("  velocity: %-6.1f %-6.1f (ticks)", velocity, drive.leftFrontDrive.getVelocity()) +
                            String.format("  %-6.1f (deg)", velocityInDegrees) +
                            String.format("  deceleration: %-6.1f", degreesOfDeceleration) +
                            String.format("  time: %6.2f", currentTime-lastTime) +
                            String.format("  power: %4.2f", power) +
                            String.format("  error: %5.2f", error) +
                            String.format("  time: %4.2f", timer.seconds()));

            if (Math.abs(toTurn) < Math.toRadians(TURN_TOLERANCE) && velocityInDegrees <=  20 ) {
                break;
            }

            if (timer.seconds() > 2) {
                Logger.message("timeout");
                break;
            }
            lastTime = currentTime;
            lastHeading = currentHeading;
        }

        drive.stopRobot();
        drive.leftFrontDrive.setVelocity(0);
        drive.rightFrontDrive.setVelocity(0);
        drive.leftBackDrive.setVelocity(0);
        drive.rightBackDrive.setVelocity(0);

        double seconds = timer.seconds();
        sleep(2000);
        Logger.message("turn ends at %5.1f  time: %5.2f  velocity tolerance %5.0f",
                Math.toDegrees(localizer.getPose().getHeading()),seconds, maxVelocity * TURN_MIN_SPEED );
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    private void displayPose() {

        Pose pose = localizer.getPose();
        SparkFunOTOS.Pose2D rawPose = otos.getPosition();

        String str1 = String.format("x %5.1f  y %5.1f  heading %5.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        String str2 = String.format("x %5.1f  y %5.1f  heading %5.1f", rawPose.x, rawPose.y, Math.toDegrees(rawPose.h));

        telemetry.addData("pose", str1);
        telemetry.addData("rawPose", str2);
        telemetry.update();

        Logger.message("pose: %s  raw pose %s", str1, str2);
    }

    private void velocityDeceleration () {

        ElapsedTime timer = new ElapsedTime();

        for (double i = 10; i <=90; i+=10) {

            double velocity = drive.getMaxVelocity() * (i / 100);
            //Logger.message("%f  %f", i, velocity);

            drive.leftFrontDrive.setVelocity(-velocity);
            drive.rightFrontDrive.setVelocity(velocity);
            drive.leftBackDrive.setVelocity(-velocity);
            drive.rightBackDrive.setVelocity(velocity);

            timer.reset();
            while (opModeIsActive() && timer.milliseconds() < 1000) {
                SparkFunOTOS.Pose2D rawPose = otos.getPosition();

                SparkFunOTOS.Pose2D pose = otos.getVelocity();
                Logger.message("velocity: %5.0f  %5.0f  %5.0f  %5.0f  %f",
                    drive.leftFrontDrive.getVelocity(),
                    drive.rightFrontDrive.getVelocity(),
                    drive.leftBackDrive.getVelocity(),
                    drive.rightBackDrive.getVelocity(),
                    pose.h);


                continue;
            }

            drive.stopRobot();
            double zeroPowerHeading = localizer.getPose().getHeading();
            double zeroPowerTime = timer.milliseconds();

            while (opModeIsActive()) {
                if (drive.leftFrontDrive.getVelocity() == 0 &&
                        drive.rightFrontDrive.getVelocity() == 0 &&
                        drive.leftBackDrive.getVelocity() == 0 &&
                        drive.rightBackDrive.getVelocity() == 0) {
                    break;
                }
            }

            double stopTime = timer.milliseconds();

            sleep(1000);
            double heading = localizer.getPose().getHeading();

            double  deceleration = Math.toDegrees(angleWrap(heading - zeroPowerHeading));
            Logger.message(" %3.0f percent  time %4.0f  deceleration %6.1f", i, stopTime - zeroPowerTime, deceleration);
        }
    }

        /*
    Pose testPose = new Pose(0, 0, 0);
    ElapsedTime poseTimer = new ElapsedTime();
    double posePreviousTime = 0;

    private Pose getPose(double power) {
        if (testPose.getX() == 0 && testPose.getY() == 0) {
            poseTimer.reset();
            posePreviousTime = 0;
        }
        double currentTime = poseTimer.seconds();
        double delta = (currentTime - posePreviousTime);
        double x = testPose.getX() + (delta * power * 10);
        testPose.setX(x);
        posePreviousTime = currentTime;
        return testPose;
    }
     */

}
