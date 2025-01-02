package common;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

@SuppressLint("DefaultLocale")

public class DriveControl extends Thread {

    public static double MAX_SPEED = 0.9;

    public static double DISTANCE_TOLERANCE_HIGH_SPEED = 10;
    public static double DISTANCE_TOLERANCE_LOW_SPEED = 0.5;

    public static double HEADING_TOLERANCE_HIGH_SPEED = 20;
    public static double HEADING_TOLERANCE_LOW_SPEED = 0.5;

    public static double VELOCITY_TOLERANCE_HIGH_SPEED = MAX_SPEED;
    public static double VELOCITY_TOLERANCE_LOW_SPEED = 0.10;

    public static CustomPIDFCoefficients driveHighSpeedPIDFCoefficients = new CustomPIDFCoefficients(
            0.3, 0, 0, 0);

    public static CustomPIDFCoefficients driveLowSpeedPIDFCoefficients = new CustomPIDFCoefficients(
            0.12, 0, 0, 0);

    public static CustomPIDFCoefficients headingHighSpeedPIDFCoefficients = new CustomPIDFCoefficients(
            2.5, 0, 0, 0);

    public static CustomPIDFCoefficients headingLowSpeedPIDFCoefficients = new CustomPIDFCoefficients(
            1, 0, 0, 0);

    PIDFController drivePID  = new PIDFController(driveHighSpeedPIDFCoefficients);
    PIDFController headingPID = new PIDFController(headingHighSpeedPIDFCoefficients);

    private double distanceTolerance;
    private double headingTolerance;
    private double velocityTolerance;
    private double timeout;
    private final ElapsedTime timeoutTimer;

    private double targetX;
    private double targetY;
    private double targetHeading;

    private double stopDistance;

    private double leftX;
    private double leftY;
    private double rightX;

    private boolean interruptAction = false;

    private boolean highSpeed;

    private enum DRIVE_STATE { IDLE, MOVING, MOVING_TO_COORDINATE, MOVING_TO_OBJECT }
    private DRIVE_STATE driveState;

    private final Drive drive;
    Localizer localizer;
    LinearOpMode opMode;

    public DriveControl(LinearOpMode opMode, Drive drive) {

        this.setName("driveControl");
        this.opMode = opMode;
        this.drive = drive;
        driveState = DRIVE_STATE.IDLE;
        timeoutTimer = new ElapsedTime();

        try {
            localizer = new OTOSLocalizer(opMode.hardwareMap);
        } catch (Exception e) {
            Logger.error(e, "Optical Tracking Odometry Sensor not found");
        }
    }

    /**
     * Control the motor on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("driveControl thread started for %s", this.getName());

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
            switch (driveState) {
                case IDLE:
                    Thread.yield();
                    continue;

                case MOVING:
                    moveWithJoystick();
                    continue;

                case MOVING_TO_COORDINATE:
                    moveToCoordinate();
                    driveState = DRIVE_STATE.IDLE;
                    continue;

                case MOVING_TO_OBJECT:
                    moveToObject();
                    driveState = DRIVE_STATE.IDLE;
            }
        }
        Logger.message("driveControl thread stopped");
    }

    private void moveInit (boolean highSpeed) {

        this.highSpeed = highSpeed;

        drivePID.reset();
        headingPID.reset();

        if (highSpeed) {
            headingPID.setCoefficients(headingHighSpeedPIDFCoefficients);
            drivePID.setCoefficients(driveHighSpeedPIDFCoefficients);
            distanceTolerance = DISTANCE_TOLERANCE_HIGH_SPEED;
            headingTolerance = HEADING_TOLERANCE_HIGH_SPEED;
            velocityTolerance = VELOCITY_TOLERANCE_HIGH_SPEED * drive.getMaxVelocity();
        } else {
            headingPID.setCoefficients(headingLowSpeedPIDFCoefficients);
            drivePID.setCoefficients(driveLowSpeedPIDFCoefficients);
            distanceTolerance = DISTANCE_TOLERANCE_LOW_SPEED;
            headingTolerance = HEADING_TOLERANCE_LOW_SPEED;
            velocityTolerance = VELOCITY_TOLERANCE_LOW_SPEED * drive.getMaxVelocity();
        }
    }

    /**
     * Move to the specified coordinate and heading
     */
    private void moveToCoordinate() {

        Logger.message("to %3.0f, %3.0f, %4.0f", targetX, targetY, targetHeading);

        moveInit(true);
        timeoutTimer.reset();

        // Looping until we move the desired distance
        while (opMode.opModeIsActive() && !interruptAction) {

            double maxVelocity = drive.getMaxVelocity() * MAX_SPEED;
            Pose pose = localizer.getPose();

            double currentX = pose.getX();
            double currentY = pose.getY();
            double currentHeading = pose.getHeading();
            double a = targetY - currentY;
            double b = targetX - currentX;
            double signedAngle = polarToSignedAngle(currentHeading);
            double angle = Math.atan2(a, b) - signedAngle;   // angle is robot relative
            double distance = Math.hypot(a, b);
            double sin = Math.sin(angle - (Math.PI / 4));
            double cos = Math.cos(angle - (Math.PI / 4));
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double headingError = angleWrap(currentHeading - Math.toRadians(targetHeading));

            headingPID.updateError(headingError);
            double turn = headingPID.runPIDF();

            drivePID.updateError(distance);
            double power = drivePID.runPIDF();

            // If the heading error is greater than 45 degrees then give the heading error greater weight
            if (Math.abs(headingError) < Math.toRadians(45))  {
                turn *= 2;
            }

            double scale = 1;
            if (power + Math.abs(turn) > MAX_SPEED)
                scale = (power + Math.abs(turn)) / MAX_SPEED;

            double leftFrontPower  = (power * (cos / max) + turn) / scale;
            double rightFrontPower = (power * (sin / max) - turn) / scale;
            double leftRearPower   = (power * (sin / max) + turn) / scale;
            double rightRearPower  = (power * (cos / max) - turn) / scale;

            drive.setVelocity(leftFrontPower * maxVelocity,
                              rightFrontPower * maxVelocity,
                              leftRearPower * maxVelocity,
                              rightRearPower * maxVelocity);

            double currentVelocity = drive.getCurrentVelocity();

            Logger.verbose("%s",
                    String.format("x: %-5.1f  y: %-5.1f  h: %-5.1f  ", currentX, currentY, Math.toDegrees(currentHeading)) +
                            String.format("a: %5.1f  b: %5.1f  distance: %5.2f  ", a, b, distance) +
                            String.format("heading error: %6.1f  ", Math.toDegrees(headingError)) +
                            String.format("angle: %4.0f  ", Math.toDegrees(angle)) +
                            String.format("signed angle: %4.0f  ", Math.toDegrees(signedAngle)) +
                            String.format("turn: %5.2f  power: %4.2f  sin: %5.2f  cos: %5.2f  ", turn, power, sin, cos) +
                            String.format("power: %5.2f  %5.2f  %5.2f  %5.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                            String.format("velocity: %6.1f   ", currentVelocity) +
                            String.format("time: %4.0f", timeoutTimer.milliseconds())
                            //String.format("integral %5.2f", drivePID.errorIntegral)
            );

            if (Math.abs(a) < distanceTolerance && Math.abs(b) < distanceTolerance &&
                    Math.abs(Math.toDegrees(headingError)) < headingTolerance &&
                    Math.abs(currentVelocity) <= velocityTolerance) {
                if (highSpeed) {
                    Logger.message("low speed");
                    moveInit(false);
                } else {
                    break;
                }
            }

            if (timeout > 0 && timeoutTimer.milliseconds() >= timeout) {
                Logger.message("moveDistance timed out");
                break;
            }

            if (emergencyStop()) {
                break;
            }
        }

        Logger.message("time: %4.2f", timeoutTimer.seconds());
        drive.stopRobot();
    }

    private void moveToObject() {

        moveInit(true);
        timeoutTimer.reset();

        // Looping until we move the desired distance
        while (opMode.opModeIsActive() && !interruptAction) {

            double maxVelocity = drive.getMaxVelocity() * MAX_SPEED;
            Pose pose = localizer.getPose();

            double currentX = pose.getX();
            double currentY = pose.getY();
            double currentHeading = pose.getHeading();
            double distance = drive.distanceToObject() - stopDistance;
            double headingError = angleWrap(currentHeading - Math.toRadians(targetHeading));

            headingPID.updateError(headingError);
            double turn = headingPID.runPIDF();

            drivePID.updateError(distance);
            double power = drivePID.runPIDF();

            double scale = 1;
            if (power + Math.abs(turn) > MAX_SPEED)
                scale = (power + Math.abs(turn)) / MAX_SPEED;

            double leftFrontPower  = (power + turn) / scale;
            double rightFrontPower = (power - turn) / scale;
            double leftRearPower   = (power + turn) / scale;
            double rightRearPower  = (power - turn) / scale;

            drive.setVelocity(
                    leftFrontPower * maxVelocity,
                    rightFrontPower * maxVelocity,
                    leftRearPower * maxVelocity,
                    rightRearPower * maxVelocity);

            double currentVelocity = drive.getCurrentVelocity();

            Logger.verbose("%s",
                    String.format("x: %-5.1f  y: %-5.1f  h: %-5.1f  ", currentX, currentY, Math.toDegrees(currentHeading)) +
                            String.format("distance: %5.2f  ", distance) +
                            String.format("heading error: %6.1f  ", Math.toDegrees(headingError)) +
                            String.format("turn: %5.2f  power: %4.2f  ", turn, power) +
                            String.format("power: %5.2f  %5.2f  %5.2f  %5.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                            String.format("velocity: %6.1f   ", currentVelocity) +
                            String.format("time: %4.0f", timeoutTimer.milliseconds())
            );

            if (Math.abs(distance) < distanceTolerance &&
                    Math.abs(Math.toDegrees(headingError)) < headingTolerance &&
                    Math.abs(currentVelocity) <= velocityTolerance) {
                if (highSpeed) {
                    Logger.message("low speed");
                    moveInit(false);
                } else {
                    break;
                }
            }

            if (timeout > 0 && timeoutTimer.milliseconds() >= timeout) {
                Logger.message("moveDistance timed out");
                break;
            }

            if (emergencyStop()) {
                break;
            }
        }

        Logger.message("time: %4.2f", timeoutTimer.seconds());
        drive.stopRobot();
    }

    private void moveWithJoystick () {

        double x = leftX;
        double y = leftY;
        double turn = rightX;

        double maxVelocity = drive.getMaxVelocity() * MAX_SPEED;
        double angle = Math.atan2(y, x);
        double sin = Math.sin(angle - (Math.PI / 4));
        double cos = Math.cos(angle - (Math.PI / 4));
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        double power = Math.hypot(x, y);

        if (power != 0) {
            power = Math.pow(Math.abs(Math.min(power, 1)), 3);  // exponential power curve for better low speed control
            double minPower = drive.getMinPower();
            double maxPower = drive.getMaxPower();
            power = power * (maxPower - minPower) + minPower;
            power = Math.max(drive.accelerationLimit(power), minPower);
            turn /= 3;                              // limit turn speed when drive in any direction

        } else  if (turn != 0) {
            // if only turning scale joystick value for turning only.
            double sign = (turn < 0) ? -1 : 1;
            turn = Math.pow(Math.abs(Math.min(turn, 1)), 3);  // exponential power curve for better low speed control
            double minTurn = drive.getMinTurnPower();
            double maxTurn = drive.getMaxTurnPower();
            turn *= (maxTurn - minTurn) + maxTurn;
            turn *= sign;
        }

        double scale = 1;
        if (power != 0 &&(power + Math.abs(turn) > MAX_SPEED))
            scale = (power + Math.abs(turn)) / MAX_SPEED;

        double leftFrontPower  = (power * (cos/max) + turn) / scale;
        double rightFrontPower = (power * (sin/max) - turn) / scale;
        double leftRearPower   = (power * (sin/max) + turn) / scale;
        double rightRearPower  = (power * (cos/max) - turn) / scale;

        drive.setVelocity(
                leftFrontPower * maxVelocity,
                rightFrontPower * maxVelocity,
                leftRearPower * maxVelocity,
                rightRearPower * maxVelocity);

        Logger.message("%s",
                String.format("x: %5.2f  y: %5.2f  turn: %5.2f  ", x, y, turn) +
                        String.format("angle: %5.2f (rad)  %4.0f (deg)  ", angle, Math.toDegrees(angle)) +
                        String.format("power: %4.2f  sin: %5.2f  cos: %5.2f  ", power, sin, cos) +
                        String.format("power: %4.2f  %4.2f  %4.2f  %4.2f   ", leftFrontPower, rightFrontPower, leftRearPower, rightRearPower) +
                        String.format("pos: %6d %6d %6d %6d  ", drive.leftFrontDrive.getCurrentPosition(), drive.rightFrontDrive.getCurrentPosition(), drive.leftBackDrive.getCurrentPosition(), drive.rightBackDrive.getCurrentPosition()) +
                        String.format("velocity: %4.0f %4.0f %4.0f %4.0f", drive.leftFrontDrive.getVelocity(), drive.rightFrontDrive.getVelocity(), drive.leftBackDrive.getVelocity(), drive.rightBackDrive.getVelocity())
        );
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

    private double polarToSignedAngle(double polarAngle) {

        double angle = polarAngle - Math.PI/2;
        if (angle > Math.PI)
            angle = angle - (Math.PI * 2);
        return angle;
    }

    private void interruptAction () {
        if (driveState != DRIVE_STATE.IDLE) {
            interruptAction = true;
            while (driveState != DRIVE_STATE.IDLE)  {
                Thread.yield();
            }
            interruptAction = false;
        }
    }

    private boolean emergencyStop() {
        return opMode.gamepad1.back;
    }

    public void moveToCoordinate(double targetX, double targetY, double targetHeading, double timeout) {

        synchronized (this) {
            interruptAction();

            this.targetX = targetX;
            this.targetY = targetY;
            this.targetHeading = targetHeading;
            this.timeout = timeout;
            driveState = DRIVE_STATE.MOVING_TO_COORDINATE;
        }
    }

    public void moveToObject (double stopDistance, double timeout) {

        synchronized (this) {
            interruptAction();

            this.stopDistance = stopDistance;
            this.timeout = timeout;
            this.targetHeading = localizer.getPose().getHeading();
            driveState = DRIVE_STATE.MOVING_TO_OBJECT;
        }
    }

    public void moveWithJoystick (double leftX, double leftY, double rightX) {

        synchronized (this) {
            if (driveState != DRIVE_STATE.MOVING) {
                interruptAction();
            }
            this.leftX = leftX;
            this.leftY = leftY;
            this.rightX = rightX;
            driveState = DRIVE_STATE.MOVING;
        }
    }

    public void stopMoving() {
        if (driveState != DRIVE_STATE.IDLE) {
            interruptAction();
        }
        drive.stopRobot();
        driveState = DRIVE_STATE.IDLE;
    }

    public void startMoving() {
        drive.accelerationReset();
    }

    public boolean isBusy () {
        return driveState != DRIVE_STATE.IDLE;
    }

}



