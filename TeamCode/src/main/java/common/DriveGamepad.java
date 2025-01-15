package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class DriveGamepad extends Thread {

    public final double BUCKET_X = 15.5;
    public final double BUCKET_Y = 126.5;
    public final double BUCKET_HEADING = 135;

    public final double SUBMERSIBLE_X = 60;
    public final double SUBMERSIBLE_Y = 130;
    public final double SUBMERSIBLE_HEADING = 270;

    private final double MAX_WAYPOINT_SPEED = 0.5;

    private enum PoseButton { A, B, X, Y}
    private final int poseCount = PoseButton.values().length;
    private final Pose[] poses = new Pose[poseCount];

    LinearOpMode opMode;
    DriveControl driveControl;

    public DriveGamepad(LinearOpMode opMode, DriveControl driveControl) {

        this.opMode = opMode;
        this.driveControl = driveControl;

        //setPosePosition(PoseButton.A, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        //setPosePosition(PoseButton.B, SUBMERSIBLE_X, SUBMERSIBLE_Y, SUBMERSIBLE_HEADING);
    }

    /**
     * Control the motor on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("drive gamepad thread started for %s", this.getName());

        while (!opMode.isStarted()) {
            Thread.yield();
        }

        while (opMode.opModeIsActive()) {
            driveWithJoysticks();
        }

        Logger.message("drive gamepad control thread stopped");
    }

    private void driveWithJoysticks() {

        boolean moving = false;

        while (opMode.opModeIsActive()) {

            Gamepad gamepad = opMode.gamepad1;

            if (gamepad.atRest()) {
                Thread.yield();
            }

            if (gamepad.back) {
                driveControl.emergencyStop();
                break;
            }

            if (gamepad.a) {
                moveToPose(PoseButton.A);
                while (gamepad.a) Thread.yield();
            } else if (gamepad.b) {
                moveToPose(PoseButton.B);
                while (gamepad.b) Thread.yield();
            }if (gamepad.x) {
                moveToPose(PoseButton.X);
                while (gamepad.x) Thread.yield();
            }if (gamepad.y) {
                moveToPose(PoseButton.Y);
                while (gamepad.y) Thread.yield();
            }

            if (gamepad.left_bumper) {
                while (gamepad.left_bumper)
                    if (gamepad.a) {
                        setToCurrentPosition(PoseButton.A);
                        while (gamepad.a) Thread.yield();
                    } else if (gamepad.b) {
                        setToCurrentPosition(PoseButton.B);
                        while (gamepad.b) Thread.yield();
                    } else if (gamepad.x) {
                        setToCurrentPosition(PoseButton.X);
                        while (gamepad.x) Thread.yield();
                    } else if (gamepad.y) {
                        setToCurrentPosition(PoseButton.Y);
                        while (gamepad.y) Thread.yield();
                    }
                while (gamepad.left_bumper) Thread.yield();
            }

            if (gamepad.left_trigger > 0) {
                driveControl.turnBy(2, 1000);
                while (gamepad.left_trigger > 0) Thread.yield();
            }

            if (gamepad.right_trigger > 0) {
                driveControl.turnBy(-2, 1000);
                while (gamepad.right_trigger > 0) Thread.yield();
            }

            // Left stick to go forward, back and strafe. Right stick to rotate.
            double x = gamepad.left_stick_x;
            double y = -gamepad.left_stick_y;
            double x2 = gamepad.right_stick_x;
            double noise = 0.01;

            // Is either stick being used?
            if (Math.abs(x) > noise || Math.abs(y) > noise || Math.abs(x2) > noise ) {

                if (! moving) {
                    driveControl.startMoving();
                    moving = true;
                }
                driveControl.moveWithJoystick(x, y, x2);

            } else if (moving){
                moving = false;
                driveControl.stopMoving();
            }
        }
    }

    public void setPosePosition (PoseButton button, double x, double y, double heading) {
        int index = button.ordinal();
        poses[index] = new Pose(x, y, Math.toRadians(heading));
    }

    private void setToCurrentPosition(PoseButton button) {
        Pose pose = driveControl.getPose();
        int index = button.ordinal();
        poses[index] = pose;
    }

    private void moveToPose(PoseButton button) {
        int index = button.ordinal();
        Pose pose =  poses[index];
        if (pose != null) {
            double x = pose.getX();
            double y = pose.getY();
            double heading = Math.toDegrees(pose.getHeading());

            Logger.message("%s  move to (%.1f, %.1f) heading: %.0f", button, x, y, heading);
            driveControl.moveToCoordinate(x, y, heading, MAX_WAYPOINT_SPEED, 4000 );
        }
    }
}



