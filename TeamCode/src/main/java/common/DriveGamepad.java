package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveGamepad extends Thread {

    public final double BUCKET_X = 14;
    public final double BUCKET_Y = 128;
    public final double BUCKET_HEADING = 135;

    public static class Pose {
        double x;
        double y;
        double heading;

        Pose (double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    private enum PosePosition { A, B, X, Y}
    private final int poseCount = PosePosition.values().length;
    private final Pose[] poses = new Pose[poseCount];

    LinearOpMode opMode;
    DriveControl driveControl;

    public DriveGamepad(LinearOpMode opMode, DriveControl driveControl) {

        this.opMode = opMode;
        this.driveControl = driveControl;

        setPosePosition(PosePosition.A, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
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
                opMode.requestOpModeStop();      // ToDo remove, emergency stop for testing
                break;
            }

            if (gamepad.a) {
                int index = PosePosition.A.ordinal();
                driveControl.moveToCoordinate(poses[index].x, poses[index].y, poses[index].heading, 4000 );
                while (gamepad.a) Thread.yield();
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

    public void setPosePosition (PosePosition position, double x, double y, double heading) {
        int index = position.ordinal();
        poses[index] = new Pose(x, y, heading);
    }
}



