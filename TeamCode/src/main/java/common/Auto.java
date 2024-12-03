package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import main.BlueSideBlueAuto;
import main.RedSideYellowAuto;

public class Auto {

    private enum PathState {
        START_YELLOW, BUCKET1, YELLOW_RIGHT, BUCKET2, YELLOW_MIDDLE, BUCKET3, PARK, YELLOW_LEFT, SCORE_NET_ZONE,
        START_BLUE_RED, BAR1, SAMPLE_RIGHT, DROP_ZONE1, SAMPLE_MIDDLE, DROP_ZONE2, SAMPLE_LEFT, DROP_ZONE3, PICKUP_ZONE1, BAR2, PICKUP_ZONE2, BAR3, PICKUP_ZONE3, BAR4;
        public static PathState next(int id) {
            return values()[id];
        }
    }
    private PathState pathState;
    int pathCount = PathState.values().length;
    private final PathChain[] paths = new PathChain[pathCount];

    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    boolean running;

    private final LinearOpMode opMode;
    private final Robot robot;
    private final Follower follower;

    private final double MAX_POWER = 0.5;              // ToDo
    private final double MAX_POWER_ADJUST = MAX_POWER;

    private double targetX;
    private double targetY;


    public Auto(LinearOpMode opMode) {

        this.opMode = opMode;
        robot = new Robot(opMode);
        follower = new Follower(opMode.hardwareMap);
        follower.setMaxPower(MAX_POWER);
    }

    public void runYellowAuto() {

        running = true;
        elapsedTime.reset();
        while (running && opMode.opModeIsActive()) {

            follower.update();
            displayPose();
            switch (pathState) {
                case START_YELLOW:
                    robot.setToStartPosition();
                    followPath();
                    break;

                case BUCKET1:
                case BUCKET3:
                case BUCKET2:
                    waitUntilNotMoving();
                    scoreInTopBucket();
                    followPath();
                    break;

                case YELLOW_RIGHT:
                case YELLOW_MIDDLE:
                    waitUntilNotMoving();
                    robot.pickUpYellow();
                    followPath();
                    break;

                case YELLOW_LEFT:
                    robot.pushSample();
                    followPath();
                    break;

                case SCORE_NET_ZONE:
                    break;

                case PARK:
                    waitUntilRobotIdIdle();
                    robot.setToStopPosition();
                    waitUntilRobotIdIdle();
                    running = false;
                    break;
            }
        }
        Logger.message("elapsed time %4.1f", elapsedTime.seconds());
    }

    public void runBlueRedAuto () {

        running = true;
        elapsedTime.reset();
        while (running && opMode.opModeIsActive()) {

            displayPose();

            switch (pathState) {
                case START_BLUE_RED:
                    robot.setToStartPosition();
                    followPath();
                    break;

                case BAR1:
                case BAR2:
                case BAR3:
                case BAR4:
                    waitUntilNotMoving();
                    robot.scoreSpecimen();
                    followPath();
                    break;

                case SAMPLE_RIGHT:
                case SAMPLE_MIDDLE:
                case SAMPLE_LEFT:
                    // push sample to observation zone
                    break;

                case DROP_ZONE1:
                case DROP_ZONE2:
                    break;

                case DROP_ZONE3:
                    break;

                case PICKUP_ZONE1:
                case PICKUP_ZONE2:
                case PICKUP_ZONE3:
            }
        }
    }

    public void buildYellowPaths(
            double startX,
            double startY,
            double startHeading,
            double bucketX,
            double bucketY,
            double bucketHeading,
            double yellowRightX,
            double yellowRightY,
            double yellowRightHeading,
            double yellowMiddleX,
            double yellowMiddleY,
            double yellowMiddleHeading,
            double yellowLeftX,
            double yellowLeftY,
            double yellowLeftHeading,
            double netZoneX,
            double netZoneY,
            double netZoneHeading,
            double parkX,
            double parkY,
            double parkHeading) {

        double x = 4;
        if (startHeading != 0) x = -x;
        paths[PathState.START_YELLOW.ordinal()] = createLines(startX, startY, startX+x, startY, startHeading, bucketX, bucketY, bucketHeading);
        paths[PathState.BUCKET1.ordinal()] = createLine(bucketX, bucketY, yellowRightX, yellowRightY, yellowRightHeading);
        paths[PathState.YELLOW_RIGHT.ordinal()] = createLine(yellowRightX, yellowRightY, bucketX, bucketY, bucketHeading);
        paths[PathState.BUCKET2.ordinal()] = createLine(bucketX, bucketY, yellowMiddleX, yellowMiddleY, yellowMiddleHeading);
        paths[PathState.YELLOW_MIDDLE.ordinal()] = createLine(yellowMiddleX, yellowMiddleY, bucketX, bucketY, bucketHeading);
        paths[PathState.BUCKET3.ordinal()] = createLine(bucketX, bucketY, parkX, parkY, parkHeading);
        //paths[PathState.BUCKET3.ordinal()] = createLine(bucketX, bucketY, yellowLeftX, yellowLeftY, yellowLeftHeading);
        //paths[PathState.YELLOW_LEFT.ordinal()] = createLine(yellowLeftX, yellowLeftY, netZoneX, netZoneY, netZoneHeading);

        pathState = PathState.START_YELLOW;
    }

    private void nextYellowPath() {
        pathState  = PathState.next(pathState.ordinal()+1);
    }

    public void setStartingPose(double startX, double startY, double startHeading) {
        follower.setStartingPose(new Pose(startX, startY, startHeading));
    }

    private void followPath() {

        waitUntilOkToMove();
        int index = pathState.ordinal();

        PathChain currentPath = paths[index];
        int i = currentPath.size();
        Path endPath = currentPath.getPath(i-1);
        targetX = endPath.getLastControlPoint().getX();
        targetY = endPath.getLastControlPoint().getY();

        for (int j = 0; j < currentPath.size(); j++) {
            Path path = currentPath.getPath(j);
            Logger.message("path %s index %d  (%.0f, %.0f)  to  (%.0f, %.0f)",
                    pathState,
                    index,
                    path.getFirstControlPoint().getX(),
                    path.getFirstControlPoint().getY(),
                    path.getLastControlPoint().getX(),
                    path.getLastControlPoint().getY());
        }

        follower.followPath(paths[index], true);
        pathState  = PathState.next(index+1);
    }

    private void waitUntilNotMoving () {
        Logger.message("waiting");
        timer.reset();
        while (/*follower.isBusy() && */ opMode.opModeIsActive()) {
            follower.update();
            displayPose();

            Pose pose = follower.getPose();

            if (timer.milliseconds() > 3000) {
                Logger.message("pose: %5.1f %5.1f  target: %5.1f %5.1f  busy %b", pose.getX(), pose.getY(), targetX, targetY, follower.isBusy());
                Logger.warning("follower timed out");
                break;
            }
        }
        Logger.message("done waiting");
    }

    private void waitUntilRobotIdIdle() {
        Logger.message("waiting");
        timer.reset();
        while (robot.isBusy() && opMode.opModeIsActive()) {
            follower.update();
            if (timer.milliseconds() > 3000) {
                Logger.warning("robot timed out");
                break;
            }
        }
        Logger.message("done waiting");
    }

    private void waitUntilOkToMove() {
        Logger.message("waiting");
        timer.reset();
        while (!robot.okToMove() && opMode.opModeIsActive()) {
            follower.update();
            if (timer.milliseconds() > 5000) {
                Logger.warning("robot timed out");
                break;
            }
        }
        Logger.message("done waiting");
    }

    public void scoreInTopBucket() {

        //follower.setStartingPose(new Pose(0, 0, Math.toRadians(135)));
        //follower.update();

        robot.dropSampleInTopBucket();
        waitUntilOkToMove();
        if (true) return;

        PathChain path;
        Pose pose = follower.getPose();
        double adjust = 3;
        if (pose.getHeading() < 0) adjust = -adjust;            // ToDo use trig
        double heading = Math.toDegrees(pose.getHeading());
        double x1 = pose.getX();
        double y1 =  pose.getX();
        double x2 = x1 - adjust;
        double y2 = y1 + adjust;

        Logger.message("sin %f  cos %f" , Math.sin(heading), Math.cos(heading));

        robot.lifterUp();
        waitUntilRobotIdIdle();

        Logger.message("%f %f %f %f %f", x1, y1, x2, y2, heading);
        follower.setMaxPower(MAX_POWER_ADJUST);
        path = createLine(x1, y1, x2, y2, heading);
        follower.resetOffset();
        follower.followPath(path, true);
        waitUntilNotMoving();

        robot.dropSampleInTopBucket();
        waitUntilOkToMove();

        path = createLine(x2, y2, x1, y1, heading);
        follower.followPath(path, true);
        waitUntilNotMoving();
        follower.setMaxPower(MAX_POWER);

        robot.lifterDown();
    }
        
    private PathChain createCurve (double startX, double startY, double pointX, double pointY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startX, startY), new Point(pointX, pointY), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(3)
                .build();
    }

    private PathChain createLines (double startX, double startY, double pointX, double pointY, double heading, double endX, double endY, double endHeading) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startX, startY, Point.CARTESIAN), new Point(pointX, pointY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .addPath(new BezierLine(new Point(pointX, pointY, Point.CARTESIAN), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(endHeading))
                .setPathEndTimeoutConstraint(3)
                .build();
    }

    private PathChain createLine (double startX, double startY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startX, startY, Point.CARTESIAN), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(3)
                .build();
    }

    private void displayPose () {
        Pose pose = follower.getPose();
        opMode.telemetry.addData("Path", pathState);
        opMode.telemetry.addData("pose", "x %5.1f  y %5.1f  heading %5.1f  is busy %b",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()),
                follower.isBusy());
        opMode.telemetry.update();
        //Logger.message("pose x %4.0f  y %4.0f  heading %4.0f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private void waitForButtonPress() {
        Logger.message("waiting for button press");

        boolean pressed = false;
        while (opMode.opModeIsActive()) {
            follower.update();
            displayPose();

            if (pressed) {
                if (! opMode.gamepad1.x) {
                    break;
                }
            } else if (opMode.gamepad1.x) {
                pressed = true;
            }
        }
        Logger.message("done waiting for button press");
    }

    public void test () {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        follower.setStartingPose(new Pose(32, 127, 0));
        follower.update();

        PathChain path =  follower.pathBuilder()
                .addPath(new BezierLine(new Point(32, 127, Point.CARTESIAN), new Point(24, 120, Point.CARTESIAN)))
                .setPathEndHeadingConstraint(Math.toRadians(135))
                .setPathEndTimeoutConstraint(3)
                .build();

        follower.followPath(path);
        waitUntilNotMoving();

        opMode.telemetry.addData("Time", elapsedTime.milliseconds());
        while (opMode.opModeIsActive()) opMode.sleep(10);
    }
}



