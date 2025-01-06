package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
@Deprecated
public class AutoPedroPathing {

    private enum PathState {
        START_YELLOW,
        NET_ZONE_IN1, BUCKET1, NET_ZONE_OUT1, YELLOW_RIGHT,
        NET_ZONE_IN2, BUCKET2, NET_ZONE_OUT2, YELLOW_MIDDLE,
        NET_ZONE_IN3, BUCKET3, NET_ZONE_OUT3, PARK,
        YELLOW_LEFT, SCORE_NET_ZONE,
        START_BLUE_RED, BAR1, SPECIMEN_RIGHT, OBSERVE_ZONE1, SPECIMEN_MIDDLE, OBSERVE_ZONE2, SPECIMEN_LEFT, OBSERVE_ZONE3, PICKUP_ZONE1, BAR2, PICKUP_ZONE2, BAR3, PICKUP_ZONE3, BAR4;
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

    Drive drive;

    private final double MAX_POWER = 0.6;              // ToDo increase
    private final double MAX_POWER_ADJUST = MAX_POWER;

    private double targetX;
    private double targetY;


    public AutoPedroPathing(LinearOpMode opMode) {

        this.opMode = opMode;
        robot = new Robot(opMode);
        follower = new Follower(opMode.hardwareMap);
        follower.setMaxPower(MAX_POWER);

        //drive = new Drive(opMode);
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

                case NET_ZONE_IN1:
                case NET_ZONE_IN2:
                case NET_ZONE_IN3:
                    waitUntilNotMoving();
                    waitUntilRobotIdIdle();
                    robot.lifterUp();
                    waitUntilRobotIdIdle();
                    followPath();
                    break;

                case BUCKET1:
                case BUCKET3:
                case BUCKET2:
                    waitUntilNotMoving();
                    robot.dropSampleInTopBucket();
                    followPath();
                    break;

                case NET_ZONE_OUT1:
                case NET_ZONE_OUT2:
                case NET_ZONE_OUT3:
                    robot.lifterDown();
                    opMode.sleep(500);     // ToDo necessary, wait for lifter to start to come down?
                    followPath();
                    break;

                case YELLOW_RIGHT:
                case YELLOW_MIDDLE:
                    waitUntilNotMoving();
                    robot.pickUpSample();
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

                case SPECIMEN_RIGHT:
                case SPECIMEN_MIDDLE:
                case SPECIMEN_LEFT:
                case OBSERVE_ZONE1:
                case OBSERVE_ZONE2:
                case OBSERVE_ZONE3:
                    // push sample to observation zone
                    followPath();
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
            double netZoneX,
            double netZoneY,
            double netZoneHeading,
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
            double parkX,
            double parkY,
            double parkHeading) {

        double x = 4;
        double y = -4;
        if (startHeading != 0)  {
            x = -x;
            y = -y;
        }
        paths[PathState.START_YELLOW.ordinal()] =  createLines(startX, startY, startX+x, startY, startHeading, netZoneX, netZoneY, netZoneHeading, 3);
        paths[PathState.NET_ZONE_IN1.ordinal()] =  createLine(netZoneX, netZoneY, bucketX, bucketY, bucketHeading, 3);
        paths[PathState.BUCKET1.ordinal()] =       createLine(bucketX, bucketY, netZoneX, netZoneY, netZoneHeading, 3);
        paths[PathState.NET_ZONE_OUT1.ordinal()] = createLine(netZoneX, netZoneY, yellowRightX, yellowRightY, yellowRightHeading, 500);
        paths[PathState.YELLOW_RIGHT.ordinal()] =  createLine(yellowRightX, yellowRightY, netZoneX, netZoneY, netZoneHeading, 3);
        paths[PathState.NET_ZONE_IN2.ordinal()] =  createLine(netZoneX, netZoneY, bucketX, bucketY, bucketHeading, 3);
        paths[PathState.BUCKET2.ordinal()] =       createLine(bucketX, bucketY, netZoneX, netZoneY, netZoneHeading, 3);
        paths[PathState.NET_ZONE_OUT2.ordinal()] = createLine(netZoneX, netZoneY, yellowMiddleX, yellowMiddleY, yellowMiddleHeading, 3);
        paths[PathState.YELLOW_MIDDLE.ordinal()] = createLines(yellowMiddleX, yellowMiddleY, yellowMiddleX, yellowMiddleY+y, yellowMiddleHeading, netZoneX, netZoneY, netZoneHeading, 3);
        paths[PathState.NET_ZONE_IN3.ordinal()] =  createLine(netZoneX, netZoneY, bucketX, bucketY, bucketHeading, 3);
        paths[PathState.BUCKET3.ordinal()] =       createLine(bucketX, bucketY, netZoneX, netZoneY, netZoneHeading, 3);
        paths[PathState.NET_ZONE_OUT3.ordinal()] = createLine(netZoneX, netZoneY, parkX, parkY, parkHeading, 500);
        paths[PathState.PARK.ordinal()] =          createLine(parkX, parkY, parkX, parkY, parkHeading, 3);

        //paths[PathState.BUCKET3.ordinal()] = createLine(bucketX, bucketY, yellowLeftX, yellowLeftY, yellowLeftHeading);
        //paths[PathState.YELLOW_LEFT.ordinal()] = createLine(yellowLeftX, yellowLeftY, netZoneX, netZoneY, netZoneHeading);

        pathState = PathState.START_YELLOW;
    }

    public void buildBlueRedPaths(
            double startX,
            double startY,
            double startHeading,
            double barX1,
            double barY1,
            double barHeading1,
            double observeZoneX1,
            double observeZoneY1,
            double observeHeading1,
            double observeZoneX2,
            double observeZoneY2,
            double observeHeading2,
            double observeZoneX3,
            double observeZoneY3,
            double observeHeading3,
            double specimenRightX,
            double specimenRightY,
            double specimenRightHeading,
            double specimenMiddleX,
            double specimenMiddleY,
            double specimenMiddleHeading,
            double specimenLeftX,
            double specimenLeftY,
            double specimenLeftHeading,
            double barX2,
            double barY2,
            double barHeading2) {


        paths[PathState.START_BLUE_RED.ordinal()] =  createLine(startX, startY, barX1, barY1, barHeading1, 3);
        paths[PathState.BAR1.ordinal()] =            createLine(barX1, barY1, specimenRightX, specimenRightY, specimenRightHeading, 3);
        paths[PathState.SPECIMEN_RIGHT.ordinal()] =  createLine(specimenRightX, specimenRightY, observeZoneX1, observeZoneY1, observeHeading1, 3);
        paths[PathState.OBSERVE_ZONE1.ordinal()] =   createLines(observeZoneX1, observeZoneY1, specimenRightX, specimenRightY, specimenRightHeading, specimenMiddleX, specimenMiddleY, specimenMiddleHeading, 3);
        paths[PathState.SPECIMEN_MIDDLE.ordinal()] = createLine(specimenMiddleX, specimenMiddleY, observeZoneX2, observeZoneY2, observeHeading2, 3);
        paths[PathState.OBSERVE_ZONE2.ordinal()] =   createLines(observeZoneX2, observeZoneY2, specimenMiddleX, specimenMiddleY, specimenMiddleHeading, specimenLeftX, specimenLeftY, specimenLeftHeading, 3);
        paths[PathState.SPECIMEN_LEFT.ordinal()] =   createLine(specimenLeftX, specimenLeftY, observeZoneX3, observeZoneY3, observeHeading3, 3);

        /*
        paths[PathState.OBSERVE_ZONE3.ordinal()] = createLine();
        paths[PathState.PICKUP_ZONE1.ordinal()] = createLine();
        paths[PathState.BAR2.ordinal()] = createLine();
        paths[PathState.PICKUP_ZONE2.ordinal()] = createLine();
        paths[PathState.BAR3.ordinal()] = createLine();
         */

        pathState = PathState.START_BLUE_RED;

    }
    
    public void setStartingPose(double startX, double startY, double startHeading) {
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeading)));
    }

    private void followPath() {

        waitUntilOkToMove();

        int index = pathState.ordinal();
        PathChain currentPath = paths[index];
        if (currentPath == null) {
            Logger.warning("path for %s missing", pathState);
            return;
        }
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
        Logger.message("waiting, follower is %b", follower.isBusy());
        if (!follower.isBusy() )
            Logger.warning("follower is not busy");
        timer.reset();
        while (follower.isBusy() &&  opMode.opModeIsActive()) {
            follower.update();
            displayPose();

            if (timer.milliseconds() > 3000) {
                Logger.warning("follower timed out");
                break;
            }
        }
        Pose pose = follower.getPose();
        Logger.message("pose: %5.1f %5.1f  target: %5.1f %5.1f  busy %b", pose.getX(), pose.getY(), targetX, targetY, follower.isBusy());
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
        path = createLine(x1, y1, x2, y2, heading, 3);
        follower.resetOffset();
        follower.followPath(path, true);
        waitUntilNotMoving();

        robot.dropSampleInTopBucket();
        waitUntilOkToMove();

        path = createLine(x2, y2, x1, y1, heading, 3);
        follower.followPath(path, true);
        waitUntilNotMoving();
        follower.setMaxPower(MAX_POWER);

        robot.lifterDown();
    }

    public void scoreSpecimen() {

        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        follower.update();

        PathChain path;
        Pose pose = follower.getPose();
        double adjust = 8;
        if (pose.getHeading() < 0) adjust = -adjust;            // ToDo use trig
        double heading = Math.toDegrees(pose.getHeading());
        double x1 = pose.getX();
        double y1 =  pose.getX();
        double x2 = x1 + adjust;
        double y2 = y1 + adjust;

        Logger.message("sin %f  cos %f" , Math.sin(heading), Math.cos(heading));

        Logger.message("%f %f %f %f %f", x1, y1, x2, y2, heading);
        follower.setMaxPower(.4);
        path = createLine(x1, y1, x2, y2, heading, 3);
        follower.followPath(path, true);
        waitUntilNotMoving();

        follower.breakFollowing();

        Pose currentPose = follower.getPose();

        drive.moveToObject(0.25, 4, 2000);
        opMode.sleep(3000);

        follower.update();

        timer.reset();
        follower.holdPoint(currentPose);
        while (timer.seconds() < 3) {
            follower.update();
        }

    }

    private PathChain createCurve (double startX, double startY, double pointX, double pointY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startX, startY), new Point(pointX, pointY), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(2000)
                .build();
    }

    private PathChain createLines (double startX, double startY, double pointX, double pointY, double heading, double endX, double endY, double endHeading, double timeout) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startX, startY, Point.CARTESIAN), new Point(pointX, pointY, Point.CARTESIAN)))
                .setPathEndHeadingConstraint(Math.toRadians(heading))
                .addPath(new BezierLine(new Point(pointX, pointY, Point.CARTESIAN), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(endHeading))
                .setPathEndTimeoutConstraint(timeout)
                .build();
    }

    private PathChain createLine (double startX, double startY, double endX, double endY, double heading, double timeout) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startX, startY, Point.CARTESIAN), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(timeout)
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



