package main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import common.Logger;
import common.Robot;

@Autonomous(name="BlueBucketAuto", group = "Main")

 public class BlueAuto extends LinearOpMode {

    public static double START_X = 10.5;
    public static double START_Y = 107.6;
    public static double START_HEADING = 0;

    public static double BUCKET_X = 9.9;
    public static double BUCKET_Y = 134.6;
    public static double BUCKET_HEADING = 136;

    public static double YELLOW_RIGHT_X = 41.5;
    public static double YELLOW_RIGHT_Y = 121.2;
    public static double YELLOW_RIGHT_HEADING = 0;

    public static double YELLOW_MIDDLE_X = 41.5;
    public static double YELLOW_MIDDLE_Y = 131.3;
    public static double YELLOW_MIDDLE_HEADING = 0;

    public static double YELLOW_LEFT_X = 58.4;
    public static double YELLOW_LEFT_Y = 130.4;
    public static double YELLOW_LEFT_HEADING = -90;

    public static double NET_ZONE_X = 18.9;
    public static double NET_ZONE_Y = 129.1;
    public static double SCORE_NET_ZONE_HEADING = -90;


    private static enum PathState { START, BUCKET1, YELLOW_RIGHT, BUCKET2, YELLOW_MIDDLE, BUCKET3, YELLOW_LEFT, SCORE_NET_ZONE;
        public static PathState next(int id) {
            return values()[id];
        }
    }
    private PathState pathState = PathState.START;

    int pathCount = PathState.values().length;
    private final PathChain[] paths = new PathChain[pathCount];

    private ElapsedTime elapsedTime = new ElapsedTime();
    boolean running;

    private Follower follower;
    Robot   robot;

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
        elapsedTime.reset();

        running = true;
        while (running && opModeIsActive()) {

            displayPose();
            switch (pathState) {
                case START:
                    //robot.setToStartPosition();
                    followPath();
                    continue;

                case BUCKET1:
                case BUCKET3:
                case BUCKET2:
                    waitUntilNotMoving();
                    //robot.dropSampleInTopBucket();
                    followPath();
                    continue;

                case YELLOW_RIGHT:
                case YELLOW_MIDDLE:
                    waitUntilNotMoving();
                    //robot.pickUpYellow();
                    followPath();
                    continue;

                case YELLOW_LEFT:
                    robot.pushSample();
                    followPath();
                    continue;

                case SCORE_NET_ZONE:
                    waitUntilNotMoving();
                    running = false;
                    Logger.message("elapsed time %4.1f", elapsedTime.seconds());
            }
            follower.update();
        }
    }

    private void initialize() {
        robot = new Robot(this);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        buildPaths();
    }

    private void buildPaths() {
        paths[PathState.START.ordinal()] = createCurve(START_X, START_Y, START_X+36.5, START_Y-5, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET1.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_RIGHT_X, YELLOW_RIGHT_Y, YELLOW_RIGHT_HEADING);
        paths[PathState.YELLOW_RIGHT.ordinal()] = createLine(YELLOW_RIGHT_X, YELLOW_RIGHT_Y, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET2.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, YELLOW_MIDDLE_HEADING);
        paths[PathState.YELLOW_MIDDLE.ordinal()] = createLine(YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET3.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_LEFT_X, YELLOW_LEFT_Y, YELLOW_LEFT_HEADING);
        paths[PathState.YELLOW_LEFT.ordinal()] = createLine(YELLOW_LEFT_X, YELLOW_LEFT_Y, NET_ZONE_X, NET_ZONE_Y,SCORE_NET_ZONE_HEADING);
    }

    private void followPath() {

        robot.waitUntilOkToMove();
        int index = pathState.ordinal();

        Path path = paths[index].getPath(0);
        Logger.message("path %d  (%.0f, %.0f)  to  (%.0f, %.0f)",
                index,
                path.getFirstControlPoint().getX(),
                path.getFirstControlPoint().getY(),
                path.getLastControlPoint().getX(),
                path.getLastControlPoint().getY());

        follower.followPath(paths[index]);
        pathState  = PathState.next(index+1);

        ifTestingWait();
    }

    private boolean isBusy () {
        return follower.isBusy() ||  robot.isBusy();
    }

    private void waitUntilNotMoving () {
        while (follower.isBusy() ) {
            if (! opModeIsActive())
                break;
            sleep(10);
        }
    }

    private PathChain createCurve (double startX, double startY, double pointX, double pointY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startX, startY), new Point(pointX, pointY), new Point(endX, endY, Point.CARTESIAN)))
                .setPathEndHeadingConstraint(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(500)
                .build();
    }

    private PathChain createLine (double startX, double startY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startX, startY), new Point(endX, endY, Point.CARTESIAN)))
                .setPathEndHeadingConstraint(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(500)
                .build();
    }

    private void ifTestingWait () {

        if (true || robot.isTestRobot()) {
            while (! gamepad1.x && opModeIsActive()) {
                follower.update();
                displayPose();
                //telemetry.addLine("press x to continue");
                //telemetry.update();

                if (gamepad1.x) {
                    while (gamepad1.x) sleep(10);
                    break;
                }
            }
        }
    }

    private void displayPose () {
        Pose pose = follower.getPose();
        telemetry.addData("pose", "x %4.0f  y %4.0f  heading %4.0f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.update();
        Logger.message("pose x %4.0f  y %4.0f  heading %4.0f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
