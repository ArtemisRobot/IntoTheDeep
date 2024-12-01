package main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import common.Logger;
import common.Robot;

@Autonomous(name="BlueSideYellowAuto", group = "Main")
@Config

 public class BlueSideYellowAuto extends LinearOpMode {

    public static double START_X = 8.5;
    public static double START_Y = 102.5;
    public static double START_HEADING = 0;

    public static double BUCKET_X = 13;
    public static double BUCKET_Y = 127;
    public static double BUCKET_HEADING = 135;

    public static double YELLOW_RIGHT_X = 33;
    public static double YELLOW_RIGHT_Y = 117;
    public static double YELLOW_RIGHT_HEADING = 0;

    public static double YELLOW_MIDDLE_X = 33;
    public static double YELLOW_MIDDLE_Y = 127;
    public static double YELLOW_MIDDLE_HEADING = 0;

    public static double YELLOW_LEFT_X = 58.4;
    public static double YELLOW_LEFT_Y = 130.4;
    public static double YELLOW_LEFT_HEADING = -90;

    public static double NET_ZONE_X = 18.9;
    public static double NET_ZONE_Y = 129.1;
    public static double SCORE_NET_ZONE_HEADING = -90;

    public static double PARK_X = 12;
    public static double PARK_Y = 100;
    public static double PARK_HEADING = 90;

    private enum PathState { START, BUCKET1, YELLOW_RIGHT, BUCKET2, YELLOW_MIDDLE, BUCKET3, PARK, YELLOW_LEFT, SCORE_NET_ZONE;
        public static PathState next(int id) {
            return values()[id];
        }
    }
    private PathState pathState = PathState.START;

    int pathCount = PathState.values().length;
    private final PathChain[] paths = new PathChain[pathCount];

    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
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
                    robot.setToStartPosition();
                    followPath();
                    continue;

                case BUCKET1:
                case BUCKET3:
                case BUCKET2:
                    waitUntilNotMoving();
                    //waitForButtonPress();
                    Logger.addLine("dropSampleInTopBucket");
                    robot.dropSampleInTopBucket();
                    followPath();
                    continue;

                case YELLOW_RIGHT:
                case YELLOW_MIDDLE:
                    waitUntilNotMoving();
                    //waitForButtonPress();
                    robot.pickUpYellow();
                    followPath();
                    continue;

                case YELLOW_LEFT:
                    //waitForButtonPress();
                    robot.pushSample();
                    followPath();
                    continue;

                case SCORE_NET_ZONE:

                case PARK:
                    robot.setToStopPosition();
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
        paths[PathState.START.ordinal()] = createLines(START_X, START_Y, START_X+10 , START_Y, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET1.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_RIGHT_X, YELLOW_RIGHT_Y, YELLOW_RIGHT_HEADING);
        paths[PathState.YELLOW_RIGHT.ordinal()] = createLine(YELLOW_RIGHT_X, YELLOW_RIGHT_Y, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET2.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, YELLOW_MIDDLE_HEADING);
        paths[PathState.YELLOW_MIDDLE.ordinal()] = createLine(YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET3.ordinal()] = createLine(BUCKET_X, BUCKET_Y, PARK_X, PARK_Y, PARK_HEADING);
        //       paths[PathState.BUCKET3.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_LEFT_X, YELLOW_LEFT_Y, YELLOW_LEFT_HEADING);
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
    }

    private boolean isBusy () {
        return follower.isBusy() ||  robot.isBusy();
    }

    private void waitUntilNotMoving () {
        Logger.message("waiting until not moving");
        timer.reset();
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
            if (timer.milliseconds() > 3000) {
                Logger.warning("follower timed out");
                break;
            }
        }
        Logger.message("done waiting until not moving");
    }

    private PathChain createCurve (double startX, double startY, double pointX, double pointY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startX, startY), new Point(pointX, pointY), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(3)
                .build();
    }

    private PathChain createLines (double startX, double startY, double pointX, double pointY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startX, startY), new Point(pointX, pointY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(START_HEADING))
                .addPath(new BezierLine(new Point(pointX, pointY), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(3)
                .build();
    }

    private PathChain createLine (double startX, double startY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startX, startY), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(3)
                .build();
    }

    private void displayPose () {
        Pose pose = follower.getPose();
        telemetry.addData("Path", pathState);
        telemetry.addData("pose", "x %5.1f  y %5.1f  heading %5.1f  is busy %b",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()),
                follower.isBusy());
        telemetry.update();
        //Logger.message("pose x %4.0f  y %4.0f  heading %4.0f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private void waitForButtonPress() {
        Logger.message("waiting for button press");

        boolean pressed = false;
        while (opModeIsActive()) {
            follower.update();
            displayPose();

            if (pressed) {
                if (! gamepad1.x) {
                    break;
                }
            } else if (gamepad1.x) {
                pressed = true;
            }
        }
        Logger.message("done waiting for button press");
    }
}
