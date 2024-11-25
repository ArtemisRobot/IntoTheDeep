package main;

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

@Autonomous(name="BlueSideBlueAuto", group = "Main")

 public class BlueSideBlueAuto extends LinearOpMode {

    public static double START_X = 22.9;
    public static double START_Y = 55.1;
    public static double START_HEADING = 0;

    public static double OBSERV_ZONE_X = 13.7;
    public static double OBSERV_ZONE_Y = 23.3;
    public static double OBSERV_HEADING = -180;

    public static double BLUE_RIGHT_X = 61.3;
    public static double BLUE_RIGHT_Y = 5.5;
    public static double BLUE_RIGHT_HEADING = -180;

    public static double BLUE_MIDDLE_X = 61.3;
    public static double BLUE_MIDDLE_Y = 9.6;
    public static double BLUE_MIDDLE_HEADING = 180;

    public static double BLUE_LEFT_X = 61.3;
    public static double BLUE_LEFT_Y = 19.2;
    public static double BLUE_LEFT_HEADING = 180;

    public static double BLUE_BAR_ZONE_X = 46.6;
    public static double BLUE_BAR_ZONE_Y = 68.3;
    public static double BLUE_BAR_HEADING = 0;


    private static enum PathState { START, SAMPLE1, OBSERV_ZONE1, SAMPLE2, OBSERV_ZONE2, SAMPLE3, OBSERV_ZONE_PICKUP1, BLUE_BAR1, OBSERV_ZONE_PICKUP2, BLUE_BAR2, OBSERV_ZONE_PICKUP3, BLUE_BAR3;
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

                case SAMPLE1:
                case SAMPLE2:
                case SAMPLE3:
                    waitUntilNotMoving();
                    //robot.dropSampleInTopBucket();
                    followPath();
                    continue;

                case OBSERV_ZONE1:
                case OBSERV_ZONE2:
                case OBSERV_ZONE_PICKUP1:
                case OBSERV_ZONE_PICKUP2:
                case OBSERV_ZONE_PICKUP3:
                    waitUntilNotMoving();
                    robot.pushSample();
                    //robot.pickUpYellow();
                    followPath();
                    continue;

                case BLUE_BAR1:
                case BLUE_BAR2:
                case BLUE_BAR3:
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
        paths[PathState.START.ordinal()] = createLine(START_X, START_Y, START_X+36.5, START_Y-5, START_HEADING);
        paths[PathState.SAMPLE1.ordinal()] = createLine(BLUE_LEFT_X, BLUE_LEFT_Y, OBSERV_ZONE_X, OBSERV_ZONE_Y, BLUE_LEFT_HEADING);
        paths[PathState.OBSERV_ZONE1.ordinal()] = createLine(OBSERV_ZONE_X, OBSERV_ZONE_Y, BLUE_MIDDLE_X, BLUE_MIDDLE_Y, BLUE_MIDDLE_HEADING);
        paths[PathState.SAMPLE2.ordinal()] = createLine(BLUE_MIDDLE_X, BLUE_MIDDLE_Y, OBSERV_ZONE_X, OBSERV_ZONE_Y, OBSERV_HEADING);
        //paths[PathState.OBSERV_ZONE2.ordinal()] = createLine(YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, BUCKET_X, OBS_ZONE_Y, BUCKET_HEADING);
        //paths[PathState.SAMPLE3.ordinal()] = createLine(BUCKET_X, OBS_ZONE_Y, YELLOW_LEFT_X, YELLOW_LEFT_Y, YELLOW_LEFT_HEADING);
        ///paths[PathState.OBSERV_ZONE_PICKUP1.ordinal()] = createLine(YELLOW_LEFT_X, YELLOW_LEFT_Y, NET_ZONE_X, NET_ZONE_Y,SCORE_NET_ZONE_HEADING);
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
