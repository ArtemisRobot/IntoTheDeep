package main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import common.Logger;
import common.Robot;

@Autonomous(name="BlueBucketAuto", group = "Main")

 public class BlueAuto extends LinearOpMode {

    public static double START_X = 10;
    public static double START_Y = 85;
    public static double START_HEADING = 0;

    public static double BUCKET_X = 20;
    public static double BUCKET_Y = 120;
    public static double BUCKET_HEADING = 135    ;

    public static double YELLOW_RIGHT_X = 0;
    public static double YELLOW_RIGHT_Y = 0;
    public static double YELLOW_RIGHT_HEADING = 0;

    public static double YELLOW_MIDDLE_X = 0;
    public static double YELLOW_MIDDLE_Y = 0;
    public static double YELLOW_MIDDLE_HEADING = 0;

    public static double YELLOW_LEFT_X = 0;
    public static double YELLOW_LEFT_Y = 0;
    public static double YELLOW_LEFT_HEADING = 0;

    public static double SCORE_NET_ZONE_HEADING = 0;
    public static double NET_ZONE_X = 0;
    public static double NET_ZONE_Y = 0;


    private static enum PathState { START, BUCKET1, YELLOW_RIGHT, BUCKET2, YELLOW_MIDDLE, BUCKET3, YELLOW_LEFT, SCORE_NET_ZONE;
        public static PathState next(int id) {
            return values()[id];
        }
    }
    private PathState pathState = PathState.START;

    int pathCount = PathState.values().length;
    private final PathChain[] paths = new PathChain[pathCount];

    private Follower follower;
    Robot   robot;

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {

            if ( ! isBusy()) {
                switch (pathState) {
                    case START:
                        followPath();
                        continue;

                    case BUCKET1:
                    case BUCKET3:
                    case BUCKET2:
                        robot.dropSampleInTopBucket();
                        followPath();
                        continue;

                    case YELLOW_RIGHT:
                    case YELLOW_MIDDLE:
                        robot.pickUpYellow();
                        followPath();
                        continue;

                    case YELLOW_LEFT:
                        robot.pushSample();
                        followPath();
                        continue;

                    case SCORE_NET_ZONE:
                }
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
        paths[PathState.START.ordinal()] = createCurve(START_X, START_Y, START_X+20, START_Y, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET1.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_RIGHT_X, YELLOW_RIGHT_Y, YELLOW_RIGHT_HEADING);
        paths[PathState.YELLOW_RIGHT.ordinal()] = createLine(YELLOW_RIGHT_X, YELLOW_RIGHT_Y, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET2.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, YELLOW_MIDDLE_HEADING);
        paths[PathState.YELLOW_MIDDLE.ordinal()] = createLine(YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, BUCKET_X, BUCKET_Y, BUCKET_HEADING);
        paths[PathState.BUCKET3.ordinal()] = createLine(BUCKET_X, BUCKET_Y, YELLOW_LEFT_X, YELLOW_LEFT_Y, YELLOW_LEFT_HEADING);
        paths[PathState.YELLOW_LEFT.ordinal()] = createLine(YELLOW_LEFT_X, YELLOW_LEFT_Y, NET_ZONE_X, NET_ZONE_Y,SCORE_NET_ZONE_HEADING);
    }

    private void followPath() {
        int index = pathState.ordinal();
        follower.followPath(paths[index]);
        pathState  = PathState.next(index+1);
    }

    private boolean isBusy () {
        return follower.isBusy() ||  robot.isBusy();
    }

    private PathChain createCurve (double startX, double startY, double pointX, double pointY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startX, startY), new Point(pointX, pointY), new Point(endX, endY, Point.CARTESIAN)))
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
}
