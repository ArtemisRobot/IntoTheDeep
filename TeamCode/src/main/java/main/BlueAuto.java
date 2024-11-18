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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import common.Robot;

@Autonomous(name="BlueBucketAuto", group = "Main")

 public class BlueAuto extends LinearOpMode {

    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING = 0;

    public static double BUCKET_X = 8;
    public static double BUCKET_Y = 25;
    public static double BUCKET_HEADING = 0;

    private static enum PathState {
        START(0),
        BUCKET1(1),
        YELLOW_RIGHT(2),
        BUCKET2(3),
        YELLOW_MIDDLE(4),
        BUCKET3(5),
        YELLOW_LEFT(6),
        SCORE_NET_ZONE(7);

        private final int value;

        PathState(int value) {
            this.value = value;
        }
        public int getValue() {
            return value;
        }
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
    }

    private void followPath() {
        int index = pathState.ordinal();
        follower.followPath(paths[index]);
        pathState  = PathState.next(index+1);
    }

    private boolean isBusy () {
        if (follower.isBusy())
            return true;
         else
            return robot.isBusy();
    }

    private PathChain createCurve (double startX, double startY, double pointX, double pointY, double endX, double endY, double heading) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startX, startX), new Point(pointX, pointY), new Point(endX, endY, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(3)
                .build();
    }
}
