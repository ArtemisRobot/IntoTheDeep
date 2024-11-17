package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import common.Logger;

@com.acmerobotics.dashboard.config.Config

@TeleOp(name = "Route Builder", group = "Test")
public class RouteBuilder extends LinearOpMode {

    public static long WAIT_TIME_BETWEEN_LINES = 2;

    public static double BEGIN_X = 8.5;
    public static double BEGIN_Y = 85;
    public static double BEGIN_HEADING = 0;

    public static double LINE_1_END_POINT_X = 15;
    public static double LINE_1_END_POINT_Y = 85;
    public static double LINE_1_HEADING = 0;

    public static boolean LINE_2_ENABLED = true;
    public static double LINE_2_END_POINT_X = 15;
    public static double LINE_2_END_POINT_Y = 128;
    public static double LINE_2_HEADING = 45;

    public static boolean LINE_3_ENABLED = false;
    public static double LINE_3_END_POINT_X = 0;
    public static double LINE_3_END_POINT_Y = 0;
    public static double LINE_3_HEADING = 0;

    public static boolean LINE_4_ENABLED = false;
    public static double LINE_4_END_POINT_X = 0;
    public static double LINE_4_END_POINT_Y = 0;
    public static double LINE_4_HEADING = 0;

    ElapsedTime timer = new ElapsedTime();

    Pose startPose;
    private final Path[] paths = new Path[5];
    int pathCount;

    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void runOpMode() {
    try {
        buildPaths();
        initialize();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                followPaths();
                while (gamepad1.x) sleep(10);
            }

            //drive();
        }

    } catch (Exception e) {
        e.printStackTrace();
        }
    }

    private void drive() {
        Pose position = follower.getClosestPose();
        telemetry.addData("Position ", position);
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
    }


    private void initialize() {

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower.setStartingPose(startPose);
        //follower.startTeleopDrive();
    }

    /**
     * Follow the path specified in the FTC Dashboard.
     */
    private void followPaths () {

        for (int i=0; i<pathCount; i++) {
            follower.followPath(paths[i]);
            Logger.message("path %d.  %5.1f %5.1f to %5.1f %5.1f",
                    i+1,
                    paths[i].getFirstControlPoint().getX(),
                    paths[i].getFirstControlPoint().getY(),
                    paths[i].getLastControlPoint().getX(),
                    paths[i].getLastControlPoint().getY());
            timer.reset();

            while (timer.seconds() < WAIT_TIME_BETWEEN_LINES) {
                Thread.yield();
                follower.update();
                follower.resetOffset();
            }
        }
    }

    /**
     * Build a path from the values specified in the FTC Dashboard.
     */
    private void buildPaths() {

        startPose = new Pose(BEGIN_X, BEGIN_Y, BEGIN_HEADING);

        pathCount = 0;
        paths[pathCount] = new Path(new BezierCurve(new Point(BEGIN_X, BEGIN_Y), new Point(LINE_1_END_POINT_X, LINE_1_END_POINT_Y, Point.CARTESIAN)));
        paths[pathCount].setConstantHeadingInterpolation(Math.toRadians(LINE_1_HEADING));
        paths[pathCount].setPathEndTimeoutConstraint(3);
        pathCount++;

        if (LINE_2_ENABLED) {
            paths[pathCount] = new Path(new BezierCurve(paths[pathCount-1].getLastControlPoint(), new Point(LINE_2_END_POINT_X, LINE_2_END_POINT_Y, Point.CARTESIAN)));
            paths[pathCount].setConstantHeadingInterpolation(Math.toRadians(LINE_2_HEADING));
            paths[pathCount].setPathEndTimeoutConstraint(3);
            pathCount++;
        }

        if (LINE_3_ENABLED) {
            paths[pathCount] = new Path(new BezierCurve(paths[pathCount-1].getLastControlPoint(), new Point(LINE_3_END_POINT_X, LINE_3_END_POINT_Y, Point.CARTESIAN)));
            paths[pathCount].setConstantHeadingInterpolation(Math.toRadians(LINE_3_HEADING));
            paths[pathCount].setPathEndTimeoutConstraint(3);
            pathCount++;
        }

        if (LINE_4_ENABLED) {
            paths[pathCount] = new Path(new BezierCurve(paths[pathCount-1].getLastControlPoint(), new Point(LINE_4_END_POINT_X, LINE_4_END_POINT_Y, Point.CARTESIAN)));
            paths[pathCount].setConstantHeadingInterpolation(Math.toRadians(LINE_4_HEADING));
            paths[pathCount].setPathEndTimeoutConstraint(3);
            pathCount++;
        }
    }
}
