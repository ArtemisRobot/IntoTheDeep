package test.code;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import common.Config;
import common.Drive;
import common.Logger;

@com.acmerobotics.dashboard.config.Config
@Disabled
@TeleOp(name = "Route Builder", group = "Test")
public class RouteBuilder extends LinearOpMode {

    public static long WAIT_TIME_BETWEEN_LINES = 2;

    public static double BEGIN_X = 133.5;
    public static double BEGIN_Y = 39.5;
    public static double BEGIN_HEADING = 180;

    public static double LINE_1_END_POINT_X = 125;
    public static double LINE_1_END_POINT_Y = 15;
    public static double LINE_1_HEADING = 180;

    public static boolean LINE_2_ENABLED = false;
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

    public static double distanceFromObject = 4;

    ElapsedTime timer = new ElapsedTime();

    private DistanceSensor distanceSensor;

    Pose startPose;
    private final Path[] paths = new Path[5];
    int pathCount;

    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    Drive drive;
    private SparkFunOTOS otos;


    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void runOpMode() {
    try {

        drive = new Drive(this);
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        initialize();

        startPose = new Pose(BEGIN_X, BEGIN_Y, Math.toRadians(BEGIN_HEADING));
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.4);
        follower.startTeleopDrive();

        buildPaths();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                followPaths();
                while (gamepad1.x) sleep(10);
            }

            if (gamepad1.b) {
                driveToObject2(distanceFromObject);
                while (gamepad1.b) sleep(10);
            }

            displayPose();
            driveWithStick();
        }

    } catch (Exception e) {
        e.printStackTrace();
        }
    }

    private void driveWithStick() {
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

        try {
            distanceSensor = hardwareMap.get(DistanceSensor.class, Config.DISTANCE_SENSOR);
        } catch (Exception e) {
            Logger.error(e, "Distance sensor not found");
        }
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
                displayPose();
                follower.resetOffset();
            }
        }
    }

    /**
     * Build a path from the values specified in the FTC Dashboard.
     */
    private void buildPaths() {

        pathCount = 0;
        paths[pathCount] = new Path(new BezierLine(new Point(BEGIN_X, BEGIN_Y), new Point(LINE_1_END_POINT_X, LINE_1_END_POINT_Y, Point.CARTESIAN)));
        paths[pathCount].setConstantHeadingInterpolation(Math.toRadians(LINE_1_HEADING));
        paths[pathCount].setPathEndTimeoutConstraint(3);
        pathCount++;

        if (LINE_2_ENABLED) {
            paths[pathCount] = new Path(new BezierCurve(paths[pathCount-1].getLastControlPoint(), new Point(LINE_2_END_POINT_X, LINE_2_END_POINT_Y, Point.CARTESIAN)));
            paths[pathCount].setConstantHeadingInterpolation(Math.toRadians(LINE_2_HEADING));
            paths[pathCount].setPathEndTimeoutConstraint(500);
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

    private void displayPose () {
        Pose pose = follower.getPose();
        SparkFunOTOS.Pose2D rawPose = otos.getPosition();

        telemetry.addData("pose", "x %5.1f  y %5.1f  heading %5.1f",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("rawPose", "x %5.1f  y %5.1f  heading %5.1f", rawPose.x, rawPose.y, rawPose.h);
        telemetry.addData("distance", "%5.1f", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("follower is busy", follower.isBusy());
        telemetry.update();
        //Logger.message("pose x %4.0f  y %4.0f  heading %4.0f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private void driveToObject(double distanceFromObject) {
        Pose pose = follower.getPose();
        double  gap = distanceSensor.getDistance(DistanceUnit.INCH);
        double travel = gap - distanceFromObject;
        double x = pose.getX();
        double y =  pose.getX();

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(x, y, Point.CARTESIAN), new Point(x + travel, y, Point.CARTESIAN)))
                .setPathEndHeadingConstraint(pose.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();


        follower.followPath(path);
        while (follower.isBusy() && opModeIsActive()) {
            displayPose();
            follower.update();
        }
    }
    private void driveToObject2(double distanceFromObject) {
        drive.moveToObject(0.25, distanceFromObject, 2000);
        follower.update();
    }



}
