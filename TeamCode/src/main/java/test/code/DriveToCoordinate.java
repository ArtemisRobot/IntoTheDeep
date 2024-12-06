package test.code;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import common.Config;
import common.Drive;
import common.Logger;

@com.acmerobotics.dashboard.config.Config

@TeleOp(name = "DriveToCoordinate\n", group = "Test")
public class DriveToCoordinate extends LinearOpMode {

    public static long WAIT_TIME_BETWEEN_LINES = 2;

    public static double START_X = 0;
    public static double START_Y = 23;
    public static double START_HEADING = 0;

    public static double TARGET1_X = 8;
    public static double TARGET1_Y = 17;
    public static double TARGET1_HEADING = 225;

    public static double TARGET2_X = 20;
    public static double TARGET2_Y = 20;
    public static double TARGET2_HEADING = 90;

    private double startX;
    private double startY;
    private double startHeading;

    private double target1X;
    private double target1Y;
    private double target1Heading;

    private double target2X;
    private double target2Y;
    private double target2Heading;


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
    //try {

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        drive = new Drive(this);
        drive.start();

        //initialize();
        initPedroPathing(
            START_X,
            START_Y,
            START_HEADING,
            TARGET1_X,
            TARGET1_Y,
            TARGET1_HEADING,
            TARGET2_X,
            TARGET2_Y,
            TARGET2_HEADING);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                driveToCoordinate(target1X, target1Y, target1Heading);
                while (gamepad1.x) sleep(10);
            }

            if (gamepad1.b) {
                driveToCoordinate(target2X, target2Y, target2Heading);
                while (gamepad1.b) sleep(10);
            }

            follower.update();
            displayPose();
        }

    //} catch (Exception e) {
    //    e.printStackTrace();
    //}
    }

    private void driveWithStick() {
        Pose position = follower.getClosestPose();
        telemetry.addData("Position ", position);
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
    }

    private void initPedroPathing(
            double startX,
            double startY,
            double startHeading,
            double target1X,
            double target1Y,
            double target1Heading,
            double target2X,
            double target2Y,
            double target2Heading) {

        this.target1X = target1X;
        this.target1Y = target1Y;
        this.target1Heading =target1Heading;
        this.target2X = target2X;
        this.target2Y = target2Y;
        this.target2Heading = target2Heading;

        follower = new Follower(hardwareMap);
        startPose = new Pose(startX, startY, Math.toRadians(startHeading));
        follower.setStartingPose(startPose);
        //follower.setMaxPower(0.7);
    }

    private void initialize() {

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

    private void displayPose () {
        Pose pose = follower.getPose();
        SparkFunOTOS.Pose2D rawPose = otos.getPosition();

        telemetry.addData("pose", "x %5.1f  y %5.1f  heading %5.1f",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("rawPose", "x %5.1f  y %5.1f  heading %5.1f", rawPose.x, rawPose.y, rawPose.h);
        //telemetry.addData("distance", "%5.1f", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("follower is busy", follower.isBusy());
        telemetry.update();
        //Logger.message("pose x %4.0f  y %4.0f  heading %4.0f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }

    private void driveToCoordinatePP(double x, double y, double heading) {

        Pose pose = follower.getPose();
        double currentX = pose.getX();
        double currentY =  pose.getX();
        //follower.setStartingPose(new Pose(startPose.getX(), startPose.getY(), pose.getHeading()));

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(currentX, currentY, Point.CARTESIAN), new Point(x, y, Point.CARTESIAN)))
                .setPathEndHeadingConstraint(Math.toRadians(heading))
                .setPathEndTimeoutConstraint(3)
                .build();

        follower.followPath(path);

        Logger.message("start:  x %5.1f  y %5.1f  heading %5.1f  target: x %5.1f  y %5.1f  heading %5.1f   isBusy %b",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()), x, y, heading, follower.isBusy());

        while (follower.isBusy() && opModeIsActive()) {
            displayPose();
            follower.update();
        }

        pose = follower.getPose();
        Logger.message("end:  x %5.1f  y %5.1f  heading %5.1f  target: x %5.1f  y %5.1f  heading %5.1f",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()), x, y, heading);

    }

    private void driveToObject2(double distanceFromObject) {
        drive.moveToObject(0.25, distanceFromObject, 2000);
        follower.update();
    }

    private void driveToCoordinate(double x, double y, double heading) {
        SparkFunOTOS.Pose2D rawPose = otos.getPosition();
        drive.turnTo(heading);

    }

}
