package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Auto {

    public static boolean enableLifter = false;
    public static boolean enableDropper = true;
    public static boolean enableWait = false;

    private enum PathState {
        START_YELLOW, BUCKET1, YELLOW_RIGHT, BUCKET2, YELLOW_MIDDLE, BUCKET3, YELLOW_LEFT, BUCKET4, PARK,
        START_BLUE_RED, BAR1, SPECIMEN_RIGHT, OBSERVE_ZONE1, SPECIMEN_MIDDLE, OBSERVE_ZONE2, SPECIMEN_LEFT, OBSERVE_ZONE3, PICKUP_ZONE1, BAR2, PICKUP_ZONE2, BAR3, PICKUP_ZONE3, BAR4;

        public static PathState next(int id) {
            return values()[id];
        }
        private static int index(PathState state)  { return state.ordinal();}
    }

    private PathState pathState;
    int pathCount = PathState.values().length;
    private final Pose[] paths = new Pose[pathCount];

    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    boolean running;

    private final LinearOpMode opMode;
    private final Robot robot;

    private final DriveControl navigator;

    private double targetX;
    private double targetY;

    public Auto(LinearOpMode opMode) {

        this.opMode = opMode;
        robot = new Robot(opMode);
        navigator = robot.getDriveControl();
    }

    public void  runSamplesAuto() {

        running = true;
        elapsedTime.reset();
        while (running && opMode.opModeIsActive()) {

            PathState currentState = pathState;
            Logger.message("\n**\n** %s starts,  time: %6.2f", currentState, elapsedTime.seconds());

            switch (pathState) {
                case START_YELLOW:
                    setStartingPose();
                    robot.setToStartPosition();
                    followPath();
                    break;

                case BUCKET1:
                case BUCKET2:
                case BUCKET3:
                case BUCKET4:
                    //waitUntilNotMoving();
                    waitUntilRobotIdIdle();
                    waitForButtonPress();
                    waitUntilOkToLift();
                    if (enableLifter)
                        robot.lifterUp();
                    waitUntilRobotIdIdle();
                    if (enableDropper)
                        robot.dropSampleInTopBucket();
                    waitUntilRobotIdIdle();
                    robot.lifterDown();
                    opMode.sleep(500);
                    followPath();
                    break;

                case YELLOW_RIGHT:
                case YELLOW_MIDDLE:
                    robot.armMoveTo(robot.ARM_AUTO_PICK, robot.ARM_HIGH_SPEED);
                    waitUntilRobotIdIdle();
                    waitUntilNotMoving();
                    waitForButtonPress();
                    robot.pickUpSample();
                    waitUntilRobotIdIdle();
                    robot.armMoveTo(robot.ARM_EXCHANGE, robot.ARM_HIGH_SPEED);
                    followPath();
                    robot.moveSampleToDropper();
                    break;

                case YELLOW_LEFT:
                    waitUntilNotMoving();
                    robot.pickerRotateTo(robot.PICKER_YAW_90_DEGREES);
                    robot.armMoveTo(robot.ARM_AUTO_PICK);
                    opMode.sleep(200);
                    waitUntilRobotIdIdle();
                    waitForButtonPress();
                    robot.pickUpRotatedSample();
                    followPath();
                    waitUntilRobotIdIdle();
                    robot.moveSampleToDropper();
                    break;

                case PARK:
                    robot.setToStopPosition();
                    waitUntilRobotIdIdle();
                    running = false;
                    break;
            }
            Logger.message("\n** %s ends,  time: %6.2f\n**", currentState, elapsedTime.seconds());
        }
        Logger.message("elapsed time %4.1f", elapsedTime.seconds());
    }

    public void runSpecimensAuto() {

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
                    robot.pickUpSample();
            }
        }
    }

    public void buildSamplePaths(
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
            double parkX,
            double parkY,
            double parkHeading) {

        paths[PathState.index(PathState.START_YELLOW)]  = createPose(startX, startY, startHeading);
        paths[PathState.index(PathState.BUCKET1)]       = createPose(bucketX, bucketY, bucketHeading);
        paths[PathState.index(PathState.YELLOW_RIGHT)]  = createPose(yellowRightX, yellowRightY, yellowRightHeading);
        paths[PathState.index(PathState.BUCKET2)]       = createPose(bucketX, bucketY, bucketHeading);
        paths[PathState.index(PathState.YELLOW_MIDDLE)] = createPose(yellowMiddleX, yellowMiddleY, yellowMiddleHeading);
        paths[PathState.index(PathState.BUCKET3)]       = createPose(bucketX, bucketY, bucketHeading);
        paths[PathState.index(PathState.YELLOW_LEFT)]   = createPose(yellowLeftX, yellowLeftY, yellowLeftHeading);
        paths[PathState.index(PathState.BUCKET4)]       = createPose(bucketX, bucketY, bucketHeading);
        paths[PathState.index(PathState.PARK)]          = createPose(parkX, parkY, parkHeading);
        
        pathState = PathState.START_YELLOW;
    }

    public void buildSpecimenPaths(
            double startX,
            double startY,
            double startHeading,

            double barX,
            double barY,
            double barHeading,

            double observeZoneX,
            double observeZoneY,
            double observeHeading,

            double pickupX,
            double pickupY,
            double pickupHeading,

            double specimenRightX,
            double specimenRightY,
            double specimenRightHeading,

            double specimenMiddleX,
            double specimenMiddleY,
            double specimenMiddleHeading,

            double specimenLeftX,
            double specimenLeftY,
            double specimenLeftHeading) {

        paths[PathState.index(PathState.START_BLUE_RED)]  = createPose(startX, startY, startHeading);
        paths[PathState.index(PathState.BAR1)]            = createPose(barX, barY, barHeading);
        paths[PathState.index(PathState.SPECIMEN_RIGHT)]  = createPose(specimenRightX, specimenRightY, specimenRightHeading);
        paths[PathState.index(PathState.OBSERVE_ZONE1)]   = createPose(observeZoneX, observeZoneY, observeHeading);
        paths[PathState.index(PathState.SPECIMEN_MIDDLE)] = createPose(specimenMiddleX, specimenMiddleY, specimenMiddleHeading);
        paths[PathState.index(PathState.OBSERVE_ZONE2)]   = createPose(observeZoneX, specimenMiddleY, observeHeading);
        paths[PathState.index(PathState.SPECIMEN_LEFT)]   = createPose(specimenLeftX, specimenLeftY, specimenLeftHeading);
        paths[PathState.index(PathState.OBSERVE_ZONE3)]   = createPose(observeZoneX, specimenLeftY, observeHeading);
        paths[PathState.index(PathState.PICKUP_ZONE1)]    = createPose(pickupX, pickupY, pickupHeading);
        paths[PathState.index(PathState.BAR2)]            = createPose(barX, barY+5, barHeading);
        paths[PathState.index(PathState.PICKUP_ZONE2)]    = createPose(pickupX, pickupY, pickupHeading);
        paths[PathState.index(PathState.BAR3)]            = createPose(barX, barY+10, barHeading);

        pathState = PathState.START_BLUE_RED;

    }
    
    public void setStartingPose() {
        int index = pathState.ordinal();
        Pose pose = paths[index];
        Logger.message("path %s index %d  (%.0f, %.0f) heading: %.0f",
                pathState, index, pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        navigator.setPose(new Pose(pose.getX(), pose.getY(), pose.getHeading()));
    }

    private void followPath() {

        waitUntilOkToMove();

        int index = pathState.ordinal() + 1;
        pathState  = PathState.next(index);
        Pose currentPath = paths[index];
        if (currentPath == null) {
            Logger.warning("path for %s missing", pathState);
            return;
        }
        targetX = currentPath.getX();
        targetY = currentPath.getY();
        double heading = Math.toDegrees(currentPath.getHeading());

        Logger.message("path %s index %d  (%.1f, %.1f) heading: %.0f", pathState, index, targetX, targetY, heading);
        
        robot.moveToCoordinate(targetX, targetY, heading, 4000);
    }

    private void waitUntilNotMoving () {
        Logger.message("waiting, navigator is %b", navigator.isBusy());
        if (!navigator.isBusy() )
            Logger.warning("navigator is not busy");
        timer.reset();
        while (navigator.isBusy() &&  opMode.opModeIsActive()) {
            displayPose();

            if (timer.milliseconds() > 3000) {
                Logger.warning("navigator timed out");
                break;
            }
        }
        Pose pose = navigator.getPose();
        Logger.message("pose: %5.1f %5.1f  target: %5.1f %5.1f  busy %b", pose.getX(), pose.getY(), targetX, targetY, navigator.isBusy());
        Logger.message("done waiting");
    }

    private void waitUntilRobotIdIdle() {
        Logger.message("waiting");
        timer.reset();
        while (robot.isBusy() && opMode.opModeIsActive()) {
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
            if (timer.milliseconds() > 5000) {
                Logger.warning("robot timed out");
                break;
            }
        }
        Logger.message("done waiting");
    }

    private void waitUntilOkToLift() {
        Logger.message("waiting");
        timer.reset();
        while (opMode.opModeIsActive()) {
            if (navigator.nearPose() ) {
                break;
            } else {
                Thread.yield();
            }
            if (timer.milliseconds() > 5000) {
                Logger.warning("robot timed out");
                break;
            }
        }
        Logger.message("done waiting");
    }

    private Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, Math.toRadians(heading));
    }

    private void displayPose () {
        Pose pose = navigator.getPose();
        opMode.telemetry.addData("Path", pathState);
        opMode.telemetry.addData("pose", "x %5.1f  y %5.1f  heading %5.1f  is busy %b",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()),
                navigator.isBusy());
        opMode.telemetry.update();
    }

    private void waitForButtonPress() {

        if (! enableWait) return;

        Logger.message("waiting for button press");

        boolean pressed = false;
        while (opMode.opModeIsActive()) {
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

}

