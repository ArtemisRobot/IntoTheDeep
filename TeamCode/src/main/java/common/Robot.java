package common;

/*
 * This file defines a Java Class that performs all the setup and configuration for the robot's hardware (motors and sensors).
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

import java.util.concurrent.Semaphore;

@SuppressWarnings("FieldCanBeLocal")
@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Robot extends Thread {

    // lifter
    public static double LIFTER_SPEED = 0.50;
    public static double LIFTER_UP_SPEED = 0.70;
    public static double LIFTER_DOWN_SPEED = 0.50;

    public static double LIFTER_SPEED_LOW = 0.20;
    public static int    LIFTER_STOP_TICKS = 500;
    public static int    LIFTER_UP_POSITION = 1614;
    public static int    LIFTER_DOWN_POSITION = 0;
    public static int    LIFTER_TOP_BAR_POSITION = 0;

    // arm extender
    public final int    ARM_IN = 0;
    public final int    ARM_OUT = 2000;
    public final int    AMR_OUT_PART_WAY = 750;
    public final int    ARM_OUT_START = 190;
    public final int    ARM_EXCHANGE = 230;
    public final int    ARM_AUTO_PICK = 617;
    public final double ARM_SPEED = 0.5;
    public final double ARM_HIGH_SPEED = 0.75;

    // Grabbers
    private final double PICKER_UP_POSITION    = 0.150;
    private final double PICKER_STORE_POSITION = 0.094;
    private final double PICKER_DOWN_POSITION  = 0.259;

    private final double PICKER_FINGER_CLOSED  = 0.420;
    private final double PICKER_FINGER_OPEN    = 0.600;

    public  final double PICKER_YAW_0_DEGREES  = 0.167;
    public  final double PICKER_YAW_45_DEGREES = 0.320;
    public  final double PICKER_YAW_90_DEGREES = 0.494;

    private final double DROPPER_UP_POSITION   = 0.616;
    private final double DROPPER_DROP_POSITION = 0.552;
    private final double DROPPER_DOWN_POSITION = 0.495;
    private final double DROPPER_SPECIMEN_UP   = 0.550;
    private final double DROPPER_SPECIMEN_DOWN = 0.550;

    private final double DROPPER_FINGER_CLOSED = 0.463;
    private final double DROPPER_FINGER_OPEN   = 0.58;

    private final double SPECIMEN_STOP_POSTION = 5;

    private boolean pickerOpened = true;
    private boolean dropperOpened = false;
    private boolean pickerUp = false;
    private boolean dropperUp = false;
    private double pickerYawPosition = PICKER_YAW_0_DEGREES;

    // Define Motor and Servo objects
    private Servo           pickerWrist;
    private Servo           pickerFingers;
    private Servo           pickerYaw;
    private Servo           dropperWrist;
    private Servo           dropperFingers;

    private Lifter          lifter;

    private DcMotor         extendingArm;
    private MotorControl    extendingArmControl;

    // drivetrain
    public Drive      drive = null;
    private DriveGamepad driveGamepad;
    private DriveControl driveControl;

    // Declare OpMode members.
    private final LinearOpMode opMode;

    private enum ROBOT_STATE { IDLE, SET_TO_START_POSITION, SET_TO_STOP_POSITION, PICKUP_SAMPLE, MOVE_SAMPLE_TO_DROPPER, DROP_SAMPLE_INTO_TOP_BUCKET, SCORE_SPECIMEN }
    private ROBOT_STATE robotState = ROBOT_STATE.IDLE;

    private int pickingPosition = AMR_OUT_PART_WAY;

    private Semaphore okToMove;

    boolean testRobot = false;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    /**
     * Initialize the Robot
     */
    public void init() {
        this.setName("robot");
        drive = new Drive(opMode);

        driveControl = new DriveControl(opMode, drive);
        driveControl.start();

        driveGamepad = new DriveGamepad(opMode, driveControl);

        okToMove = new Semaphore(1);

        try {
            pickerWrist = opMode.hardwareMap.get(Servo.class, Config.PICKER_WRIST);
            //pickerWrist = new ServoEx(ServoEx.ServoType.TORQUE_5_TURN, opMode.hardwareMap, Config.PICKER_WRIST);
            pickerFingers = opMode.hardwareMap.get(Servo.class, Config.PICKER_FINGERS);
            pickerYaw = opMode.hardwareMap.get(Servo.class, Config.PICKER_YAW);

            dropperWrist = opMode.hardwareMap.get(Servo.class, Config.DROPPER_WRIST);
            //dropperWrist = new ServoEx(ServoEx.ServoType.TORQUE_5_TURN, opMode.hardwareMap, Config.DROPPER_WRIST);
            dropperFingers = opMode.hardwareMap.get(Servo.class, Config.DROPPER_FINGERS);

        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }

        try {
            extendingArm = opMode.hardwareMap.get(DcMotor.class, Config.ARM);
            extendingArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendingArm.setDirection(DcMotorSimple.Direction.REVERSE);
            extendingArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);       // ToDo should we do this?
            extendingArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            extendingArmControl = new MotorControl(opMode, extendingArm);
            extendingArmControl.setRange(ARM_IN, ARM_OUT);
            extendingArmControl.setName("extendingArm");
            extendingArmControl.start();

        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }

        try {
            lifter = new Lifter(opMode);
            lifter.setRange(LIFTER_DOWN_POSITION, LIFTER_UP_POSITION);
            lifter.setLowSpeedThreshold(LIFTER_STOP_TICKS);
            lifter.start();

        } catch (Exception e) {
            testRobot = true;
            Logger.error(e, "hardware not found");
        }

        if (! testRobot)
            start();
    }

    public void startDriveGamepad() {
        driveGamepad.start();
    }

    public void run () {
        Logger.message("robot thread started");

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
            switch (robotState) {
                case IDLE:
                    Thread.yield();
                    continue;

                case SET_TO_START_POSITION:
                    Logger.message("\n** Set to start position");
                    synchronized (this) {
                        double  time = System.currentTimeMillis();
                        dropperClose();
                        dropperUp();
                        armMoveTo(ARM_OUT_START, ARM_HIGH_SPEED);
                        while (armIsBusy()) delay(10);
                        pickerRotateTo(PICKER_YAW_0_DEGREES);
                        pickerOpen();
                        pickerDown();
                        armMoveTo(pickingPosition, ARM_HIGH_SPEED);
                        waitUnitTime(time + 750);       // make sure the dropper is in the up position
                        robotState = ROBOT_STATE.IDLE;
                    }
                    continue;

                case SET_TO_STOP_POSITION:
                    Logger.message("\n** Set to stop position");
                    synchronized (this) {
                        dropperOpen();
                        dropperDown();
                        pickerDown();
                        pickerRotateTo(PICKER_YAW_90_DEGREES);
                        delay(500);
                        armMoveTo(ARM_IN);
                        robotState = ROBOT_STATE.IDLE;
                    }
                    continue;

                case PICKUP_SAMPLE:
                    Logger.message("\n** Pickup sample");
                    synchronized (this) {
                        dropperOpen();
                        dropperDown();
                        pickerClose();
                        opMode.sleep(400);
                        setOkToMove(true);
                        robotState = ROBOT_STATE.IDLE;
                    }
                    continue;

                case MOVE_SAMPLE_TO_DROPPER:
                    pickerUp();
                    opMode.sleep(850);
                    while (lifterIsBusy() && opMode.opModeIsActive()) {
                        delay(10);
                    }
                    dropperClose();
                    delay(250);          // wait for the dropper to get to the closed position
                    pickerOpen();
                    delay(100);
                    dropperUp();
                    pickerDown();
                    pickerOpen();
                    delay(750);
                    robotState = ROBOT_STATE.IDLE;
                    continue;


                case DROP_SAMPLE_INTO_TOP_BUCKET:
                    Logger.message("\n**  Drop sample into top bucket");
                    synchronized (this) {
                        //ToDo lifterUp();
                        while (lifterIsBusy() && opMode.opModeIsActive()) {
                            delay(10);
                        }
                        dropperDropPosition();
                        delay(300);
                        dropperOpen();
                        delay(200);
                        dropperUp();
                        setOkToMove(true);
                        //ToDo armMoveTo(ARM_EXCHANGE);
                        //ToDo lifterDown();
                        robotState = ROBOT_STATE.IDLE;
                    }
                    continue;

                case SCORE_SPECIMEN:
                    Logger.message("** Score specimen");
                    synchronized (this) {
                        lifterToTopBar();
                        while (lifterIsBusy() && opMode.opModeIsActive()) {
                            delay(10);
                        }

                        dropperSpecimenUp();
                        delay(200);

                        driveControl.moveToObject(SPECIMEN_STOP_POSTION, 2000);
                        while (driveControl.isBusy()&& opMode.opModeIsActive()) {
                            delay(10);
                        }

                        dropperSpecimenDown();
                        delay(200);

                        dropperOpen();
                        delay(100);

                        dropperUp();
                        delay(200);

                        lifterDown();
                        setOkToMove(true);
                    }
            }
        }

        Logger.message("robot thread stopped");
    }

    private void delay (long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Extend lifter to the specified position
     */
    public void LifterExtend() {
        lifter.runLifter(LIFTER_SPEED);
    }

    /**
     * Retract lifter to the specified position
     */
    public void lifterRetract () {
        lifter.runLifter(-LIFTER_SPEED);
    }

    /**
     * Stop the lifter from moving
     */
    public void lifterStop () {
        lifter.stopLifter();
    }

    /**
     * Raise the lifter to the specified position
     */
    public void lifterUp() {
        Logger.message("set lifter to position %d at %4.2f speed", LIFTER_UP_POSITION, LIFTER_UP_SPEED);
         lifter.setPosition(LIFTER_UP_POSITION, LIFTER_UP_SPEED);
    }

    /**
     * Lower the lifter to its down position
     */
    public void lifterDown() {
        lifter.setPosition(LIFTER_DOWN_POSITION, LIFTER_DOWN_SPEED, LIFTER_SPEED_LOW);
    }

    /**
     * Raise the lifter to the specimen top bar scoring position
     */
    public void lifterToTopBar () {
        lifter.setPosition(LIFTER_TOP_BAR_POSITION, LIFTER_SPEED, LIFTER_SPEED);
    }

    /**
     * Check if the lifter is moving
     * @return true if the lifter is moving
     */
    public boolean lifterIsBusy () {
        return lifter.lifterIsBusy();
    }

    /**
     * Returns true if the arm is extendable
     * @return true if extendable, false if fully extend
     */
    public boolean armExtendable() {
        int position = extendingArm.getCurrentPosition();
        boolean extendable = position < ARM_OUT;
        Logger.message("arm position %d %B", position, extendable);
        return extendable;
    }

    /**
     * Returns true if the arm is retractable from its current position
     * @return true if fully retracted
     */
    public boolean armRetractable() {
        int position = extendingArm.getCurrentPosition();
        boolean retractable = position > ARM_IN;
        Logger.message("arm position %d %B", position, retractable);
        return  retractable;
    }

    /**
     * Extend arm to the specified position
     */
    public void armExtend() {
        extendingArmControl.runMotor(ARM_SPEED);
    }

    /**
     * Retract arm to the specified position
     */
    public void amrRetract() {
        extendingArmControl.runMotor(-ARM_SPEED);
    }

    /**
     * Stop the arm from moving
     */
    public void armStop () {
        extendingArmControl.stopMotor();
    }

    public boolean armIsBusy() {
        return extendingArmControl.motorIsBusy();
    }
    /**
     * Move the arm to the specified position
     * @param position target position
     */
    public void armMoveTo (int position) {
        extendingArmControl.setPosition(position, ARM_SPEED, ARM_SPEED);
    }

    public void armMoveTo (int position, double speed) {
        extendingArmControl.setPosition(position, speed, speed);
    }

    private void runMotorToPosition(DcMotor motor, int position, double speed, double lowSpeed) {

        ElapsedTime elapsedTime = new ElapsedTime();

        int current = motor.getCurrentPosition();
        int last = current;
        Logger.message("run from %d to %d", current, position);
        DcMotor.RunMode mode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);

        double lastMoveTime = 0;
        elapsedTime.reset();
        while (opMode.opModeIsActive()) {
            if (! motor.isBusy())
                break;
            if (emergencyStop())
                break;

            // if the motor has not moved for a while, kill the power
            current = motor.getCurrentPosition();
            if (current != last) {
                lastMoveTime = elapsedTime.milliseconds();
                last = current;
            } else if (elapsedTime.milliseconds() - lastMoveTime > 100) {
                Logger.message("motor not moving");
                break;
            }

            int remaining = Math.abs(position-current);
            if (remaining < LIFTER_STOP_TICKS && speed != lowSpeed) {
                motor.setPower(lowSpeed);
                Logger.message("remaining %d set to lower speed");
            }

            //Logger.message("position %5d   remaining %5d  elapsed %6.2f ", current, remaining, elapsedTime.milliseconds());
        }
        motor.setPower(0);
        motor.setMode(mode);
    }

    public void pickerUp() {
        Logger.message("set picker to position %f ", PICKER_UP_POSITION);
        pickerWrist.setPosition(PICKER_UP_POSITION);
        pickerUp = true;
    }

    public void pickerStore() {
        Logger.message("set picker to position %f", PICKER_STORE_POSITION);
        pickerRotateTo(PICKER_YAW_90_DEGREES);
        pickerClose();
        pickerDown();
    }

    public void pickerDown() {
        Logger.message("set picker to position %f", PICKER_DOWN_POSITION);
        pickerWrist.setPosition(PICKER_DOWN_POSITION);
        pickerUp = false;
    }

    public void pickerOpen(){
        Logger.message("set picker to position %f", PICKER_FINGER_OPEN);
        pickerFingers.setPosition(PICKER_FINGER_OPEN);
        pickerOpened = true;
    }

    public void pickerClose() {
        Logger.message("set picker to position %f", PICKER_FINGER_CLOSED);
        pickerFingers.setPosition(PICKER_FINGER_CLOSED);
        pickerOpened = false;
    }

    public boolean pickerIsOpen() {
        Logger.message("picker is open: %b", pickerOpened);
        return pickerOpened;
    }

    public boolean pickerIsUp() {
        return pickerUp;
    }

    public void pickerRotate() {

        if (pickerYawPosition == PICKER_YAW_0_DEGREES) {
            pickerYawPosition = PICKER_YAW_45_DEGREES;
        } else if (pickerYawPosition == PICKER_YAW_45_DEGREES) {
            pickerYawPosition = PICKER_YAW_90_DEGREES;
        } else {
            pickerYawPosition = PICKER_YAW_0_DEGREES;
        }
        pickerYaw.setPosition(pickerYawPosition);
    }

    public void pickerRotateTo(double position) {
        pickerYaw.setPosition(position);
        pickerYawPosition = position;
    }

    public void dropperUp() {
        Logger.message("set dropper to position %f", DROPPER_UP_POSITION);
        dropperWrist.setPosition(DROPPER_UP_POSITION);
        dropperUp = true;
    }

    public boolean dropperIsUp () {
        return dropperUp;
    }

    public void dropperDropPosition () {
        Logger.message("set dropper to position %f", DROPPER_DOWN_POSITION);
        dropperWrist.setPosition(DROPPER_DROP_POSITION);
        dropperUp = false;
    }

    public void dropperDown() {
        Logger.message("set dropper to position %f", DROPPER_DOWN_POSITION);
        dropperWrist.setPosition(DROPPER_DOWN_POSITION);
        dropperUp = false;
    }

    public void dropperSpecimenUp() {
        dropperWrist.setPosition(DROPPER_SPECIMEN_UP);
        dropperUp = false;
    }

    public void dropperSpecimenDown() {
        dropperWrist.setPosition(DROPPER_SPECIMEN_DOWN);
        dropperUp = false;
    }

    public void dropperOpen(){
        Logger.message("set dropper to position %f", DROPPER_FINGER_OPEN);
        dropperFingers.setPosition(DROPPER_FINGER_OPEN);
        dropperOpened = true;
    }

    public void dropperClose(){
        Logger.message("set dropper to position %f", DROPPER_FINGER_CLOSED);
        dropperFingers.setPosition(DROPPER_FINGER_CLOSED);
        dropperOpened = false;
    }

    public boolean dropperIsOpen() {
        Logger.message("dropper is open: %b", pickerOpened);
        return dropperOpened;
    }

    public boolean emergencyStop() {
        return opMode.gamepad1.back;
    }

    @SuppressWarnings("unused")
    public void turn(double degrees) {
        drive.turn(degrees);
    }

    @SuppressWarnings("unused")
    public void forward (double distance) {
        drive.forward(distance);
    }

    @SuppressWarnings("unused")
    public void back (double distance) {
        drive.back(distance);
    }

    @SuppressWarnings("unused")
    public void strafeLeft (double distance) { drive.strafeLeft(distance); }

    @SuppressWarnings("unused")
    public void strafeRight (double distance) {
        drive.strafeRight(distance);
    }

    public boolean isBusy () {
        return lifter.lifterIsBusy() || extendingArmControl.motorIsBusy() || robotState != ROBOT_STATE.IDLE;
    }

    public void setToStartPosition() {
        synchronized (this) {
            pickingPosition = ARM_EXCHANGE;
            robotState = ROBOT_STATE.SET_TO_START_POSITION;
        }
    }

    public void setToStopPosition() {
        synchronized (this) {
            robotState = ROBOT_STATE.SET_TO_STOP_POSITION;
        }
    }

    public void setToPickingPosition(int position) {
        synchronized (this) {
            pickingPosition = position;
            robotState = ROBOT_STATE.SET_TO_START_POSITION;
        }
    }

    public void pickUpSample() {
        synchronized (this) {
            robotState = ROBOT_STATE.PICKUP_SAMPLE;
            setOkToMove(false);
        }
    }

    public void moveSampleToDropper () {
        synchronized (this) {
            robotState = ROBOT_STATE.MOVE_SAMPLE_TO_DROPPER;
        }
    }

    public void dropSampleInTopBucket() {
        synchronized (this) {
            robotState = ROBOT_STATE.DROP_SAMPLE_INTO_TOP_BUCKET;
            setOkToMove(false);
        }
    }

    public void scoreSpecimen() {
        synchronized (this) {
            robotState = ROBOT_STATE.SCORE_SPECIMEN;
            setOkToMove(false);
        }
    }

    public void pushSample() {

    }

    public boolean okToMove () {
        return okToMove.availablePermits() == 1;
    }

    public void waitUntilOkToMove () {
        if (okToMove.availablePermits() != 1) {
            Logger.message("wait until ok to move");
            while (okToMove.availablePermits() != 1)
                delay(1);                       // ToDo this a better why
            Logger.message("ok to move, continue");
        }
    }

    private void waitUnitTime(double time) {
        while (System.currentTimeMillis() < time)
            delay(1);
    }

    private void setOkToMove(boolean ok)  {

        if (testRobot) return;

        try {
            if (ok) {
                Logger.message("ok to move");
                okToMove.release();
            } else {
                Logger.message("not ok to move");
                okToMove.acquire();
            }
        } catch (InterruptedException e) {
            Logger.message("InterruptedException");
        }
    }

    public void moveToCoordinate(double targetX, double targetY, double targetHeading, double timeout) {
        driveControl.moveToCoordinate(targetX, targetY, targetHeading, timeout);
    }

    public boolean driveIsBusy() {
        return driveControl.isBusy();
    }

    public DriveControl getDriveControl () {
        return driveControl;
    }

    public void dropTest () {
        // ToDo remove, for testing only
        dropperDropPosition();
        opMode.sleep(300);
        dropperOpen();
        opMode.sleep(200);
        dropperUp();
    }

} // end of class

