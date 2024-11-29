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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Semaphore;

@SuppressWarnings("FieldCanBeLocal")
@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Robot extends Thread {

    // arm extender
    public final int    ARM_IN = 0;
    public final int    ARM_OUT = 2000;
    public final int    AMR_OUT_PART_WAY = 750;
    public final int    ARM_EXCHANGE = 511;
    public static double ARM_SPEED = 0.5;

    // lifter
    public static double LIFTER_SPEED = 0.90;
    public static double LIFTER_SPEED_LOW = 0.20;
    public static int    LIFTER_STOP_TICKS = 500;
    public static int    LIFTER_UP_POSITION = 3228;
    public static int    LIFTER_DOWN_POSITION = 0;

    // Grabbers
    private final double PICKER_UP_POSITION = 0.383;
    private final double PICKER_DOWN_POSITION = 0.497;

    private final double PICKER_FINGER_CLOSED = 0.51;
    private final double PICKER_FINGER_OPEN = 0.189 ;

    private final double DROPPER_UP_POSITION = 0.616;
    private final double DROPPER_DROP_POSITION = 0.572;
    private final double DROPPER_DOWN_POSITION = 0.492;

    private final double DROPPER_FINGER_CLOSED = 0.475;
    private final double DROPPER_FINGER_OPEN = 0.60;

    private boolean pickerOpened = true;
    private boolean dropperOpened = false;

    // Define Motor and Servo objects
    private DcMotorEx       lifter;
    private DcMotor         extendingArm;
    private Servo           pickerWrist;
    private Servo           pickerFingers;
    private Servo           dropperWrist;
    private Servo           dropperFingers;

    private MotorControl    lifterControl;
    private MotorControl    extendingArmControl;

    // drivetrain
    public Drive      drive = null;

    // Declare OpMode members.
    private final LinearOpMode opMode;

    private enum ROBOT_STATE { IDLE, SET_TO_START_POSITION, PICKUP_SAMPLE, DROP_SAMPLE_INTO_TOP_BUCKET }
    private ROBOT_STATE robotState = ROBOT_STATE.IDLE;

    private int pickingPosition = AMR_OUT_PART_WAY;

    Semaphore okToMove;

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
        okToMove = new Semaphore(1);

        try {
            pickerWrist = opMode.hardwareMap.get(Servo.class, Config.PICKER_WRIST);
            pickerFingers = opMode.hardwareMap.get(Servo.class, Config.PICKER_FINGERS);

            dropperWrist = opMode.hardwareMap.get(Servo.class, Config.DROPPER_WRIST);
            dropperFingers = opMode.hardwareMap.get(Servo.class, Config.DROPPER_FINGERS);

        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }

        try {
            extendingArm = opMode.hardwareMap.get(DcMotor.class, Config.ARM);
            extendingArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendingArm.setDirection(DcMotorSimple.Direction.REVERSE);
            extendingArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendingArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            extendingArmControl = new MotorControl(opMode, extendingArm);
            extendingArmControl.setRange(ARM_IN, ARM_OUT);
            extendingArmControl.setName("extendingArm");
            extendingArmControl.start();

        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }

        try {
            lifter = opMode.hardwareMap.get(DcMotorEx.class, Config.LIFTER);
            lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lifter.setDirection(DcMotorSimple.Direction.REVERSE);
            lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            lifterControl = new MotorControl(opMode, lifter);
            lifterControl.setRange(LIFTER_DOWN_POSITION, LIFTER_UP_POSITION);
            lifterControl.setLowSpeedThreshold(LIFTER_STOP_TICKS);
            lifterControl.setName("lifter");
            lifterControl.start();

        } catch (Exception e) {
            testRobot = true;
            Logger.error(e, "hardware not found");
        }

        if (! testRobot)
            start();
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
                    Logger.message("** Set start position");
                    synchronized (this) {
                        dropperDown();
                        armMoveTo(pickingPosition);
                        delay(1000);
                        pickerOpen();
                        pickerDown();
                        dropperOpen();
                        robotState = ROBOT_STATE.IDLE;
                    }
                    continue;

                case PICKUP_SAMPLE:
                    Logger.message("** Pickup sample");
                    synchronized (this) {
                        dropperOpen();
                        dropperDown();
                        pickerClose();
                        opMode.sleep(750);
                        setOkToMove(true);
                        pickerUp();
                        opMode.sleep(1000);
                        while (lifterIsBusy() && opMode.opModeIsActive()) {
                            delay(10);
                        }
                        dropperClose();
                        delay(400);          // wait for the dropper to get to the closed position
                        pickerOpen();
                        delay(400);
                        dropperUp();
                        dropperOpen();
                        pickerOpen();
                        pickerDown();
                        robotState = ROBOT_STATE.IDLE;
                    }
                    continue;

                case DROP_SAMPLE_INTO_TOP_BUCKET:
                    Logger.message("\n** Drop sample into top bucket");
                    synchronized (this) {
                        lifterUp();
                        while (lifterIsBusy() && opMode.opModeIsActive()) {
                            delay(10);
                        }
                        dropperDropPosition();
                        delay(300);
                        dropperOpen();
                        delay(200);
                        dropperUp();
                        setOkToMove(true);
                        armMoveTo(ARM_EXCHANGE);
                        lifterDown();
                        robotState = ROBOT_STATE.IDLE;
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
     * Returns true if the lifter is extendable
     * @return true if extendable, false if fully extend
     */
    public boolean lifterExtendable() {
        int position = lifter.getCurrentPosition();
        boolean extendable = position < LIFTER_UP_POSITION;
        Logger.message("lifter position %d %B", position, extendable);
        return extendable;
    }

    /**
     * Returns true if the lifter is retractable from its current position
     * @return true if retractable, false if all the way down
     */
    public boolean lifterRetractable() {
        int position = lifter.getCurrentPosition();
        boolean retractable = position > LIFTER_DOWN_POSITION;
        Logger.message("lifter position %d %B", position, retractable);
        return  retractable;
    }

    /**
     * Extend lifter to the specified position
     */
    public void LifterExtend() {
        if (lifterExtendable())
            lifter.setPower(LIFTER_SPEED);
    }

    /**
     * Retract lifter to the specified position
     */
    public void lifterRetract () {
        if (lifterRetractable())
            lifter.setPower(-LIFTER_SPEED);
    }

    /**
     * Stop the lifter from moving
     */
    public void lifterStop () {
        lifter.setPower(0);
    }

    /**
     * Raise the lifter to the specified position
     */
    public void lifterUp() {
        Logger.message("set lifter to position %d at %4.2f speed", LIFTER_UP_POSITION, LIFTER_SPEED);

        //runMotorToPosition(lifter, LIFTER_UP_POSITION, LIFTER_SPEED);
        lifterControl.setPosition(LIFTER_UP_POSITION, LIFTER_SPEED, LIFTER_SPEED);
    }

    /**
     * Lower the lifter to its down position
     */
    public void lifterDown() {
        //runMotorToPosition(lifter, LIFTER_DOWN_POSITION, LIFTER_SPEED, LIFTER_SPEED_LOW);
        lifterControl.setPosition(LIFTER_DOWN_POSITION, LIFTER_SPEED, LIFTER_SPEED_LOW);
    }

    /**
     * Check if the lifter is moving
     * @return true if the lifter is moving
     */
    public boolean lifterIsBusy () {
        return lifterControl.motorIsBusy();
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

    /**
     * Move the arm to the specified position
     * @param position target position
     */
    public void armMoveTo (int position) {
        extendingArmControl.setPosition(position, ARM_SPEED, ARM_SPEED);
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
        Logger.message("set picker to position %f", PICKER_UP_POSITION);
        pickerWrist.setPosition(PICKER_UP_POSITION);
    }

    public void pickerDown() {
        Logger.message("set picker to position %f", PICKER_DOWN_POSITION);
        pickerWrist.setPosition(PICKER_DOWN_POSITION);
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

    public void dropperUp() {
        Logger.message("set dropper to position %f", DROPPER_UP_POSITION);
        dropperWrist.setPosition(DROPPER_UP_POSITION);
    }

    public void dropperDropPosition () {
        Logger.message("set dropper to position %f", DROPPER_DOWN_POSITION);
        dropperWrist.setPosition(DROPPER_DROP_POSITION);
    }

    public void dropperDown() {
        Logger.message("set dropper to position %f", DROPPER_DOWN_POSITION);
        dropperWrist.setPosition(DROPPER_DOWN_POSITION);
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

        if (testRobot) return false;
        return lifterControl.motorIsBusy() || extendingArmControl.motorIsBusy();
    }

    public void setToStartPosition() {
        synchronized (this) {
            robotState = ROBOT_STATE.SET_TO_START_POSITION;
        }
    }

    public void setToPickingPosition(int position) {
        synchronized (this) {
            pickingPosition = position;
            robotState = ROBOT_STATE.SET_TO_START_POSITION;
        }
    }

    public void pickUpYellow() {
        synchronized (this) {
            robotState = ROBOT_STATE.PICKUP_SAMPLE;
            setOkToMove(false);
        }
    }

    public  void dropSampleInTopBucket() {
        synchronized (this) {
            robotState = ROBOT_STATE.DROP_SAMPLE_INTO_TOP_BUCKET;
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

    public void dropTest () {
        // ToDo remove, for testing only
        dropperDropPosition();
        opMode.sleep(300);
        dropperOpen();
        opMode.sleep(200);
        dropperUp();
    }

    public boolean isTestRobot () {
        return testRobot;
    }
} // end of class

