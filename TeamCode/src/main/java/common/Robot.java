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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import test.code.CalibrateMotor;
@SuppressWarnings("FieldCanBeLocal")

public class Robot {

    // arm extender
    public  final int ARM_IN = 0;
    public  final int ARM_OUT = 2000;
    public  final int ARM_EXCHANGE = 490;
    private final double ARM_SPEED = 0.3;

    // lifter
    private final int LIFTER_UP_POSITION = 1000;
    private final int LIFTER_DOWN_POSITION = 0;
    private final double LIFTER_SPEED = 0.25;

    // Grabbers
    private final double PICKER_UP_POSITION =0.5;
    private final double PICKER_DOWN_POSITION = 0.39;

    private final double PICKER_FINGER_CLOSED = 0.51;
    private final double PICKER_FINGER_OPEN = 0.189 ;

    private final double DROPPER_UP_POSITION = 0.563;
    private final double DROPPER_DOWN_POSITION = 0.50;

    private final double DROPPER_FINGER_CLOSED = 0.48;
    private final double DROPPER_FINGER_OPEN = 0.60;

    // Define Motor and Servo objects
    private DcMotorEx   lifter;
    public DcMotor      extendingArm;
    private Servo       pickerWrist;
    private Servo       pickerFingers;
    private Servo       dropperWrist;
    private Servo       dropperFingers;

    // drivetrain
    public Drive      drive = null;

    /* Declare OpMode members. */
    private final LinearOpMode opMode;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    /**
     * Initialize the Robot
     */
    public void init() {

        drive = new Drive(opMode);

        try {
            pickerWrist = opMode.hardwareMap.get(Servo.class, Config.DROPPER_WRIST);
            pickerFingers = opMode.hardwareMap.get(Servo.class, Config.DROPPER_FINGERS);

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
            extendingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }

        try {
            lifter = opMode.hardwareMap.get(DcMotorEx.class, Config.LIFTER);
            lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }
    }

    /**
     * Returns true if the lifter is retractable from its current position
     * @return true if retractable, false if all the way dowm
     */
    public boolean lifterRetractable() {
        int position = extendingArm.getCurrentPosition();
        boolean retractable = position > LIFTER_DOWN_POSITION;
        Logger.message("lifter position %d %B", position, retractable);
        return  retractable;
    }

    /**
     * Returns true if the lifter is extendable
     * @return true if extendable, false if fully extend
     */
    public boolean lifterExtendable() {
        int position = extendingArm.getCurrentPosition();
        boolean extendable = position < LIFTER_UP_POSITION;
        Logger.message("lifter position %d %B", position, extendable);
        return extendable;
    }

    /**
     * Retract lifter to the specified position
     */
    public void lifterRetract () {
        if (lifterRetractable())
            lifter.setPower(-LIFTER_SPEED);
    }

    /**
     * Extend lifter to the specified position
     */
    public void LifterExtend() {
        if (lifterExtendable())
            lifter.setPower(LIFTER_SPEED);
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
        runMotorToPosition(lifter, LIFTER_UP_POSITION, LIFTER_SPEED);
    }

    /**
     * Lower the lifter to its down position
     */
    public void lifterDown() {
        runMotorToPosition(lifter, LIFTER_DOWN_POSITION, LIFTER_SPEED);
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
     * Retract arm to the specified position
     */
    public void amrRetract() {
        if (armRetractable())
            extendingArm.setPower(-ARM_SPEED);
    }

    /**
     * Extend arm to the specified position
      */
    public void armExtend() {
        if (armExtendable())
            extendingArm.setPower(ARM_SPEED);
    }

    /**
     * Stop the arm from moving
     */
    public void armStop () {
        extendingArm.setPower(0);
    }

    /**
     * Move the arm to the specified position
     * @param position target position
     */
    public void moveArm (int position) {
        runMotorToPosition(extendingArm, position, ARM_SPEED);
    }

    private void runMotorToPosition(DcMotor motor, int position, double speed) {

        int current = motor.getCurrentPosition();
        int last = current;
        Logger.message("run from %d to %d", current, position);
        DcMotor.RunMode mode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while (opMode.opModeIsActive()) {
            if (! motor.isBusy())
                break;
            if (emergencyStop())
                break;
            current = motor.getCurrentPosition();
            Logger.message("position %d", current);
        }
        motor.setPower(0);
        motor.setMode(mode);
    }


    public void pickerUp() {
        pickerWrist.setPosition(PICKER_UP_POSITION);
    }

    public void pickerDown() {
        pickerWrist.setPosition(PICKER_DOWN_POSITION);
    }

    public void pickerOpen(){
        pickerFingers.setPosition(PICKER_FINGER_OPEN);
    }

    public void pickerClosed(){
        pickerFingers.setPosition(PICKER_FINGER_CLOSED);
    }

    public void dropperUp() {
        dropperWrist.setPosition(DROPPER_UP_POSITION);
    }

    public void dropperDown() {
        dropperWrist.setPosition(DROPPER_DOWN_POSITION);
    }

    public void dropperOpen(){
        dropperFingers.setPosition(DROPPER_FINGER_OPEN);
    }

    public void dropperClosed(){
        dropperFingers.setPosition(DROPPER_FINGER_CLOSED);
    }

    public boolean emergencyStop() {
        return opMode.gamepad1.back;
    }

    public void turn(double degrees) {
        drive.turn(degrees);
    }

    public void forward (double distance) {
        drive.forward(distance);
    }
    public void back (double distance) {
        drive.back(distance);
    }

    public void strafeLeft (double distance) { drive.strafeLeft(distance); }

    public void strafeRight (double distance) {
        drive.strafeRight(distance);
    }

} // end of class

