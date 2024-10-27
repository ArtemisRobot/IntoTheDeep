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
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    // lifter
    public static double LIFTER_IN = 0;
    public static double LIFTER_OUT = 0;

    // Grabbers
    public static double BOTTOM_GRABBER_OPEN = 0;
    public static double BOTTOM_GRABBER_CLOSED = 0;

    private int LIFTER_UP_POSITION = 1000;
    private int LIFTER_DOWN_POSITION = 0;

    private double PICKER_UP_POSITION = 0.5;
    private double PICKER_DOWN_POSITION = .39;

    private final double PICKER_FINGER_CLOSED = 0.51;
    private final double PICKER_FINGER_OPEN = .189  ;

    private double LIFTER_SPEED = 0.25;

    // Define Motor and Servo objects
    private DcMotor extendingArm;
    private Servo   pickerWrist;
    private DcMotorEx lifter;
    private Servo pickerFingers;

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
            extendingArm = opMode.hardwareMap.get(DcMotor.class, Config.ARM);
            pickerWrist = opMode.hardwareMap.get(Servo.class, Config.PICKER_WRIST);
            pickerFingers = opMode.hardwareMap.get(Servo.class, Config.PICKER_FINGERS);

        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }

        try {
            lifter = opMode.hardwareMap.get(DcMotorEx.class, Config.LIFTER);
            lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }

    }

    /**
     * Raise the lifter to the specified position
     */
    public void lifterUp() {

        lifter.setTargetPosition(LIFTER_UP_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(LIFTER_SPEED);
        while (opMode.opModeIsActive() && lifter.isBusy()) {
            if (opMode.gamepad1.back)
                break;
        }
        lifter.setPower(0);
    }

    public void lifterDown() {
        lifter.setTargetPosition(LIFTER_DOWN_POSITION);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(LIFTER_SPEED);
        while (opMode.opModeIsActive() && lifter.isBusy()) {
            if (opMode.gamepad1.back)
                break;
        }
        lifter.setPower(0);
    }



    public void pickerUp() {
        pickerWrist.setPosition(PICKER_DOWN_POSITION);
    }

    public void pickerDown() {
        pickerWrist.setPosition(PICKER_UP_POSITION);
    }

    public void pickerOpen(){
        pickerFingers.setPosition(PICKER_FINGER_OPEN);
    }

    public void pickerClosed(){
        pickerFingers.setPosition(PICKER_FINGER_CLOSED);
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

