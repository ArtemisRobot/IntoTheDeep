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
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    // drone launcher servo position
    public static final double DRONE_ANGLE_DOWN = 0.5;
    public static       double DRONE_ANGLE_UP   = 0.471; //ToDo Test Angle
    public static final double DRONE_FIRE_DOWN  = 0.52;
    public static final double DRONE_FIRE_UP    = 0.65;

    // pixel dropper servo positions
    public static final double DROPPER_OPEN     = 0.51;
    public static final double DROPPER_CLOSE    = 0.67;

    // Intake
    static final double INTAKE_HOME      = 0.32;
    static final double INTAKE_TARGET    = 0.98;
    static final double SPINNER_SPEED    = 0.2;

    public boolean intakeUp              = true;
    public boolean intakeOn              = false;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor pixelIntake = null;
    private Servo   intakeRotate = null;
    private CRServo spinnerGray = null;
    private CRServo spinnerBlack = null;
    private CRServo spinnerBucket = null;

    public Servo dropper = null;        // Servo to drop the purple pixel
    public Servo droneAngle = null;
    public Servo droneFire = null;

    public Drive      drive = null;
    public Vision     vision = null;

    /* Declare OpMode members. */
    private final LinearOpMode opMode;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * Initialize the Robot
     */
    public void init() {

        drive = new Drive(opMode);

        try {
            dropper = opMode.hardwareMap.get(Servo.class, Config.PIXEL_DROPPER);

            droneAngle = opMode.hardwareMap.get(Servo.class, Config.DRONE_ANGLE);
            droneFire = opMode.hardwareMap.get(Servo.class, Config.DRONE_FIRE);

            pixelIntake = opMode.hardwareMap.get(DcMotor.class, Config.PIXEL_INTAKE);
            intakeRotate = opMode.hardwareMap.get(Servo.class, Config.INTAKE_ROTATE);
            spinnerGray = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_GRAY);
            spinnerBlack = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_BLACK);
            spinnerBucket = opMode.hardwareMap.get(CRServo.class, Config.SPINNER_BUCKET);


        } catch (Exception e) {
            Logger.error(e, "hardware not found");
        }
    }

    public void startVision () {
        vision = new Vision(opMode);
    }

    /**
     * Turn the intake off
     */
    public void intakeOff() {
        pixelIntake.setPower(0);
        spinnerGray.setPower(0);
        spinnerBlack.setPower(0);
        spinnerBucket.setPower(0);
        intakeOn = false;
    }

    /**
     * Toggle the pixel intake on or off.
     */
    public void toggleIntake()
    {
        if(intakeOn) {
            intakeOff();
        } else {
            pixelIntake.setPower(1);
            spinnerGray.setPower(SPINNER_SPEED);
            spinnerBlack.setPower(SPINNER_SPEED);
            spinnerBucket.setPower(SPINNER_SPEED);
            intakeOn = true;
        }
    }

    public void intakeReverse() {

        pixelIntake.setPower(-1);
        intakeOn = false;
    }

    public void toggleIntakeRotate() {
        if (intakeUp) {
            intakeRotate.setPosition(INTAKE_TARGET);
            intakeUp = false;
        } else {
            intakeRotate.setPosition(INTAKE_HOME);
            intakeUp = true;
        }
    }

    /**
     * Drop the preload purple pixel at the current location.
     */
    public void dropPurplePixel(){
        dropper.setPosition(DROPPER_OPEN);
        opMode.sleep(500);
        dropper.setPosition(DROPPER_CLOSE);
    }

    public void dropYellowPixel() {
        spinnerBucket.setPower(-SPINNER_SPEED);
        opMode.sleep(2500);
        spinnerBucket.setPower(0);
    }

    public void dropPixel () {
        spinnerBucket.setPower(-SPINNER_SPEED);
    }

    /**
     * Launch the drone
     */
    public void launchDrone(){
        droneAngle.setPosition(DRONE_ANGLE_UP);
        opMode.sleep(1000);
        droneFire.setPosition(DRONE_FIRE_UP);
        opMode.sleep(750);
        droneFire.setPosition(DRONE_FIRE_DOWN);
        droneAngle.setPosition(DRONE_ANGLE_DOWN);
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

    public void strafeLeft (double distance) {
        drive.strafeLeft(distance);
    }

    public void strafeRight (double distance) {
        drive.strafeRight(distance);
    }

} // end of class

