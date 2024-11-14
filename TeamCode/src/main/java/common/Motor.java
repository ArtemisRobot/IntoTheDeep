package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Motor extends Thread {

    public enum MOTOR_STATE {IDLE, MOVING, MOVING_TO_POSITION }
    private MOTOR_STATE state = MOTOR_STATE.IDLE;
    
    private final DcMotor motor;
    LinearOpMode opMode;

    private int targetPosition = 0;
    private int minPosition = 0;
    private int maxPosition = 0;
    private int lowSpeedThreshold = 200;
    private double speed = 0.5;
    private double lowSpeed = 0.2;

    public Motor(LinearOpMode opMode, DcMotor motor) {

        this.opMode = opMode;
        this.motor = motor;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Control the motor on a separate thread to avoid blocking
     */
    public void run() {

        Logger.message("robot motor thread started");

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {
            switch (state) {
                case IDLE:
                    yield();
                    continue;
                case MOVING:
                    limitRange();
                    continue;
                case MOVING_TO_POSITION:
                    runMotorToPosition();
                    state = MOTOR_STATE.IDLE;
            }
        }
        Logger.message("robot motor thread stopped");
    }

    /**
     * Turn the motor on.
     *
     * @param speed  motor power (-1 to 1)
     */
    public void runMotor(int speed) {
        interruptAction();
        synchronized (motor) {
            motor.setPower(speed);
            state = MOTOR_STATE.MOVING;
        }
    }

    /**
     * Stop the motor
     */
    public void stopMotor () {
        motor.setPower(0);
        state = MOTOR_STATE.IDLE;
    }

    /**
     * Run the motor to the specified encoder position.
     * @param position encoder position to run to
     * @param speed motor power (-1 to 1)
     * @param lowSpeed ramp down motor power
     */
    public void setPosition(int position, double speed, double lowSpeed) {

        interruptAction();
        synchronized (motor) {
            targetPosition = position;
            this.speed = speed;
            this.lowSpeed = lowSpeed;
            state = MOTOR_STATE.MOVING_TO_POSITION;
        }
    }

    public void setRange (int minPosition, int maxPosition) {
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
    }

    public void setLowSpeedThreshold(int threshold) {
        lowSpeedThreshold = threshold;
    }

    public boolean motorIsIdle () {
        return state == MOTOR_STATE.IDLE;
    }

    private void interruptAction () {
        if (state != MOTOR_STATE.IDLE) {
            motor.setPower(0);
            state = MOTOR_STATE.IDLE;
        }
    }

    private void runMotorToPosition() {

        ElapsedTime elapsedTime = new ElapsedTime();

        int current = motor.getCurrentPosition();
        int last = current;
        Logger.message("run from %d to %d", current, targetPosition);
        DcMotor.RunMode mode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);

        double lastMoveTime = 0;
        elapsedTime.reset();
        while (opMode.opModeIsActive() && state == MOTOR_STATE.MOVING_TO_POSITION) {
            if (!motor.isBusy())
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

            int remaining = Math.abs(targetPosition - current);
            if (remaining < lowSpeedThreshold && speed != lowSpeed) {
                motor.setPower(lowSpeed);
                Logger.message("remaining %d set to lower speed");
            }

            //Logger.message("position %5d   remaining %5d  elapsed %6.2f ", current, remaining, elapsedTime.milliseconds());
        }
        motor.setPower(0);
        motor.setMode(mode);
    }

    private void limitRange () {

        if (minPosition != 0 || maxPosition != 0) {
            while (opMode.opModeIsActive() && state == MOTOR_STATE.MOVING) {
                int position = motor.getCurrentPosition();
                if (position < minPosition || position > maxPosition) {
                    interruptAction();
                    break;
                }
            }
        }
    }

    private boolean emergencyStop() {
        return opMode.gamepad1.back;
    }
}



