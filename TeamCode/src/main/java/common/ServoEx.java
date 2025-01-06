package common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoEx  {

    public enum ServoType { TORQUE, SPEED, SUPER_SPEED, TORQUE_5_TURN }

    private final Servo servo;
    private final ServoType servoType;
    private double position;
    private long idleTime;

    public ServoEx(ServoType servoType, HardwareMap hardwareMap, String name) {

        this.servoType = servoType;
        this.position = Double.NaN;
        this.idleTime = System.currentTimeMillis();

        servo = hardwareMap.get(Servo.class, name);
    }

    public void setPosition(double position) {

        servo.setPosition(position);

        double SERVO_TRAVEL        = 300;
        double SERVO_TRAVEL_5_TURN = 1800;

        double TORQUE_SPEED        = (double) 250 / 60;
        double SPEED_SPEED         = (double) 110 / 60;
        double SPEED_SUPER_SPEED   = (double) 55 / 60;
        double speed;
        double travel;

        switch (servoType) {
            case TORQUE:
                speed = TORQUE_SPEED;
                travel = SERVO_TRAVEL;
                break;
            case SPEED:
                speed = SPEED_SPEED;
                travel = SERVO_TRAVEL;
                break;
            case SUPER_SPEED:
                speed = SPEED_SUPER_SPEED;
                travel = SERVO_TRAVEL;
                break;
            case TORQUE_5_TURN:
                speed = TORQUE_SPEED;
                travel = SERVO_TRAVEL_5_TURN;
                break;
            default:
                speed = 0;
                travel = 0;
        }

        if (!Double.isNaN(this.position)) {
            double degrees = travel * Math.abs(this.position - position);
            idleTime = System.currentTimeMillis() + (long)(degrees * speed);
            Logger.message("%s   move from %5.2f to %5.2f  degrees: %4.0f  idle in: %6d", servoType, this.position, position, degrees, timeUntilIdle());
        }
        this.position = position;
    }

    public boolean isBusy() {
        return System.currentTimeMillis() > idleTime;
    }

    public long timeUntilIdle() {
        return Math.max (idleTime - System.currentTimeMillis(), 0);
    }

}
