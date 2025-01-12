package common;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotSimulator {

    public double PICKER_YAW_90_DEGREES = 0;

    private final LinearOpMode opMode;
    private final Robot robot;

    public RobotSimulator(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.robot = robot;
    }

    public DriveControl getDriveControl() {
        return robot.getDriveControl();
    }

    public void setToStartPosition() {
        Logger.message("  *  setToStartPosition");
    }

    public void setToStopPosition() {
        Logger.message("  *  setToStopPosition");
    }

    public void lifterUp() {
        Logger.message("  *  lifterUp");
    }

    public void lifterDown() {
        Logger.message("  *  lifterDown");
    }

    public void dropSampleInTopBucket() {
        Logger.message("  *  dropSampleInTopBucket");
    }

    public void pickUpSample() {
        Logger.message("  *  pickUpSample");
    }

    public void moveSampleToDropper() {
        Logger.message("  *  moveSampleToDropper");
    }

    public void pickerRotateTo(double position) {
        Logger.message("  *  pickerRotateTo %f", position);
    }

    public boolean isBusy() {
        Logger.message("  *  isBusy");
        return false;
    }

    public boolean okToMove() {
        Logger.message("  *  okToMove");
        return true;
    }

    public void scoreSpecimen() {
        Logger.message("  *  scoreSpecimen");
    }

    public void moveToCoordinate(double targetX, double targetY, double targetHeading, double timeout) {
        Logger.message("  *  moveToCoordinate");
    }

}
