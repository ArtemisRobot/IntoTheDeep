package main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

import common.Robot;

@Autonomous(name="BlueBucketAuto", group = "Main")

 public class BlueAuto extends LinearOpMode {

    private static enum PathState {
        START(0),
        BUCKET1(1),
        YELLOW_RIGHT(2),
        BUCKET2(3),
        YELLOW_MIDDLE(4),
        BUCKET3(5),
        YELLOW_LEFT(6),
        SCORE_NET_ZONE(7);

        private final int value;

        PathState(int value) {
            this.value = value;
        }
        public int getValue() {
            return value;
        }
        public static PathState next(int id) {
            return values()[id];
        }
    }
    PathState pathState = PathState.START;
    Path[] paths = new Path[8];

    Robot   robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);


        buildPaths();
        waitForStart();


        while (opModeIsActive()) {

            switch (pathState) {
                case START:
                    followPath();
                    continue;

                case BUCKET1:
                case BUCKET3:
                case BUCKET2:
                    robot.dropSampleInTopBucket();
                    followPath();
                    continue;

                case YELLOW_RIGHT:
                case YELLOW_MIDDLE:
                    robot.pickUpYellow();
                    followPath();
                    continue;

                case YELLOW_LEFT:
                    robot.pushSample();
                    followPath();
                    continue;

                case SCORE_NET_ZONE:

            }

            while (isBusy()) sleep(10);
        }
    }

    private void buildPaths() {

    }

    private void followPath() {
        int index = pathState.value;

        pathState  = PathState.next(index+1);

    }

    private boolean isBusy () {
        return false;

    }
}
