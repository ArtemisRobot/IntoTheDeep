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

    private enum PathState { START, BUCKET1, YELLOW_RIGHT, BUCKET2, YELLOW_MIDDLE, BUCKET3, YELLOW_LEFT, SCORE_NET_ZONE}
    PathState pathState = PathState.START;

    Robot   robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this);

        Path[] paths = new Path[5];

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

    }

    private boolean isBusy () {
        return false;

    }
}
