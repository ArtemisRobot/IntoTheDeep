package main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Auto Samples Park Ascent (Yellow)", group = "Main")
@Config

public class AutoSamplesParkAscent extends LinearOpMode {

    public static double START_X = 8.5;
    public static double START_Y = 102.5;
    public static double START_HEADING = 90;

    public static double BUCKET_X = 15.5;
    public static double BUCKET_Y = 126.5;
    public static double BUCKET_HEADING = 135;

    public static double YELLOW_RIGHT_X = 32.5;
    public static double YELLOW_RIGHT_Y = 120;
    public static double YELLOW_RIGHT_HEADING = 0;

    public static double YELLOW_MIDDLE_X = 32.5;
    public static double YELLOW_MIDDLE_Y = 130;
    public static double YELLOW_MIDDLE_HEADING = 0;

    public static double YELLOW_LEFT_X = 47;
    public static double YELLOW_LEFT_Y = 127.5;
    public static double YELLOW_LEFT_HEADING = 90;

    public static double PARK_X = 63;
    public static double PARK_Y = 95.5;
    public static double PARK_HEADING = 270;

    Auto auto;

    @Override
    public void runOpMode() {

        try {
            auto = new Auto(this);

            auto.createPath(Auto.PathState.START_YELLOW,          START_X,         START_Y,         START_HEADING);
            auto.createPath(Auto.PathState.BUCKET1,              BUCKET_X,        BUCKET_Y,        BUCKET_HEADING);
            auto.createPath(Auto.PathState.YELLOW_RIGHT,   YELLOW_RIGHT_X,  YELLOW_RIGHT_Y,  YELLOW_RIGHT_HEADING);
            auto.createPath(Auto.PathState.BUCKET2,              BUCKET_X,        BUCKET_Y,        BUCKET_HEADING);
            auto.createPath(Auto.PathState.YELLOW_MIDDLE, YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, YELLOW_MIDDLE_HEADING);
            auto.createPath(Auto.PathState.BUCKET3,              BUCKET_X,        BUCKET_Y,        BUCKET_HEADING);
            auto.createPath(Auto.PathState.YELLOW_LEFT,     YELLOW_LEFT_X,   YELLOW_LEFT_Y,   YELLOW_LEFT_HEADING);
            auto.createPath(Auto.PathState.BUCKET4,              BUCKET_X,        BUCKET_Y,        BUCKET_HEADING);
            auto.createPath(Auto.PathState.PARK,                   PARK_X,          PARK_Y,          PARK_HEADING);

            auto.setStartPath(Auto.PathState.START_YELLOW);
            auto.enableAscent(true);

            sleep(1000); // wait for pinpoint hardware to reset

            telemetry.addData("Lifter enabled", Auto.enableLifter);
            telemetry.addData("Dropper enabled", Auto.enableDropper);
            telemetry.addData("wait enabled", Auto.enableWait);
            telemetry.update();

            waitForStart();

            auto.runSamplesAuto();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }
}


