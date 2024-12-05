package main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;

@Autonomous(name="RedSideYellowAuto", group = "Main")
@Config

 public class RedSideYellowAuto extends LinearOpMode {

    public static double START_X = 133.5;
    public static double START_Y = 39.5;
    public static double START_HEADING = 180;

   public static double NET_ZONE_X = 125;
   public static double NET_ZONE_Y = 15;
   public static double NET_ZONE_HEADING = -45;

   public static double BUCKET_X = 126;
    public static double BUCKET_Y = 14;
    public static double BUCKET_HEADING = -45;

    public static double YELLOW_RIGHT_X = 109;
    public static double YELLOW_RIGHT_Y = 24;
    public static double YELLOW_RIGHT_HEADING = 180;

    public static double YELLOW_MIDDLE_X = 109;
    public static double YELLOW_MIDDLE_Y = 14;
    public static double YELLOW_MIDDLE_HEADING = 180;

    public static double YELLOW_LEFT_X = 109;
    public static double YELLOW_LEFT_Y = 10;
    public static double YELLOW_LEFT_HEADING = 180;

    public static double PARK_X = START_X - 4;
    public static double PARK_Y = START_Y;
    public static double PARK_HEADING = START_HEADING;

    Auto auto;

    @Override
    public void runOpMode() {

        auto = new Auto(this);

        auto.setStartingPose(START_X, START_Y, START_HEADING);

        auto.buildYellowPaths(
            START_X,
            START_Y,
            START_HEADING,
            NET_ZONE_X,
            NET_ZONE_Y,
            NET_ZONE_HEADING,
            BUCKET_X,
            BUCKET_Y,
            BUCKET_HEADING,
            YELLOW_RIGHT_X,
            YELLOW_RIGHT_Y,
            YELLOW_RIGHT_HEADING,
            YELLOW_MIDDLE_X,
            YELLOW_MIDDLE_Y,
            YELLOW_MIDDLE_HEADING,
            YELLOW_LEFT_X,
            YELLOW_LEFT_Y,
            YELLOW_LEFT_HEADING,
            PARK_X,
            PARK_Y,
            PARK_HEADING);

            waitForStart();
            auto.runYellowAuto();
    }
}
