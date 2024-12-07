package main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;

@Autonomous(name="YellowAuto", group = "Main")
@Config

 public class YellowAuto extends LinearOpMode {

    public static double START_X = 8.5;
    public static double START_Y = 102.5;
    public static double START_HEADING = 0;

   public static double NET_ZONE_X = 15;
   public static double NET_ZONE_Y = 120;
   public static double NET_ZONE_HEADING = 135;

   public static double BUCKET_X = 15;
    public static double BUCKET_Y = 124 ;
    public static double BUCKET_HEADING = 135;

    public static double YELLOW_RIGHT_X = 31;
    public static double YELLOW_RIGHT_Y = 118;
    public static double YELLOW_RIGHT_HEADING = 0;

    public static double YELLOW_MIDDLE_X = 31;
    public static double YELLOW_MIDDLE_Y = 128;
    public static double YELLOW_MIDDLE_HEADING = 0;

    public static double YELLOW_LEFT_X = 31;
    public static double YELLOW_LEFT_Y = 134;
    public static double YELLOW_LEFT_HEADING = 0;

    public static double PARK_X = BUCKET_X;
    public static double PARK_Y = BUCKET_Y;
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

            //auto.scoreSpecimen();
    }
}
