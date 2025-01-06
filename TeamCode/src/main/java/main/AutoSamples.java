package main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Autonomous(name="Auto Samples (Yellow)", group = "Main")
@Config

 public class AutoSamples extends LinearOpMode {

    public static double START_X = 8.5;
    public static double START_Y = 102.5;
    public static double START_HEADING = 90;

    public static double BUCKET_X = 14;
    public static double BUCKET_Y = 128;
    public static double BUCKET_HEADING = 135;

    public static double YELLOW_RIGHT_X = 32;
    public static double YELLOW_RIGHT_Y = 119.5;
    public static double YELLOW_RIGHT_HEADING = 0;

    public static double YELLOW_MIDDLE_X = 32;
    public static double YELLOW_MIDDLE_Y = 129.5;
    public static double YELLOW_MIDDLE_HEADING = 0;

    public static double YELLOW_LEFT_X = 43;
    public static double YELLOW_LEFT_Y = 129.5;
    public static double YELLOW_LEFT_HEADING = 90;

    public static double PARK_X = BUCKET_X;
    public static double PARK_Y = BUCKET_Y;
    public static double PARK_HEADING = START_HEADING;

    Auto auto;

    @Override
    public void runOpMode() {

       try {
          auto = new Auto(this);

          auto.buildSamplePaths(
               START_X,
               START_Y,
               START_HEADING,
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

         auto.runSamplesAuto();

       } catch (Exception e) {
          Logger.error(e, "Exception");
          throw e;
       }
    }
}

