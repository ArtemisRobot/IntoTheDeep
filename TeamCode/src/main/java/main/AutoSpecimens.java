package main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;

@Disabled
@Autonomous(name="Auto Specimens (Red/Blue)", group = "Main")

 public class AutoSpecimens extends LinearOpMode {

    public static double START_X = 22.9;
    public static double START_Y = 55.1;
    public static double START_HEADING = 0;

    public static double BAR_X = 62;
    public static double BAR_Y = 45;
    public static double BAR_Heading = 90;

    public static double OBSERVE_ZONE_X = 12;
    public static double OBSERVE_ZONE_Y = 23.5;
    public static double OBSERVE_HEADING = 0;

   public static double PICKUP_X = 65;
   public static double PICKUP_Y = 45;
   public static double PICKUP_HEADING = 0;

    public static double SPECIMEN_RIGHT_X = 60;
    public static double SPECIMEN_RIGHT_Y = 23.5;
    public static double SPECIMEN_RIGHT_HEADING = 0;

    public static double SPECIMEN_MIDDLE_X = 60;
    public static double SPECIMEN_MIDDLE_Y = 13.5;
    public static double SPECIMEN_MIDDLE_HEADING = 0;

    public static double SPECIMEN_LEFT_X = 60;
    public static double SPECIMEN_LEFT_Y = 9;
    public static double SPECIMEN_LEFT_HEADING = 0;


    Auto auto;

    @Override
    public void runOpMode() {

        auto = new Auto(this);


        auto.buildSpecimenPaths(
            START_X,
            START_Y,
            START_HEADING,

            BAR_X,
            BAR_Y,
            BAR_Heading,

            OBSERVE_ZONE_X,
            OBSERVE_ZONE_Y,
            OBSERVE_HEADING,

            PICKUP_X,
            PICKUP_Y,
            PICKUP_HEADING,

            SPECIMEN_RIGHT_X,
            SPECIMEN_RIGHT_Y,
            SPECIMEN_RIGHT_HEADING,

            SPECIMEN_MIDDLE_X,
            SPECIMEN_MIDDLE_Y,
            SPECIMEN_MIDDLE_HEADING,

            SPECIMEN_LEFT_X,
            SPECIMEN_LEFT_Y,
            SPECIMEN_LEFT_HEADING);

        waitForStart();
        auto.runSpecimensAuto();
    }
}


