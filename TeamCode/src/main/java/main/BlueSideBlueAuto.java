package main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import common.Auto;
import common.Logger;
import common.Robot;

@Autonomous(name="BlueSideBlueAuto", group = "Main")

 public class BlueSideBlueAuto extends LinearOpMode {

    public static double START_X = 22.9;
    public static double START_Y = 55.1;
    public static double START_HEADING = 0;

    public static double BAR_X1 = 62;
    public static double BAR_Y1 = 45;
    public static double BAR_Heading1 = 90;

    public static double OBSERVE_ZONE_X1 = 12;
    public static double OBSERVE_ZONE_Y1 = 23.5;
    public static double OBSERVE_HEADING1 = 0;

    public static double OBSERVE_ZONE_X2 = 12;
    public static double OBSERVE_ZONE_Y2 = 13.5;
    public static double OBSERVE_HEADING2 = 0;

    public static double OBSERVE_ZONE_X3 = 12;
    public static double OBSERVE_ZONE_Y3 = 9;
    public static double OBSERVE_HEADING3 = 0;

    public static double SPECIMEN_RIGHT_X = 60;
    public static double SPECIMEN_RIGHT_Y = 23.5;
    public static double SPECIMEN_RIGHT_HEADING = 0;

    public static double SPECIMEN_MIDDLE_X = 60;
    public static double SPECIMEN_MIDDLE_Y = 13.5;
    public static double SPECIMEN_MIDDLE_HEADING = 0;

    public static double SPECIMEN_LEFT_X = 60;
    public static double SPECIMEN_LEFT_Y = 9;
    public static double SPECIMEN_LEFT_HEADING = 0;

    public static double BAR_X2 = 65;
    public static double BAR_Y2 = 45;
    public static double BAR_HEADING2 = 0;

    Auto auto;

    @Override
    public void runOpMode() {

        auto = new Auto(this);

        auto.setStartingPose(START_X, START_Y, START_HEADING);

        auto.buildBlueRedPaths(
       START_X,
       START_Y,
       START_HEADING,
       BAR_X1,
       BAR_Y1,
       BAR_Heading1,
       OBSERVE_ZONE_X1,
       OBSERVE_ZONE_Y1,
       OBSERVE_HEADING1,
       OBSERVE_ZONE_X2,
       OBSERVE_ZONE_Y2,
       OBSERVE_HEADING2,
       OBSERVE_ZONE_X3,
       OBSERVE_ZONE_Y3,
       OBSERVE_HEADING3,
       SPECIMEN_RIGHT_X,
       SPECIMEN_RIGHT_Y,
       SPECIMEN_RIGHT_HEADING,
       SPECIMEN_MIDDLE_X,
       SPECIMEN_MIDDLE_Y,
       SPECIMEN_MIDDLE_HEADING,
       SPECIMEN_LEFT_X,
       SPECIMEN_LEFT_Y,
       SPECIMEN_LEFT_HEADING,
       BAR_X2,
       BAR_Y2,
       BAR_HEADING2);

        waitForStart();
        auto.runBlueRedAuto();
    }
}


