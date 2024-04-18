package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import common.Drive;
import common.Logger;
import common.Robot;

@TeleOp(name="Drive To Object", group="Test")
@Disabled
@SuppressWarnings("unused")
public class DriveToObject extends LinearOpMode {

  private static final double MIN_SPEED = 0.20;

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Press start");
    telemetry.update();

    Drive drive = new Drive(this);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    boolean found = drive.moveToObject(1.5,0.25, 10000);
    sleep(2000);
    double distance = drive.distanceSensor.getDistance(DistanceUnit.INCH);
    Logger.message("distance %6.2f  time  %6.2f", distance, runtime.seconds());


    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      //double distance = drive.distanceSensor.getDistance(DistanceUnit.INCH);
      //Logger.message("distance %6.2f  time  %8.0f", distance, runtime.milliseconds());

      telemetry.addData("Distance"," %6.2f  time  %8.0f", distance, runtime.milliseconds());
      telemetry.update();
      }
  }
}
