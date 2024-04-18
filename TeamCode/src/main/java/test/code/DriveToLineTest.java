package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Drive;
import common.Robot;

/*
 * Test code for the hanging arm
 */
@TeleOp(name="Drive To Line Test", group="Test")
@Disabled
@SuppressWarnings("unused")
public class DriveToLineTest extends LinearOpMode {

  private static final double MIN_SPEED = 0.20;

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Press start");
    telemetry.update();

    Robot robot = new Robot(this);
    robot.init();

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    robot.drive.moveToColor(Drive.COLOR.RED, 1, 0, MIN_SPEED, 4000);

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.update();
      }
  }
}
