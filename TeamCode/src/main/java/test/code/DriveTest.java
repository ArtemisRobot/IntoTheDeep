/*
 * Test code for threads
 */
package test.code;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Drive;
import common.Logger;

@TeleOp(name=" Drive Test", group="Test")
@SuppressWarnings("unused")
@SuppressLint("DefaultLocale")
public class DriveTest extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();
  //Logger logger = Logger.getLogger("MyLog");
  Drive drive  = null;

  @Override
  public void runOpMode() {
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      telemetry.addData("Status", "Press start");
      telemetry.update();

      drive = new Drive(this);
      drive.start();

      waitForStart();

      /*
      for (double power=0.2; power<=0.801; power+=0.05) {
          if (! opModeIsActive()) break;
          checkAccuracy(power);
      }
       */

      while (opModeIsActive()) {

        if (gamepad1.a) {
            for (double power=0.2; power<=0.801; power+=0.05) {
                if (! opModeIsActive()) break;
                checkAccuracy(power);
            }
        }

        if (gamepad1.x) {
            boolean found = drive.moveToObject(3, 0.25, 3000);
        }
      }
  }

    private void checkAccuracy (double power) {

        drive.resetEncoders();
        drive.moveRobot(power, 0, 0);

        runtime.reset();
        while (runtime.milliseconds() < 10000 && opModeIsActive()) {
            sleep(1);
        }

        //while (gamepad1.a) sleep(100);
        drive.stopRobot();
        sleep(1000);

        int leftFrontPos = Math.abs(drive.leftFrontDrive.getCurrentPosition());
        int rightFrontPos = Math.abs(drive.rightFrontDrive.getCurrentPosition());
        int leftBackPos = Math.abs(drive.leftBackDrive.getCurrentPosition());
        int rightBackPos = Math.abs(drive.rightBackDrive.getCurrentPosition());
        int maxPos = Math.max(Math.max(Math.max(leftFrontPos, rightFrontPos), leftBackPos), rightBackPos);

        double COUNTS_PER_MOTOR_REV = 384.5;           // Gobilda Yellow Jacket Motor 5203-2402-0001
        double WHEEL_DIAMETER_INCHES = (96 / 25.4);    // 96 mm wheels converted to inches
        double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

        Logger.message("\n");
        Logger.message("power %5.2f", power);
        Logger.message("left front  %6d    %5.2f    %6.3f", leftFrontPos,  (leftFrontPos / COUNTS_PER_INCH),  (((double)leftFrontPos  / maxPos) - 1) * 100);
        Logger.message("right front %6d    %5.2f    %6.3f", rightFrontPos, (rightFrontPos / COUNTS_PER_INCH), (((double)rightFrontPos / maxPos) - 1) * 100);
        Logger.message("left rear   %6d    %5.2f    %6.3f", leftBackPos,   (leftBackPos / COUNTS_PER_INCH),   (((double)leftBackPos   / maxPos) - 1) * 100);
        Logger.message("right rear  %6d    %5.2f    %6.3f", rightBackPos,  (rightBackPos / COUNTS_PER_INCH),  (((double)rightBackPos  / maxPos) - 1) * 100);

    }

    private void checkVelocity() {

        for (double power = 0.20; power <= 0.25; power += 0.001) {
            drive.moveRobot(power, 0, 0);
            Logger.message("\n");
            for (int i = 0; i<10; i++) {
                sleep(100);
                Logger.message("power: %5.3f   velocity: %6.1f  %6.1f",
                        power ,
                        drive.leftFrontDrive.getVelocity(),
                        drive.rightFrontDrive.getVelocity());
            }
        }
        drive.stopRobot();
    }

}
