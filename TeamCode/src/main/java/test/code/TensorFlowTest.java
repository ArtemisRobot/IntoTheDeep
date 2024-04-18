/*
 * Test code for threads
 */
package test.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Vision;
@Disabled
@TeleOp(name="TensorFlow Test", group="Test")
@SuppressWarnings("unused")
public class TensorFlowTest extends LinearOpMode {

  private final ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
      //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

      Vision vision = new Vision(this);

      telemetry.addLine("waiting for camera");
      telemetry.update();
      while (vision.cameraReady())
          sleep(100);
      telemetry.addLine("camera ready, press start");
      telemetry.update();

      vision.enableCameraStream(true);

      waitForStart();

      vision.enableCameraStream(false);

      runtime.reset();

      String name;
      while (opModeIsActive()) {

        if (gamepad1.a) {
            while (gamepad1.a) sleep(100);
        }

        if (gamepad1.x) {
            name = "blue";
            if (vision.findTeamElement(name, 2000)) {
                telemetry.addData("Found", "%s %.0f %% ", name, vision.getElementConfidence() * 100);
            } else {
                telemetry.addLine("blue not found");
            }
            telemetry.update();
            while (gamepad1.x) sleep(100);
        }

        if (gamepad1.b) {
            name = "red";
            if (vision.findTeamElement(name, 2000)) {
                telemetry.addData("Found", "%s %.0f %% ", name, vision.getElementConfidence() * 100);
            } else {
                telemetry.addLine("red not found");
            }
            telemetry.update();
            while (gamepad1.b) sleep(100);
        }

    }
  }
}
