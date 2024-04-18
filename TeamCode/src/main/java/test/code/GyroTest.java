/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package test.code;

import android.annotation.SuppressLint;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Drive;
import common.Logger;


@TeleOp(name="Gyro Test", group="Test")
@Disabled
public class GyroTest extends LinearOpMode {

    final double TURN_RAMP_UP_TIME      = 1000;                       // turn ramp up time in milliseconds
    final double TURN_RAMP_DOWN_DEGREES = 5;
    final double TURN_SPEED             = 0.25;
    final double TURN_MIN_SPEED         = 0.2;

    Drive drive;
    AHRS gyro;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive = new Drive(this);
        initGyro();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.y)
                turnTo(0);

            if (gamepad1.b)
                turnTo(90);

            if (gamepad1.a)
                turnTo(180);

            if (gamepad1.x)
                turnTo(270);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Run Time", "%6.0f ", runtime.seconds());
            telemetry.addData("calibrated", "%s ", gyro.isCalibrating() ? "false" : "true");
            telemetry.addData("yaw navx", "%6.2f ", gyro.getYaw());
            telemetry.addData("yaw imu", "%6.2f ", drive.getOrientation());
            telemetry.update();
        }
    }

    private void initGyro ()
    {
        gyro = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);

        while ( gyro.isCalibrating() ) {
            if (! opModeIsActive()) break;
        }
        gyro.zeroYaw();
    }

    private float getOrientation () {
        return gyro.getYaw();
    }

    /**
     * Calculate to turn speed. The turn speed to ramped up based on time, and ramped down based on
     * degrees remaining to turn.
     *
     * @param startTime time in millisecond of the start of the turn
     * @param speed speed of the turn (0-1)
     * @param degreesToGo degrees remaining to turn
     * @return motor power (0-1)
     */
    private double getRampedPower (double startTime, double speed, double degreesToGo) {
        double speedRange = Math.max(Math.abs(speed) - TURN_MIN_SPEED, 0);
        double ramUp = (startTime / TURN_RAMP_UP_TIME) * speedRange + TURN_MIN_SPEED;
        double ramDown = (Math.pow(degreesToGo, 2) / Math.pow(TURN_RAMP_DOWN_DEGREES, 2)) * speedRange + TURN_MIN_SPEED;
        return Math.min(Math.min(ramUp, ramDown), speed);
    }

    @SuppressLint("DefaultLocale")
    public void turnTo(float toDegrees) {

        float target;
        if (toDegrees <= 180)
            target = toDegrees;
        else
            target = -(360 - toDegrees);

        float start = getOrientation();
        float degrees = target - start;

        // Determine the shortest direction to turn
        Drive.DIRECTION direction;
        if (degrees > 0) {
            if (degrees <= 180)
                direction = Drive.DIRECTION.TURN_RIGHT;
            else
                direction = Drive.DIRECTION.TURN_LEFT;
        } else if (degrees < 0) {
            if (degrees <= -180)
                direction = Drive.DIRECTION.TURN_RIGHT;
            else
                direction = Drive.DIRECTION.TURN_LEFT;
        } else {
            return;
        }

        float lastPos = start;
        float current;
        double lastTime = 0;
        double velocity = 0;
        double degreesTurned = 0;
        double degreesEstimated = 0;
        double degreesToTurn = Math.abs(degrees);
        degreesToTurn = Math.min(degreesToTurn, 360 - degreesToTurn);

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        drive.moveRobot(direction, 0.1);

        while (opModeIsActive()) {
            current = getOrientation();
            if (current != lastPos) {
                double currentTime = elapsedTime.milliseconds();
                float diff = Math.abs(current - lastPos);
                diff = Math.min(diff, 360-diff);
                velocity = diff / (currentTime - lastTime);
                lastTime = currentTime;
                lastPos = current;
                degreesTurned += diff;
                degreesEstimated = 0;
            } else {
                degreesEstimated = velocity * (elapsedTime.milliseconds() - lastTime);
            }

            Logger.message(
                    //String.format("%s", direction) +
                    String.format("  to: %-6.1f", target) +
                    String.format("  from: %-6.1f", current) +
                    String.format("  to turn: %-6.2f", degreesToTurn) +
                    String.format("  turned: %-6.2f", degreesTurned) +
                    String.format("  estimated: %-6.4f", degreesEstimated) +
                    String.format("  velocity: %-6.4f", velocity) +
                    String.format("  curr: %-6.4f", current) +
                    String.format("  last: %-6.4f", lastPos));

            if (degreesTurned + degreesEstimated >= degreesToTurn)
                break;
        }

        drive.stopRobot();
    }
}
