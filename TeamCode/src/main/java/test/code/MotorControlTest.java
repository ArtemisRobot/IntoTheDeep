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


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import common.Config;
import common.Logger;
import common.MotorControl;

@TeleOp(name=" Motor Control Test", group="Test")
@com.acmerobotics.dashboard.config.Config

public class MotorControlTest extends LinearOpMode {

    MotorControl motorControl;

    @Override
    public void runOpMode() {

        try {
            run();
        } catch (Exception e)  {
            Logger.error(e, "Exception");
        }
    }

    private void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, Config.RIGHT_FRONT);
        motorControl = new MotorControl(this, motor);
        motorControl.start();

        telemetry.addLine("Press start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                motorControl.setLowSpeedThreshold(500);
                motorControl.setPosition(5000, 0.5, 0.2);
                while (gamepad1.x) sleep(10);

            } else if (gamepad1.b) {
                motorControl.stopMotor();
                while (gamepad1.b) sleep(10);

            } else if (gamepad1.a) {
                motorControl.setRange(0, 5000);

            } else if (gamepad1.right_trigger > 0) {
                while (gamepad1.right_trigger > 0)
                    motorControl.runMotor(gamepad1.right_trigger);
                motorControl.stopMotor();

            } else if (gamepad1.left_trigger > 0) {
                while (gamepad1.left_trigger > 0)
                    motorControl.runMotor(-gamepad1.left_trigger);
                motorControl.stopMotor();
            }

            telemetry.addData("Motor is idle", motorControl.motorIsIdle());
            telemetry.update();

        }
    }
}
