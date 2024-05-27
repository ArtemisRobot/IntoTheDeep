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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Objects;
import java.util.SortedSet;

import common.Logger;

/*
 * This OpMode calibrate any of the robot motors.
 */

@TeleOp(name=" Calibrate Motor", group="Test")
//@SuppressWarnings("unused")

public class CalibrateMotor extends LinearOpMode {

    public enum MOTOR_SELECT { PREVIOUS, NEXT }

    private final ElapsedTime     runtime = new ElapsedTime();

    private final double incrementSlow = 1;
    private final double incrementMedium = 10;
    private final double incrementFast = 25;

    private DcMotorEx motor   = null;
    private int position = 0;
    private int home = 0;
    private int target = 0;
    final private double speed = 0.25;

    private static class MotorInfo implements Comparable<MotorInfo>{
        String      name;
        DcMotorEx   motor;
        int         home;
        int         target;

        @Override
        public int compareTo(MotorInfo o) {
            //return name.compareTo(o.name);
            return motor.getPortNumber() - o.motor.getPortNumber();
        }
    }
    MotorInfo[] motors = new MotorInfo[12];
    int motorCount;
    int currentMotor;

     static class MotorPositions {
        String name;
        int home;
        int target;

         public  MotorPositions (String name, int home, int target ){
             this.name = name;
             this.home = home;
             this.target = target;
         }
    }
    MotorPositions[] positions = new MotorPositions[12];


    @Override
    public void runOpMode() {

        getMotors();

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Motor Calibration Controls", "\n" +
                "  dpad left - select previous motor\n" +
                "  dpad right - select next motor\n" +
                "  left trigger - run motor backwards\n" +
                "  right trigger - run the motor forward\n" +
                "  left stick - increase/decrease target position\n" +
                "  right stick - increase/decrease home position\n" +
                "  y - set home position to current position\n" +
                "  a - set target position to current position\n" +
                "  x - run to home position\n" +
                "  b - run motor to target position\n" +
                "\n");

        Telemetry.Item motorNameMsg =  telemetry.addData("Motor name", 0);
        Telemetry.Item directionMsg = telemetry.addData("Motor direction", 0);
        Telemetry.Item positionMsg = telemetry.addData("Motor position", 0);
        Telemetry.Item homeMsg = telemetry.addData("Home position", 0);
        Telemetry.Item targetMsg = telemetry.addData("Target position", 0);

        motorNameMsg.setValue("%s  (%d)", motors[currentMotor].name, motor.getPortNumber());
        directionMsg.setValue("%s", motor.getDirection());
        positionMsg.setValue( "%d", motor.getCurrentPosition());
        homeMsg.setValue("%d", home);
        targetMsg.setValue("%d", target);

        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // set target position to current position
                target = motor.getCurrentPosition();
                targetMsg.setValue("%d", target);

            } else if (gamepad1.y) {
                // set the home position to the current position
                home = motor.getCurrentPosition();
                homeMsg.setValue("%d", home);

            } else if (gamepad1.x) {
                // run to zero position
                runToPosition(home);

            } else if (gamepad1.b) {
                // Run motor to an target position
                runToPosition(target);

            } else if (gamepad1.left_trigger > 0) {
                // manually run the motor backwards
                motor.setPower(-speed);
                while (gamepad1.left_trigger > 0) {
                    position = motor.getCurrentPosition();
                    positionMsg.setValue( "%d", position);
                    telemetry.update();
                    Logger.message("y %f", gamepad1.left_trigger);
                    sleep(100);
                }
                motor.setPower(0);

            } else if (gamepad1.right_trigger > 0) {
                // manually run the motor forward
                motor.setPower(speed);
                while (gamepad1.right_trigger > 0) {
                    position = motor.getCurrentPosition();
                    positionMsg.setValue( "%d", position);
                    telemetry.update();
                    sleep(100);
                }
                motor.setPower(0);

            } else if (gamepad1.left_stick_y > 0) {
                // increase target position
                runtime.reset();
                while (gamepad1.left_stick_y > 0) {
                    target -= increment(incrementSlow, incrementMedium, incrementFast);
                    targetMsg.setValue("%d", target);
                    telemetry.update();
                }

            } else if (gamepad1.left_stick_y < 0) {
                // decrease the target position
                runtime.reset();
                while (gamepad1.left_stick_y < 0) {
                    target += increment(incrementSlow, incrementMedium, incrementFast);
                    targetMsg.setValue("%d", target);
                    telemetry.update();
                }
                motor.setPower(0);

            } else if (gamepad1.right_stick_y > 0) {
                // increase home position
                runtime.reset();
                while (gamepad1.right_stick_y > 0) {
                    home -= increment(incrementSlow, incrementMedium, incrementFast);
                    homeMsg.setValue("%d", home);
                    telemetry.update();
                }

            } else if (gamepad1.right_stick_y < 0) {
                // decrease the home position
                runtime.reset();
                while (gamepad1.right_stick_y < 0) {
                    home += increment(incrementSlow, incrementMedium, incrementFast);
                    homeMsg.setValue("%d", home);
                    telemetry.update();
                }
                motor.setPower(0);

            } else if (gamepad1.dpad_left) {
                // select the next motor
                selectMotor(MOTOR_SELECT.NEXT);
                while (gamepad1.dpad_left){
                    sleep(10);
                }
                motorNameMsg.setValue("%s  (%d)", motors[currentMotor].name, motors[currentMotor].motor.getPortNumber());

            } else if (gamepad1.dpad_right) {
                // select the previous motor
                selectMotor(MOTOR_SELECT.PREVIOUS);
                while (gamepad1.dpad_right){
                    sleep(10);
                }
                motorNameMsg.setValue("%s  (%d)", motors[currentMotor].name, motors[currentMotor].motor.getPortNumber());

            } else if (gamepad1.dpad_up) {
                // change the direction of the motor
                if (motor.getDirection() == DcMotorEx.Direction.FORWARD)
                    motor.setDirection(DcMotorEx.Direction.REVERSE);
                else
                    motor.setDirection(DcMotorEx.Direction.FORWARD);
                directionMsg.setValue("%s", motor.getDirection());
                while (gamepad1.dpad_up){
                    sleep(10);
                }

            } else if (gamepad1.dpad_down) {
                while (gamepad1.dpad_down){
                    sleep(10);
                }
            }

            positionMsg.setValue( "%d", motor.getCurrentPosition());
            homeMsg.setValue("%d", home);
            targetMsg.setValue("%d", target);
            telemetry.update();
        }
    }

    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    public double increment(double v1, double v2, double v3){
        int sleepTime;
        double delta;
        if (runtime.seconds() < 3){
            delta = v1;
            sleepTime = 500;
        }
        else if (runtime.seconds() < 6){
            delta = v2;
            sleepTime = 200;
        }
        else{
            delta = v3;
            sleepTime = 100;
        }
        sleep(sleepTime);
        return delta;
    }

    /**
     * Build a list of motors sorted by port number
     */
    private void getMotors(){

        motorCount = 0;
        SortedSet<String> names = hardwareMap.getAllNames(DcMotorEx.class);
        for (String name: names){
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            
            motors[motorCount] = new MotorInfo();
            motors[motorCount].name = name;
            motors[motorCount].motor = m;
            motors[motorCount].home = 0;
            motors[motorCount].target = 0;

            for (MotorPositions p : positions) {
                if (p != null && Objects.equals(p.name, name)) {
                    motors[motorCount].home = p.home;
                    motors[motorCount].target = p.target;
                    break;
                }
            }
        motorCount++;
        }

        Arrays.sort(motors, 0, motorCount);

        for (int i = 0; i < motors.length; i++) {
            if (motors[i] != null) {
                if (motor == null) {
                    motor = motors[i].motor;
                    home = motors[i].home;
                    target = motors[i].target;
                    currentMotor = i;
                }
                Logger.message("motor name %s port %d", motors[i].name, motors[i].motor.getPortNumber());
            }
        }
    }

    private void selectMotor (MOTOR_SELECT select){
        int index;
        for (int i = 1; i <= motors.length; i++) {
            if (select == MOTOR_SELECT.NEXT)
                index = (currentMotor + i) %  motors.length;
            else
                index = (currentMotor + motors.length - i) %  motors.length;

            if (motors[index] != null){
                motor = motors[index].motor;
                home = motors[index].home;
                target = motors[index].target;
                currentMotor = index;
                break;
            }
        }

    }

    private void runToPosition(int position) {

        //DcMotor.RunMode mode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while (opModeIsActive()) {
            if (! motor.isBusy())
                break;
        }
        motor.setPower(0);
    }
}

