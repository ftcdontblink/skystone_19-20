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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp States w/ State Machines", group="Linear Opmode")

public class TeleStatesMachines extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // subsystems objects
    Robot robot;
    public double x, y, rotate, magnitude, theta, t;
    public double lFrontSpeed, rFrontSpeed, lBackSpeed, rBackSpeed;

    public final int intakeSwitch = 0;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    public enum state {
        INTAKE,
        INTAKED,
        IDLE,
        OUTTAKE,
        OUTTAKED
    }

    public state intakeState = state.INTAKE;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot = new Robot(hardwareMap, telemetry, lastAngles, imu);

        waitForStart();

        while (opModeIsActive()) {
            drive();
            lift();
            intake();
            claw();
            foundation();
        }
    }

    public void drive() {
        x = gamepad1.left_stick_x; // x value determined from x movement of joystick
        y = -gamepad1.left_stick_y; // same for y value
        rotate = gamepad1.right_stick_x; // same for rotate value (other joystick)

        if(gamepad1.a) // speed - 100%
            t = 1;

        if(gamepad1.b) // speed - 50%
            t = 2;

        if(gamepad1.x) // speed - 25%
            t = 4;

        magnitude = Math.hypot(x, y) / t; // magnitude determined by joystick value and turtle
        theta = Math.atan2(y, x); // theta determined by the angle of the joystick

        // uses unit circle to get maximum attainable speeds
        lFrontSpeed = magnitude * (Math.sin(theta + Math.PI/4)) + (t * rotate);
        lBackSpeed = magnitude * (Math.sin(theta - Math.PI/4)) + (t * rotate);
        rFrontSpeed = magnitude * (Math.sin(theta - Math.PI/4)) - (t * rotate);
        rBackSpeed = magnitude * (Math.sin(theta + Math.PI/4)) - (t * rotate);

        // setting speeds for all the motors
        robot.leftFront.setPower(lFrontSpeed);
        robot.leftBack.setPower(lBackSpeed);
        robot.rightFront.setPower(rFrontSpeed);
        robot.rightBack.setPower(rBackSpeed);

        if(gamepad1.left_bumper) {
            robot.kicker.setPosition(0.5);
        }
    }

    public void lift() {
        robot.leftLift.setPower(gamepad2.left_stick_y);
        robot.rightLift.setPower(-gamepad2.left_stick_y);

        if(gamepad2.right_bumper)
            robot.kicker.setPosition(0.1);

        if(gamepad2.left_bumper)
            robot.kicker.setPosition(0.65);

        if(gamepad2.left_stick_button)
            robot.liftMove(0, this);

        if(gamepad2.right_stick_button)
            robot.liftMove(4.5, this);

        robot.liftExtension.setPower(-gamepad2.right_stick_y);

        if (gamepad2.x)
            robot.liftClamp.setPosition(0.20); //clamp position

        if(gamepad2.y)
            robot.liftClamp.setPosition(0.6); //open position
    }

    public void intake() {
        switch(intakeState) {
            case INTAKE:
                robot.leftIntake.setPower(-1);
                robot.rightIntake.setPower(1);
                robot.kicker.setPosition(0.65);

                if(gamepad2.right_bumper) {
                    intakeState = state.INTAKED;
                }
                break;

            case INTAKED:
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
                robot.liftClamp.setPosition(0.6);
                sleep(300);
                robot.kicker.setPosition(0.1);
                sleep(700);
                intakeState = state.IDLE;
                break;

            case IDLE:

        }

        if (gamepad2.dpad_right) { //Intake Position
            robot.rightIntakeServo.setPosition(0.27);
            robot.leftIntakeServo.setPosition(0.64);
        }

        if(gamepad2.dpad_left) {
            robot.rightIntakeServo.setPosition(0.31);
            robot.leftIntakeServo.setPosition(0.6);
        }


        if(gamepad2.left_trigger > 0) { // outtake
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(-1);
            robot.kicker.setPosition(0.65);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

        if(gamepad2.right_trigger > 0) { // intake
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(1);
            robot.kicker.setPosition(0.65);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }

    public void claw() {
        if (gamepad1.dpad_up) // set position to raise
            robot.autonHook.setPosition(0);

        if (gamepad1.dpad_down) // set position to plough and clamp
            robot.autonHook.setPosition(0.27);

        if (gamepad1.dpad_left) // not clamped
            robot.autonClamp.setPosition(0.35);

        if (gamepad1.dpad_right) // clamped
            robot.autonClamp.setPosition(0.78);
    }

    public void foundation() {
        if (gamepad2.a) { // foundation grabbed
            robot.leftFoundation.setPosition(0.3);
            robot.rightFoundation.setPosition(0.3);
        }

        if (gamepad2.b) { // foundation not grabbed
            robot.leftFoundation.setPosition(0.45);
            robot.rightFoundation.setPosition(0.45);
        }
    }
}
