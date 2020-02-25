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
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * @author - Aarush Sharma
 * @version - 9/29/19 - Draft 1.0 */

/**
 * For a mecanum drivetrain, we need to be able to calculate the different inputs of the two
 * joysticks including the axes for the left joystick in particular, as the right joystick only
 * controls the rotation.
 *
 * Left joystick will control the translation of the robot in all directions as this is a mecanum
 * drivetrain.  Right joystick will control the rotation of the robot from its center
 */

@TeleOp(name="TeleStates", group="Linear Opmode")
public class TeleStates extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // subsystems objects
    Robot robot;
    public double x, y, rotate, magnitude, theta, t;
    public double lFrontSpeed, rFrontSpeed, lBackSpeed, rBackSpeed;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

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
    }

    public void lift() {
        robot.leftLift.setPower(gamepad2.left_stick_y);
        robot.rightLift.setPower(-gamepad2.left_stick_y);

        if(gamepad2.left_bumper)
            robot.kicker.setPosition(0.1);

        if(gamepad2.right_bumper)
            robot.kicker.setPosition(0.65);


        robot.liftExtension.setPower(gamepad2.right_stick_y);

        if (gamepad2.x)
            robot.liftClamp.setPosition(0.20); //clamp position

        if(gamepad2.y)
            robot.liftClamp.setPosition(0.6); //open position
    }

    public void intake() {
        if (gamepad2.dpad_right) { //Intake Position
            robot.rightIntakeServo.setPosition(0.27);
            robot.leftIntakeServo.setPosition(0.64);
        }


        if(gamepad2.left_trigger > 0) { // intake
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(-1);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

        if(gamepad2.right_trigger > 0) { // outtake
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(1);
        } else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }

    public void claw() {
        if (gamepad1.dpad_up) // set position to raise
            robot.autonHook.setPosition(0);

        if (gamepad1.dpad_down) // set position to plough and clamp
            robot.autonHook.setPosition(0.58);

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
