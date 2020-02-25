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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class ServoTester extends LinearOpMode {

    public double x, y, rotate, magnitude, theta, t;

    /*
        Variable x - x movement
        Variable y - y movement
        Variable rotate - turning on the robot axis
        Variable magnitude - joystick value (between -1 and 1)
        Variable theta - angle we want to move the robot to
        Variable t - turtle mode divisor
    */


    public double lFrontSpeed, rFrontSpeed, lBackSpeed, rBackSpeed;

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    // intake motors and intake servos
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public Servo leftIntakeServo;
    public Servo rightIntakeServo;

    public DcMotor leftLift;
    public DcMotor rightLift;
    public Servo liftClamp;
    public CRServo liftExtension;
    public Servo capstone;
    public Servo kicker;

    public Servo lf;
    public Servo rf;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "lfm");
        rightFront = hardwareMap.get(DcMotor.class, "rfm");
        leftBack = hardwareMap.get(DcMotor.class, "lbm");
        rightBack = hardwareMap.get(DcMotor.class, "rbm");

        leftLift = hardwareMap.get(DcMotor.class, "ll");
        rightLift = hardwareMap.get(DcMotor.class, "rl");

        leftIntake = hardwareMap.get(DcMotor.class, "ilm");
        rightIntake = hardwareMap.get(DcMotor.class, "irm");
        leftIntakeServo = hardwareMap.get(Servo.class, "ils");
        rightIntakeServo = hardwareMap.get(Servo.class, "irs");
//
        lf = hardwareMap.get(Servo.class, "lf");
        rf = hardwareMap.get(Servo.class, "rf");
//
//        autonHook = hwMap.get(Servo.class, "auto");
//        autonClamp = hwMap.get(Servo.class, "clamp");
//
        liftClamp = hardwareMap.get(Servo.class, "clamp");
        liftExtension = hardwareMap.get(CRServo.class, "extension");
        kicker = hardwareMap.get(Servo.class, "kicker");
//        capstone = hwMap.get(Servo.class, "capstone");
//
//        imu = hwMap.get(BNO055IMU.class, "imu");

//      -----------------------------------------------------------------------------------------

        // set motor directions

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // stop and reset the encoders

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the mode to brake; no power = stop immediately

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set lift direction and stop and reset lift motor encoders

        rightLift.setDirection(DcMotor.Direction.REVERSE);

        // set mode to brake

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
//
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//        imu.initialize(parameters);
//        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // run using encoders

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Initialized: ", "YAY");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.guide) {
                drive("tank");
            }

            if(gamepad1.start) {
                drive("mecanum");
            }

            lift();
            fh();
            intake();
        }
    }

    public void lift() {
        leftLift.setPower(gamepad2.left_stick_y);
        rightLift.setPower(-gamepad2.left_stick_y);

        if(gamepad2.left_bumper)
            kicker.setPosition(0.1);

        if(gamepad2.right_bumper)
            kicker.setPosition(1);


        liftExtension.setPower(-gamepad2.right_stick_y);

        if (gamepad2.x)
            liftClamp.setPosition(0.20); //clamp position

        if(gamepad2.y)
            liftClamp.setPosition(0.6); //open position
    }

    public void intake() {
        if (gamepad2.dpad_right) { //Intake Position
            rightIntakeServo.setPosition(0.27);
            leftIntakeServo.setPosition(0.64);
        }


        if(gamepad2.left_trigger > 0) { // intake
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }

        if(gamepad2.right_trigger > 0) { // outtake
            leftIntake.setPower(-1);
            rightIntake.setPower(1);
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void fh() {
        if(gamepad2.a) {
            lf.setPosition(0.3);
            rf.setPosition(0.3);
        }

        if(gamepad2.b) {
            lf.setPosition(0.45);
            rf.setPosition(0.45);
        }
    }

    public void drive(String mode) {
        gamepad1.setJoystickDeadzone(0.05f);

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
        lFrontSpeed = magnitude * (Math.sin(theta + Math.PI/4)) + (1/t * rotate);
        lBackSpeed = magnitude * (Math.sin(theta - Math.PI/4)) + (1/t * rotate);
        rFrontSpeed = magnitude * (Math.sin(theta - Math.PI/4)) - (1/t * rotate);
        rBackSpeed = magnitude * (Math.sin(theta + Math.PI/4)) - (1/t * rotate);

        if(mode.equals("tank")) {
            // tank
            leftFront.setPower(y + rotate);
            rightFront.setPower(y - rotate);
            leftBack.setPower(y + rotate);
            rightBack.setPower(y - rotate);
        } else {
            // mecanum
            leftFront.setPower(lFrontSpeed);
            leftBack.setPower(lBackSpeed);
            rightFront.setPower(rFrontSpeed);
            rightBack.setPower(rBackSpeed);
        }
    }
}
