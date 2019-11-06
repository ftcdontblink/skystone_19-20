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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

public class MainClass extends LinearOpMode {

    public DcMotor lFrontMotor;
    public DcMotor rFrontMotor;
    public DcMotor rBackMotor;
    public DcMotor lBackMotor;
    public Servo ServoLeft;
    public Servo ServoRight;
    public Servo ServoStone;

    public double lFrontSpeed;
    public double lBackSpeed;
    public double rFrontSpeed;
    public double rBackSpeed;
    public final double FperS = 1.68;

    public double translateY; // -gamepad1.left_stick_y
    public double translateX; // -gamepad1.left_stick_x
    public double rotate;     // -gamepad1.right_stick_x
    public double deadzone = 0.05; // deadzone
    public int motorScale;

    public double leftstartAngle = 0;
    public double rightStartAngle = 0.75;
    public double leftterminalAngle = 0.6;
    public double rightterminalAngle = 0.15;
    public double stoneStartAngle = 0.5;
    public double stoneterminalAngle = 0.95;

    //Setting Motor values
    public ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 26.9;
    static final double     FINAL_DRIVE_REDUCTION   = 2.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH =(COUNTS_PER_MOTOR_REV * FINAL_DRIVE_REDUCTION* DRIVE_GEAR_REDUCTION)/ (Math.PI*WHEEL_DIAMETER_INCHES);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.2;

    HardwareMap hwMap;

    public MainClass() {

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void init (HardwareMap h) {
        hwMap = h;
        lFrontMotor = hwMap.get(DcMotor.class, "left_Front_Motor");
        rFrontMotor = hwMap.get(DcMotor.class, "right_Front_Motor");
        lBackMotor = hwMap.get(DcMotor.class, "left_Back_Motor");
        rBackMotor = hwMap.get(DcMotor.class, "right_Back_Motor");
        ServoLeft = hwMap.get(Servo.class, "servo_left");
        ServoRight = hwMap.get(Servo.class, "servo_right");
        ServoStone = hwMap.get(Servo.class, "servo_stone");


        //Set left motors to reverse
        rBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        rFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set motor power to zero.
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
    }

    public void buildingZoneRed(boolean op) {
        EncoderMove(-5, op);
        EncoderStrafe(-36, op);
    }

    public void buildingZoneBlue(boolean op) {
        EncoderMove(-5, op);
        EncoderStrafe(36, op);
    }

    public void loadingZoneRed(boolean op) {
        EncoderStrafe( -26, opModeIsActive());
        EncoderMove(36, opModeIsActive());
    }

    public void loadingZoneBlue(boolean op) {
        EncoderStrafe(-33, opModeIsActive());
        sleep(1000);
        ServoStone.setPosition(0.95);
        sleep(1000);
        EncoderStrafe(30, opModeIsActive());
        EncoderMove(-38, opModeIsActive());
        sleep(1000);
        ServoStone.setPosition(0.5);
        sleep(1000);
        EncoderMove(46, opModeIsActive());
        EncoderStrafe(-32, opModeIsActive());
        sleep(1000);
        ServoStone.setPosition(0.95);
        sleep(1000);
        EncoderStrafe(28, opModeIsActive());
        sleep(1000);
        ServoStone.setPosition(0.5);
        sleep(1000);
        EncoderMove(16, opModeIsActive());

    }

    public void SafetyZoneRed() {}

    public void SafetyZoneBlue() {}

    public void EncoderMove(int inches, boolean opMode) {
        boolean op = opMode;

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = lFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newRightFrontTarget = rFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newLeftBackTarget = lBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newRightBackTarget = rBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        lFrontMotor.setTargetPosition(newLeftFrontTarget);
        lBackMotor.setTargetPosition(newLeftBackTarget);
        rBackMotor.setTargetPosition(newRightBackTarget);
        rFrontMotor.setTargetPosition(newRightFrontTarget);

        // Turn On RUN_TO_POSITION
        lFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        //TODO Wouldnt this actually run the motors and be the motion in the program?
        runtime.reset();
        lFrontMotor.setPower(Math.abs(0.5));
        lBackMotor.setPower(Math.abs(0.5));
        rBackMotor.setPower(Math.abs(0.5));
        rFrontMotor.setPower(Math.abs(0.5));

        while (op == true &&
                (runtime.seconds() < 30) &&
                (lFrontMotor.isBusy() && lBackMotor.isBusy() || rFrontMotor.isBusy() && rBackMotor.isBusy())) {
            //TODO The isBusy check is at the beggining of the while opModeIsActive
            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
//            telemetry.addData("Path2",  "Running at %7d :%7d:%7d :%7d",
//                    lFrontMotor.getCurrentPosition(),
//                    lBackMotor.getCurrentPosition(),
//                    rBackMotor.getCurrentPosition(),
//                    rFrontMotor.getCurrentPosition());
//            telemetry.update();
        }

        // Stop all motion;
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);
    }

    public void EncoderStrafe(int inches, boolean op) {

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = lFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        newRightFrontTarget = rFrontMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newLeftBackTarget = lBackMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
        newRightBackTarget = rBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        lFrontMotor.setTargetPosition(newLeftFrontTarget);
        lBackMotor.setTargetPosition(newLeftBackTarget);
        rBackMotor.setTargetPosition(newRightBackTarget);
        rFrontMotor.setTargetPosition(newRightFrontTarget);

        // Turn On RUN_TO_POSITION
        lFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        //TODO Wouldnt this actually run the motors and be the motion in the program?
        runtime.reset();
        lFrontMotor.setPower(Math.abs(0.5));
        lBackMotor.setPower(Math.abs(0.5));
        rBackMotor.setPower(Math.abs(0.5));
        rFrontMotor.setPower(Math.abs(0.5));

        while (op == true &&
                (runtime.seconds() < 30) &&
                (lFrontMotor.isBusy() && lBackMotor.isBusy() && rFrontMotor.isBusy() && rBackMotor.isBusy())) {
            //TODO The isBusy check is at the beggining of the while opModeIsActive
            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
//            telemetry.addData("Path2",  "Running at %7d :%7d:%7d :%7d",
//                    lFrontMotor.getCurrentPosition(),
//                    lBackMotor.getCurrentPosition(),
//                    rBackMotor.getCurrentPosition(),
//                    rFrontMotor.getCurrentPosition());
//            telemetry.update();
        }

        // Stop all motion;
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);
    }
}
