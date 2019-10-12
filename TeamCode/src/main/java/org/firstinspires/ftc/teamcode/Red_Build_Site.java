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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Red Build Site", group="Pushbot")
//@Disabled
public class Red_Build_Site extends LinearOpMode {

    /* Declare OpMode members. */
    // Defining Motors
    public DcMotor lFront;
    public DcMotor lBack;
    public DcMotor rFront;
    public DcMotor rBack;
    public Servo ServoLeft;
    public Servo ServoRight;
    // Defining Motor Speeds
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
    public double leftterminalAngle = 0.75;
    public double rightterminalAngle = 0;
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 26.9 ;    // Counts per the motor revolutions
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        lFront = hardwareMap.get(DcMotor.class, "left_Front_Motor"); // Defining Motors
        rFront = hardwareMap.get(DcMotor.class, "right_Front_Motor");
        lBack = hardwareMap.get(DcMotor.class, "left_Back_Motor");
        rBack = hardwareMap.get(DcMotor.class, "right_Back_Motor");
        ServoLeft = hardwareMap.get(Servo.class, "servo_left");      // Defining Servos
        ServoRight = hardwareMap.get(Servo.class, "servo_right");

        lFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rFront.setDirection(DcMotor.Direction.REVERSE); // The right motors should spin counterclockwise to move forward and the right motors to move clockwise.
        rBack.setDirection(DcMotor.Direction.REVERSE);

        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ServoRight.setPosition(rightStartAngle);
        ServoLeft.setPosition(leftstartAngle);

        waitForStart();
        runtime.reset();


        if(opModeIsActive()){
            // move backward

            lFront.setPower(-1);
            lBack.setPower(-1);
            rFront.setPower(-1);
            rBack.setPower(-1);

            sleep(1300);

            lFront.setPower(-1);
            lBack.setPower(1);
            rFront.setPower(1);
            rBack.setPower(-1);

            sleep(600);

            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);

            sleep(200);
            // foundation grab

            ServoLeft.setPosition(leftterminalAngle);
            ServoRight.setPosition(rightterminalAngle);

            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);

            sleep(400);

            // move forward

            lFront.setPower(1);
            lBack.setPower(1);
            rFront.setPower(1);
            rBack.setPower(1);

            sleep(1630);
            // release foundation

            ServoLeft.setPosition(leftstartAngle);
            ServoRight.setPosition(rightStartAngle);

            // strafe right

            lFront.setPower(1);
            lBack.setPower(-1);
            rFront.setPower(-1);
            rBack.setPower(1);

            sleep(2200);

            // stop

            lFront.setPower(0);
            rFront.setPower(0);
            lBack.setPower(0);
            rBack.setPower(0);
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = lFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = lBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = rBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            lFront.setTargetPosition(newLeftFrontTarget);
            lBack.setTargetPosition(newLeftBackTarget);
            rFront.setTargetPosition(newRightFrontTarget);
            rBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            lFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lFront.setPower(Math.abs(speed));
            lBack.setPower(Math.abs(speed));
            rFront.setPower(Math.abs(speed));
            rBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lFront.isBusy() && lBack.isBusy() && rFront.isBusy() && rBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Left Path: ",  "Running to %7d :%7d", newLeftFrontTarget, newLeftBackTarget);
                telemetry.addData("Right Path: ",  "Running ", newRightBackTarget, newRightFrontTarget);
                lFront.getCurrentPosition();
                lBack.getCurrentPosition();
                rFront.getCurrentPosition();
                rBack.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            lFront.setPower(0);
            lBack.setPower(0);
            rBack.setPower(0);
            rFront.setPower(0);

            // Turn off RUN_TO_POSITION
            lFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
