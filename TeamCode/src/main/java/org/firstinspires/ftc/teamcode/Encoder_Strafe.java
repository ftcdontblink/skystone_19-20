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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Encoder_Strafe_Measurement", group="Linear Opmode")
//@Disabled
/**
 * This is a diagnostic program to test all of the motors, whether it be on the drivetrain
 * or the mechanisms. We are running all of the motors at the same time first, and then
 * individually running each of the motors separately using "A", "B", "X", and "Y".
 */
public class Encoder_Strafe extends LinearOpMode {
    boolean Rtrig = false;
    public ElapsedTime runtime = new ElapsedTime(); // Starting an Elapsed Time counter, in seconds
    int strafe = 1; // setting diagnostic state for the switch system
    public DcMotor lFront; // Defining Motors
    public DcMotor lBack;
    public DcMotor rFront;
    public DcMotor rBack;
    double cpr = 28;
    double x;

    @Override
    public void runOpMode() {

        lFront = hardwareMap.get(DcMotor.class, "left_Front_Motor"); // Defining Motors
        rFront = hardwareMap.get(DcMotor.class, "right_Front_Motor");
        lBack = hardwareMap.get(DcMotor.class, "left_Back_Motor");
        rBack = hardwareMap.get(DcMotor.class, "right_Back_Motor");

        rFront.setDirection(DcMotor.Direction.REVERSE); // The left motors should spin counterclockwise to move forward and the right motors to move clockwise.
        rBack.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized"); // showing that the robot has been initialized
        telemetry.addData("Diagnostic Program", "No boot problems"); // furthermore showing the user the initialization and showing no boot problems
        telemetry.update();

        waitForStart();


        if (opModeIsActive()) {


        }


    }

    public void encoderStrafeRight(double s,
                                   double et,
                                   double t) {

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = lFront.getCurrentPosition() + (int) (et);
            newRightFrontTarget = rFront.getCurrentPosition() + (int) (et);
            newLeftBackTarget = lBack.getCurrentPosition() + (int) (et);
            newRightBackTarget = rBack.getCurrentPosition() + (int) (et);
            lFront.setTargetPosition(newLeftFrontTarget);
            lBack.setTargetPosition(newLeftBackTarget);
            rBack.setTargetPosition(newRightBackTarget);
            rFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            lFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lFront.setPower(s);
            lBack.setPower(-s);
            rFront.setPower(-s);
            rBack.setPower(s);

            // keep looping while we are still active, and there is time left, and all 4 motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < t) &&
                    (lFront.isBusy() && lBack.isBusy() || rFront.isBusy() && rBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d:%7d :%7d",
                        lFront.getCurrentPosition(),
                        lBack.getCurrentPosition(),
                        rBack.getCurrentPosition(),
                        rFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);

            // Turn off RUN_TO_POSITION
            lFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderStrafeLeft(double s,
                                  double et,
                                  double t) {

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = lFront.getCurrentPosition() + (int) (et);
            newRightFrontTarget = rFront.getCurrentPosition() + (int) (et);
            newLeftBackTarget = lBack.getCurrentPosition() + (int) (et);
            newRightBackTarget = rBack.getCurrentPosition() + (int) (et);
            lFront.setTargetPosition(newLeftFrontTarget);
            lBack.setTargetPosition(newLeftBackTarget);
            rBack.setTargetPosition(newRightBackTarget);
            rFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            lFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lFront.setPower(-s);
            lBack.setPower(s);
            rFront.setPower(s);
            rBack.setPower(-s);

            // keep looping while we are still active, and there is time left, and all 4 motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < t) &&
                    (lFront.isBusy() && lBack.isBusy() || rFront.isBusy() && rBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d:%7d :%7d",
                        lFront.getCurrentPosition(),
                        lBack.getCurrentPosition(),
                        rBack.getCurrentPosition(),
                        rFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);

            // Turn off RUN_TO_POSITION
            lFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }




    public void encoderDrive(double speed,
                             double encodertics,
                             double timeoutS) {

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = lFront.getCurrentPosition() + (int)(encodertics);
            newRightFrontTarget = rFront.getCurrentPosition() + (int)(encodertics);
            newLeftBackTarget = lBack.getCurrentPosition() + (int)(encodertics);
            newRightBackTarget = rBack.getCurrentPosition() + (int)(encodertics);
            lFront.setTargetPosition(newLeftFrontTarget);
            lBack.setTargetPosition(newLeftBackTarget);
            rBack.setTargetPosition(newRightBackTarget);
            rFront.setTargetPosition(newRightFrontTarget);

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

            // keep looping while we are still active, and there is time left, and all 4 motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lFront.isBusy() && lBack.isBusy() || rFront.isBusy() && rBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d:%7d :%7d",
                        lFront.getCurrentPosition(),
                        lBack.getCurrentPosition(),
                        rBack.getCurrentPosition(),
                        rFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);

            // Turn off RUN_TO_POSITION
            lFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}




