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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Diagnostic", group="Linear Opmode")
//@Disabled
/**
 * This is a diagnostic program to test all of the motors, whether it be on the drivetrain
 * or the mechanisms. We are running all of the motors at the same time first, and then
 * individually running each of the motors separately using "A", "B", "X", and "Y".
 */
public class Diagnostic extends LinearOpMode {
    public ElapsedTime     runtime = new ElapsedTime(); // Starting an Elapsed Time counter, in seconds
    int diagnostic = 1; // setting diagnostic state for the switch system
    public DcMotor lFront;
    public DcMotor lBack;
    public DcMotor rFront;
    public DcMotor rBack;
    HardwareMap hwMap = null;
    public void init (HardwareMap ahwMap){
        hwMap = ahwMap;
        lFront = hwMap.get(DcMotor.class, "lFront"); // defining motors
        rFront = hwMap.get(DcMotor.class, "rFront");
        lBack = hwMap.get(DcMotor.class, "lBack");
        rBack = hwMap.get(DcMotor.class, "rBack");
    }

    @Override
    public void runOpMode() {

//TODO Don't you have to call the above init() method, here?
        init(hwMap);

        lFront.setDirection(DcMotor.Direction.REVERSE); // The left motors should spin counterclockwise to move forward and the right motors to move clockwise.
        lBack.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized"); // showing that the robot has been initialized
        telemetry.addData("Diagnostic Program", "No boot problems"); // furthermore showing the user the initialization and showing no boot problems
        telemetry.update();

        waitForStart();

        /**
         * The following code is a state machine for the diagnostics, so we can go between each of the motors with ease.
         * When the driver presses "A", we switch the state to power the left front motor.
         * When they press "B", we switch power to the right Front motor. Pressing "X" switches power to left Back motor
         * Pressing "Y" switches power to the right back motor. The default state is lets you use all the motors at once.
         * We are using the left gamepad stick to move the left motors and the right gamepad stick to move the right motors.
        **/
        /**
         * Button    | Motor
         * ==========|===========
         *      A    |   lFront
         *      B    |   rFront
         *      X    |   lBack
         *      Y    |   rBack
         *  Dpad up  |   lFront & rFront
         *  Dpad down|   lBack & rBack
        **/
//        TODO Please add a comment block with two columns that list the button and motor pairings in an easily - read format.
        while (opModeIsActive())
        {
            if (gamepad1.a) {
                diagnostic = 2; //switches it to the mode where only lFront is running, and all other motors are disabled
            } else if (gamepad1.b) {
                diagnostic = 3;//switches it to the mode where only rFront is running, and all other motors are disabled
            } else if (gamepad1.x) {
                diagnostic = 4;//switches it to the mode where only lBack is running, and all other motors are disabled
            } else if (gamepad1.y) {
                diagnostic = 5;//switches it to the mode where only rBack is running, and all other motors are disabled
            } else if (gamepad1.dpad_up) {
                diagnostic = 6;//switches it to the mode where only front motors are running, and all other motors are disabled
            } else if (gamepad1.dpad_down) {
                diagnostic = 7;//switches it to the mode where only back motors are running, and all other motors are disabled
            }
        switch(diagnostic) {
//                TODO Java syntax: you need a "break" statement after every "case" block, otherwise, all the other cases will run.

//            TODO In cases 2 - 7, please set the inactive motor powers to zero.

            case 1: // first case for the switch machine, will make it so that we can run all motors at the same time (basic driving)
//TODO Add telemetry lines reporting four-wheel operation.

                lFront.setPower(-gamepad1.left_stick_y); // setting mode to basic driving for all motors
                lBack.setPower(-gamepad1.left_stick_y);
                rBack.setPower(-gamepad1.right_stick_y);
                rFront.setPower(-gamepad1.right_stick_y);

                telemetry.addData("Mode: ", "All motors running");
                telemetry.update();
                break;

            case 2: // Only controlling the Front Left Wheel


                telemetry.addData("Mode: ", "Left Front Wheel");
                telemetry.update();


                lFront.setPower(-gamepad1.left_stick_y);
                lBack.setPower(0);       // Setting the other motors to 0 power so they will not move
                rBack.setPower(0);
                rFront.setPower(0);
                break;

            case 3: // Only controlling the Front Right Wheel

                rFront.setPower(-gamepad1.right_stick_y);
                lFront.setPower(0);      // Setting the other motors to 0 power so they will not move
                lBack.setPower(0);
                rBack.setPower(0);

                telemetry.addData("Mode: ", "Right Front Wheel");
                telemetry.update();
                break;

            case 4: // Only controlling the Back Left Wheel

                telemetry.addData("Mode: ", "Left Back Wheel");
                telemetry.update();

                lBack.setPower(-gamepad1.left_stick_y);
                lFront.setPower(0);      // Setting the other motors to 0 power so they will not move
                rFront.setPower(0);
                rBack.setPower(0);

                break;

            case 5: // Only controlling the Back Right Wheel

                telemetry.addData("Mode: ", "Right Back Wheel");
                telemetry.update();

                rBack.setPower(-gamepad1.right_stick_y);
                lFront.setPower(0);      // Setting the other motors to 0 power so they will not move
                rFront.setPower(0);
                lBack.setPower(0);

                break;
            case 6: // Only controlling the Front wheels

                telemetry.addData("Mode: ", "Front wheels");
                telemetry.update();

                rFront.setPower(-gamepad1.right_stick_y);
                lFront.setPower(-gamepad1.left_stick_y);
                rBack.setPower(0);       // Setting the other motors to 0 power so they will not move
                lBack.setPower(0);

                break;
            case 7: // Only controlling the Back wheels

                telemetry.addData("Mode: ", "Back wheels");
                telemetry.update();

                rBack.setPower(-gamepad1.right_stick_y);
                lBack.setPower(-gamepad1.left_stick_y);
                rFront.setPower(0);      // Setting the other motors to 0 power so they will not move
                lFront.setPower(0);

                break;

            case 8: // Running all four motors forward and returning encoder ticks

                powerDrive(0.25, 5);

                sleep(5000);


        }
            }






        }

    public void powerDrive(double power, double timeoutS){ // creating a method to easily input the power and the time limit (in seconds)

        runtime.reset();
        while(runtime.seconds() < timeoutS){ // Conditional so that while the runtime is less than the time limit, the code within will run
            rBack.setPower(power); // sets all motors to the power declared when calling the method
            lFront.setPower(power);
            rFront.setPower(power);
            lBack.setPower(power);


        }
        rBack.setPower(0); // sets all motors to 0 power, so that they cannot move after the timeout is passed.
        lFront.setPower(0);     //ok
        rFront.setPower(0);
        lBack.setPower(0);

    }

    }