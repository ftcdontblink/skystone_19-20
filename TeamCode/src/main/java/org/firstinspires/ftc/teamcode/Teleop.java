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
 * @author - Aarush Sharma, Abhinav Keswani, Arin Aggarwal, Mahija Mogalipuvvu
 * @version - 9/29/19 - Draft 1.0 */
/**
 * For a mecanum drivetrain, we need to be able to calculate the different inputs of the two joysticks including the axes
 * for the left joystick in particular, as the right joystick only controls the rotation.
 *
 * Left joystick will control the translation of the robot in all directions as this is a mecanum drivetrain
 * Right joystick will control the rotation of the robot from its center
 */

@TeleOp(name="Teleop", group="Linear Opmode")
//@Disabled
public class Teleop extends LinearOpMode {

    MainClass mc = new MainClass();
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    // Defining Motors
    public DcMotor lFront;
    public DcMotor lBack;
    public DcMotor rFront;
    public DcMotor rBack;
    public Servo ServoLeft;
    public Servo ServoRight;
    public Servo ServoStone;
    public Servo FlipLeft;
    public Servo FlipRight;
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public DcMotor Lift1;
    public DcMotor Lift2;
    // Defining Motor Speeds
    public double lFrontSpeed;
    public double lBackSpeed;
    public double rFrontSpeed;
    public double rBackSpeed;
    public double translateY; // -gamepad1.left_stick_y
    public double translateX; // -gamepad1.left_stick_x
    public double rotate;     // -gamepad1.right_stick_x
    public double deadzone = 0.05; // deadzone
    public double motorScale;

    public double leftstartAngle = 0.1;
    public double rightStartAngle = 0.65;
    public double leftterminalAngle = 0.6;
    public double rightterminalAngle = 0.15;
    public double stoneStartAngle = 0.4;
    public double stoneterminalAngle = 0.9;
    public final double pos = 0.5;
    public final double pos2 = 0.2;
    public final double pos3 = 0.3;
    public final double pos4 = 0.4;
    public int          lposition = 1;
    public final int    ThirdStone = 17;
    public final int    FourthStone = 22;
    public final int    FifthStone = 27;

    HardwareMap hwMap; // Defining the hardware map

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lFront = hardwareMap.get(DcMotor.class, "left_Front_Motor"); // Defining Motors
        rFront = hardwareMap.get(DcMotor.class, "right_Front_Motor");
        lBack = hardwareMap.get(DcMotor.class, "left_Back_Motor");
        rBack = hardwareMap.get(DcMotor.class, "right_Back_Motor");
        ServoLeft = hardwareMap.get(Servo.class, "servo_left");      // Defining Servos
        ServoRight = hardwareMap.get(Servo.class, "servo_right");
        ServoStone = hardwareMap.get(Servo.class, "servo_stone");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        FlipRight = hardwareMap.get(Servo.class, "flip_right");
        FlipLeft = hardwareMap.get(Servo.class, "flip_left");
        Lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        Lift2 = hardwareMap.get(DcMotor.class, "Lift2");




        //mc.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //mc.Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mc.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mc.Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rFront.setDirection(DcMotor.Direction.REVERSE); // The right motors should spin counterclockwise to move forward and the left motors to move clockwise.
        rBack.setDirection(DcMotor.Direction.REVERSE);

        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        lFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ServoStone.setPosition(stoneStartAngle); //Initializes the servo stone
//        FlipLeft.setPosition(0.29);//initilaizes the flip mechanism
//        FlipRight.setPosition(0.50);
        ServoLeft.setPosition(leftstartAngle); //initializes the foundation hooks
        ServoRight.setPosition(rightStartAngle);

        FlipRight.setPosition(0.63);
        FlipLeft.setPosition(0.29);

        waitForStart(); // Waiting for the start button to be pushed on the phone
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            translateX = gamepad1.left_stick_x; // defining, to reduce processing speeds
            translateY = -gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;


            motorScale = 0; // set the motorScale = 0 to start with

            /**
             * The motorScale attribute is a divisor to the actual speed, as the gamepad reads inputs from -1 to 1
             * To scale the speeds back, we divide the equations by the number of signals that the gamepad reads
             */

            /*
             * A dead zone is neccesary because no piece of hardware is perfect and the springs in
             * our logitech gamepads are no exception. The springs will not move the joystick all
             * the way back to the (0,0) position. If we leave the code as is, then the robot will
             * drift. Because of this, we need a dead zone. The three "if" loops above make it so
             * that if the joysticks value are a certain distance close to the center point (this
             * value is defined by the double, deadzone), then there will be no effect in the motion
             * of the robot. If there is a motion, the motorScale attribute is incremented.
             * The motorScale is used to divide the speed by the number of signals present.
             */


            if (Math.abs(translateX) <= deadzone) {
                translateX = 0;
            }

            if (Math.abs(translateY) <= deadzone) {
                translateY = 0;
            }

            if (Math.abs(rotate) <= deadzone) {
                rotate = 0;
            }

            if (motorScale == 0) { // If divided by 0, an IOException will incur
                motorScale = 1;
            }

            // Speeds are calculated by the different joystick signals
            // They are capped by the motorScale so the range stays between -1 and 1
            // They are assigned variables to make the code concise and easier to read

            lFrontSpeed = (translateY + translateX + rotate);
            lBackSpeed = (translateY - translateX + rotate);
            rFrontSpeed = (translateY - translateX - rotate);
            rBackSpeed = (translateY + translateX - rotate);

            // setting the power of the motors to the calculated speeds
            if ((Math.abs(lFrontSpeed) > 1) || (Math.abs(lBackSpeed) > 1) || (Math.abs(rFrontSpeed) > 1) || (Math.abs(rBackSpeed) > 1)) {

                motorScale = Math.max(Math.max(Math.abs(lFrontSpeed), Math.abs(lBackSpeed)), Math.max(Math.abs(rFrontSpeed), Math.abs(rBackSpeed)));

            } else {
                motorScale = 1;
            }

            lFrontSpeed = lFrontSpeed / motorScale;
            lBackSpeed = lBackSpeed / motorScale;
            rFrontSpeed = rFrontSpeed / motorScale;
            rBackSpeed = rBackSpeed / motorScale;

            lFront.setPower(lFrontSpeed);
            lBack.setPower(lBackSpeed);
            rFront.setPower(rFrontSpeed);
            rBack.setPower(rBackSpeed);
//       *************************************************************************************************
//                                      OPERATOR CONTROLS
//       *************************************************************************************************









//       *************************************************************************************************
//                                        SERVO CONTROLS
//       *************************************************************************************************

            if (gamepad2.a) { // Moves servos to foundation position
                ServoLeft.setPosition(leftterminalAngle);
                ServoRight.setPosition(rightterminalAngle);
            }

            if (gamepad2.b) { // Brings Robot Back to Start Angles (Inside size limit)
                ServoLeft.setPosition(leftstartAngle);
                ServoRight.setPosition(rightStartAngle);
            }
            if (gamepad2.x) { //Grabs Stone
                ServoStone.setPosition(stoneterminalAngle);
            }
            if (gamepad2.y) { //Releases Stone
                ServoStone.setPosition(stoneStartAngle);
            }

            if(gamepad2.left_bumper) { //Intake Position
                FlipRight.setPosition(0.63);
                FlipLeft.setPosition(0.29);
            }

            if(gamepad2.right_bumper) { // Highest (Init) Position
                FlipRight.setPosition(0.520);
                FlipLeft.setPosition(0.4);
            }

            if(gamepad2.dpad_left) { // Spit Position
                FlipRight.setPosition(0.5575);
                FlipLeft.setPosition(0.362);
            }

//       *************************************************************************************************
//                                        INTAKE CONTROLS
//       *************************************************************************************************

            leftIntake.setPower(gamepad2.right_trigger);
            rightIntake.setPower(-gamepad2.right_trigger);


            leftIntake.setPower(-gamepad2.left_trigger);
            rightIntake.setPower(gamepad2.left_trigger);


//       *************************************************************************************************
//                                        LIFT CONTROLS
//       *************************************************************************************************

            switch(lposition) {
                case 1: //move to the 3rd stone position
                    mc.setLiftTarget(ThirdStone, opModeIsActive());

                    break;
                case 2: //Move to the 4th stone position
                    mc.setLiftTarget(FourthStone, opModeIsActive());
                    break;

                case 3: //Move to the fifth stone positions
                    mc.setLiftTarget(FifthStone, opModeIsActive());
                    break;
            }




        }

    }
}

/**
 * This file is the code for a basic mecanum drive which includes the deadzones and a divisor to
 * ensure that our final speeds stay in the range of -1 to 1. This class will be used for Tele-Op
 * which is a driver controlled period, where there is a driver that inputs certain actions through
 * the gamepad, which is read through this code. We set the deadzone to avoid imperfections in the
 * gamepad, to set the signals to 0 when close to no input are detected. Lastly, we set the motor
 * power to the speeds which allows us to translate and rotate easily.
 */

