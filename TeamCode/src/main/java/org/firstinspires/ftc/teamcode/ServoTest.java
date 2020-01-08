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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="TeleopLift2", group="Linear Opmode")
//@Disabled
public class ServoTest extends LinearOpMode {

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
    public Servo Clamp;
    public CRServo Extension;
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
    public double turtle = 5;

    public double leftstartAngle = 0.1;
    public double rightStartAngle = 1;
    public double leftterminalAngle = 1;
    public double rightterminalAngle = 0.1;
    public double stoneStartAngle = 0.4;
    public double stoneterminalAngle = 0.9;
    public final double pos = 0.5;
    public final double pos2 = 0.2;
    public final double pos3 = 0.3;
    public final double pos4 = 0.4;
    public int lposition = 0;
    public final int ThirdStone = 5;
    public final int FourthStone = 10;
    public final int FifthStone = 15;
    public final int SixthStone = 20;
    public final int SeventhStone = 25;
    static final int Max_Pos = 38;
    static final int Min_Pos = 0;

    boolean hooks = false;

    static final double LIFT_COUNTS_PER_MOTOR_REV = 4;
    static final double LIFT_GEAR_REDUCTION = 72;
    static final double LIFT_WHEEL_DIAMETER = 2.5; //???
    static final double LIFT_COUNTS_PER_INCH = LIFT_COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION / (Math.PI * LIFT_WHEEL_DIAMETER);

    HardwareMap hwMap; // Defining the hardware map

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lFront = hardwareMap.get(DcMotor.class, "left_Front_Motor"); // Defining Motors
        rFront = hardwareMap.get(DcMotor.class, "right_Front_Motor");
        lBack = hardwareMap.get(DcMotor.class, "left_Back_Motor");
        rBack = hardwareMap.get(DcMotor.class, "right_Back_Motor");
        ServoLeft = hardwareMap.get(Servo.class, "stone_left");      // Defining Servos
        ServoRight = hardwareMap.get(Servo.class, "stone_right");
        ServoStone = hardwareMap.get(Servo.class, "servo_stone");
        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        FlipRight = hardwareMap.get(Servo.class, "flip_right");
        FlipLeft = hardwareMap.get(Servo.class, "flip_left");
        Lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        Lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        Extension = hardwareMap.get(CRServo.class, "extension");
        Clamp = hardwareMap.get(Servo.class, "Clamp");


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

        Lift2.setDirection(DcMotorSimple.Direction.REVERSE);

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

        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FlipRight.setPosition(0.625);
        FlipLeft.setPosition(0.285);

        Lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart(); // Waiting for the start button to be pushed on the phone
        runtime.reset();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                ServoLeft.setPosition(0);
                ServoRight.setPosition(1);
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

