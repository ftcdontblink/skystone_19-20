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

//TODO The above comment block no longer reflects this code, and should be replaced with one
    //TODO that describes this class as a Mecanum drive class for Tele-Op, and which gamepad
    //TODO control does what.

@TeleOp(name="Mecanum_Drive", group="Linear Opmode")
// @Disabled
public class Mecanum_drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor lFront; // Defining Motors
    public DcMotor lBack;
    public DcMotor rFront;
    public DcMotor rBack;
    public double lFrontSpeed; // Defining Motor Speeds
    public double lBackSpeed;
    public double rFrontSpeed;
    public double rBackSpeed;
    public double translateY; // -gamepad1.left_stick_y
    public double translateX; // -gamepad1.left_stick_x
    public double rotate;     // -gamepad1.right_stick_x
    public double deadzone = 0.05; // deadzone
    public int motorScale;
    HardwareMap hwMap = null; // Defining the hardware map

    public void init (HardwareMap ahwMap){ // Initializing input from the robot and control hub
        hwMap = ahwMap;
        lFront = hwMap.get(DcMotor.class, "lFront"); // defining motors
        rFront = hwMap.get(DcMotor.class, "rFront");
        lBack = hwMap.get(DcMotor.class, "lBack");
        rBack = hwMap.get(DcMotor.class, "rBack");
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        init(hwMap); // Initializing the HardwareMap

        //TODO For discussion: should we drive a Mecanum drivetrain with "Run without encoders", or
        //TODO "Run with encoders"?  The latter may give us better speed matching on the motors,
        //TODO which is important for consistent performance.

        lFront.setDirection(DcMotor.Direction.REVERSE); // The left motors should spin counterclockwise to move forward and the right motors to move clockwise.
        lBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart(); // Waiting for the start button to be pushed on the phone
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            translateX = -gamepad1.left_stick_x; // defining, to reduce processing speeds
            translateY = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;

            motorScale = 0; // set the motorScale = 0 to start with

            /**
             * The motorScale attribute is a divisor to the actual speed, as the gamepad reads inputs from -1 to 1
             * To scale the speeds back, we divide the equations by the number of signals that the gamepad reads
             */

            if (Math.abs(translateX) <= deadzone) {
                translateX = 0;
            } else {
                motorScale++;
            }

            if (Math.abs(translateY) <= deadzone) {
                translateY = 0;
            } else {
                motorScale++;
            }

            if (Math.abs(rotate) <= deadzone) {
                rotate = 0;
            } else {
                motorScale++;
            }

            if(motorScale == 0) { // If divided by 0, an IOException will incur
                motorScale = 1;
            }

            //TODO Please move the "deadzone" comment block up to the top of the "if" sequence.
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

            lFrontSpeed = (translateX + translateY + rotate) / motorScale;
            lBackSpeed  = (translateX + translateY - rotate) / motorScale;
            rFrontSpeed = (translateX - translateY - rotate) / motorScale;
            rBackSpeed  = (translateX - translateY + rotate) / motorScale;

  //TODO Please move this comment block to the top, near the class declaration.
             /**
             * For a mecanum drivetrain, we need to be able to calculate the different inputs of the two joysticks including the axes
             * for the left joystick in particular, as the right joystick only controls the rotation.
             *
             * Left joystick will control the translation of the robot in all directions as this is a mecanum drivetrain
             * Right joystick will control the rotation of the robot from its center
             */


        }
    }
}
