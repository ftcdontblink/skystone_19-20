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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


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

public final class Lift {

    public DcMotor leftLift;
    public DcMotor rightLift;
    public Servo liftClamp;
    public CRServo liftExtension;
    public Servo capstone;

    static final double LIFT_COUNTS_PER_MOTOR_REV = 4;
    static final double LIFT_GEAR_REDUCTION = 72;
    static final double LIFT_WHEEL_DIAMETER = 2.5; //???
    static final double LIFT_COUNTS_PER_INCH = LIFT_COUNTS_PER_MOTOR_REV * LIFT_GEAR_REDUCTION / (Math.PI * LIFT_WHEEL_DIAMETER);

    public Lift(DcMotor ll, DcMotor rl, Servo lc, CRServo le, Servo cap) {
        leftLift = ll;
        rightLift = rl;
        liftClamp = lc;
        liftExtension = le;
        capstone = cap;
    }

    public void control(Gamepad gamepad, LinearOpMode op) {
        while (op.opModeIsActive()) {
            if (leftLift.getCurrentPosition() > 0 && rightLift.getCurrentPosition() > 0
                    && leftLift.getCurrentPosition() < 38.5 * LIFT_COUNTS_PER_INCH && rightLift.getCurrentPosition() > 38.5 * LIFT_COUNTS_PER_INCH) {
                leftLift.setPower(-gamepad.left_stick_y);
                rightLift.setPower(-gamepad.left_stick_y);
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
            }


//        liftExtension.setPower(gamepad.right_stick_y);
//
//        if(gamepad.left_bumper)
//            capstone.setPosition(0);
//
//        if(gamepad.right_bumper)
//            capstone.setPosition(1);
//
//        if (gamepad.x)
//            liftClamp.setPosition(0.67); //clamp position
//
//        if(gamepad.y)
//            liftClamp.setPosition(0.56); //open position
        }
    }
}
