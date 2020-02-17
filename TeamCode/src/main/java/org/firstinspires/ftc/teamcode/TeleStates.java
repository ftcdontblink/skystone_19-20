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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * @author - Aarush Sharma, Abhinav Keswani, Arin Aggarwal, Mahija Mogalipuvvu
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

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    // Defining Motors

    Robot robot;
    Drive drive;
    FoundationHooks foundationHooks;
    Intake intake;
    Lift lift;
    Claw claw;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new Robot(hardwareMap, telemetry);
        drive = new Drive(robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack);
        foundationHooks = new FoundationHooks(robot.leftFoundation, robot.rightFoundation);
        intake = new Intake(robot.leftIntakeServo, robot.rightIntakeServo, robot.leftIntake, robot.rightIntake);
        lift = new Lift(robot.leftLift, robot.rightLift, robot.liftClamp, robot.liftExtension, robot.capstone);
        claw = new Claw(robot.autonHook, robot.autonClamp);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1, this);
//            foundationHooks.control(gamepad2);
//            intake.control(gamepad2);
            lift.control(gamepad2, this);
//            claw.control(gamepad1);
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