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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Red Build Site", group="Pushbot")
@Disabled
public class Red_Build_Site extends LinearOpMode {

    MainClass mc = new MainClass();
    public ElapsedTime     runtime = new ElapsedTime();
    public Servo ServoStone;
    public Servo sl;
    public Servo sr;
    public double leftstartAngle = 0;
    public double rightStartAngle = 0.75;
    public double leftterminalAngle = 0.6;
    public double rightterminalAngle = 0.15;

    @Override
    public void runOpMode() {
        mc.init(hardwareMap);

        sl = hardwareMap.get(Servo.class, "servo_left");
        sr = hardwareMap.get(Servo.class, "servo_right");

        ServoStone = hardwareMap.get(Servo.class, "servo_stone");
        sl.setPosition(leftstartAngle);
        sr.setPosition(rightStartAngle);
        ServoStone.setPosition(0.5);

        waitForStart();
        runtime.reset();

        if(opModeIsActive()) {
            EncoderMove(-30);
            EncoderStrafe(-10);
            sleep(1000);
            sl.setPosition(leftterminalAngle);
            sr.setPosition(rightterminalAngle);
            sleep(2000);
            EncoderMove(31);
            sleep(1000);
            sl.setPosition(leftstartAngle);
            sr.setPosition(rightStartAngle);
            sleep(2000);
            EncoderStrafe(52);

        }
    }

    public void EncoderMove(int inches) {
        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = mc.lFrontMotor.getCurrentPosition() + (int)(inches * mc.COUNTS_PER_INCH);
        newRightFrontTarget = mc.rFrontMotor.getCurrentPosition() + (int)(inches * mc.COUNTS_PER_INCH);
        newLeftBackTarget = mc.lBackMotor.getCurrentPosition() + (int)(inches * mc.COUNTS_PER_INCH);
        newRightBackTarget = mc.rBackMotor.getCurrentPosition() + (int)(inches * mc.COUNTS_PER_INCH);
        mc.lFrontMotor.setTargetPosition(newLeftFrontTarget);
        mc.lBackMotor.setTargetPosition(newLeftBackTarget);
        mc.rBackMotor.setTargetPosition(newRightBackTarget);
        mc.rFrontMotor.setTargetPosition(newRightFrontTarget);

        // Turn On RUN_TO_POSITION
        mc.lFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mc.lBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mc.rFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mc.rBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        //TODO Wouldnt this actually run the motors and be the motion in the program?
        runtime.reset();
        mc.lFrontMotor.setPower(Math.abs(0.6));
        mc.lBackMotor.setPower(Math.abs(0.6));
        mc.rBackMotor.setPower(Math.abs(0.6));
        mc.rFrontMotor.setPower(Math.abs(0.6));

        while (opModeIsActive() &&
                (runtime.seconds() < 30) &&
                (mc.lFrontMotor.isBusy() && mc.lBackMotor.isBusy() || mc.rFrontMotor.isBusy() && mc.rBackMotor.isBusy())) {
            //TODO The isBusy check is at the beggining of the while opModeIsActive
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d:%7d :%7d",
                    mc.lFrontMotor.getCurrentPosition(),
                    mc.lBackMotor.getCurrentPosition(),
                    mc.rBackMotor.getCurrentPosition(),
                    mc.rFrontMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        mc.lFrontMotor.setPower(0);
        mc.lBackMotor.setPower(0);
        mc.rFrontMotor.setPower(0);
        mc.rBackMotor.setPower(0);
    }

    public void EncoderStrafe(int inches) {
        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = mc.lFrontMotor.getCurrentPosition() + (int)(inches * mc.COUNTS_PER_INCH);
        newRightFrontTarget = mc.rFrontMotor.getCurrentPosition() - (int)(inches * mc.COUNTS_PER_INCH);
        newLeftBackTarget = mc.lBackMotor.getCurrentPosition() - (int)(inches * mc.COUNTS_PER_INCH);
        newRightBackTarget = mc.rBackMotor.getCurrentPosition() + (int)(inches * mc.COUNTS_PER_INCH);
        mc.lFrontMotor.setTargetPosition(newLeftFrontTarget);
        mc.lBackMotor.setTargetPosition(newLeftBackTarget);
        mc.rBackMotor.setTargetPosition(newRightBackTarget);
        mc.rFrontMotor.setTargetPosition(newRightFrontTarget);

        // Turn On RUN_TO_POSITION
        mc.lFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mc.lBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mc.rFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mc.rBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        //TODO Wouldnt this actually run the motors and be the motion in the program?
        runtime.reset();
        mc.lFrontMotor.setPower(Math.abs(0.6));
        mc.lBackMotor.setPower(Math.abs(0.6));
        mc.rBackMotor.setPower(Math.abs(0.6));
        mc.rFrontMotor.setPower(Math.abs(0.6));

        while (opModeIsActive() &&
                (runtime.seconds() < 30) &&
                (mc.lFrontMotor.isBusy() && mc.lBackMotor.isBusy() || mc.rFrontMotor.isBusy() && mc.rBackMotor.isBusy())) {
            //TODO The isBusy check is at the beggining of the while opModeIsActive
            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d:%7d :%7d",
                    mc.lFrontMotor.getCurrentPosition(),
                    mc.lBackMotor.getCurrentPosition(),
                    mc.rBackMotor.getCurrentPosition(),
                    mc.rFrontMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        mc.lFrontMotor.setPower(0);
        mc.lBackMotor.setPower(0);
        mc.rFrontMotor.setPower(0);
        mc.rBackMotor.setPower(0);
    }
}