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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Move", group="Pushbot")

public class Move extends LinearOpMode {

    MainClass2 mc = new MainClass2();
    public ElapsedTime     runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        mc.init(hardwareMap, imu, lastAngles);
        mc.resetAngle();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            EncoderMove(60, 0.4, 0.4, opModeIsActive());
        }
    }

    public void EncoderMove(int inches, double power1, double power2, boolean opMode) {
        mc.correction = mc.checkDirection();

        boolean op = opMode;

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = mc.lFrontMotor.getCurrentPosition() + (int) (inches * mc.COUNTS_PER_INCH);
        newRightFrontTarget = mc.rFrontMotor.getCurrentPosition() + (int) (inches * mc.COUNTS_PER_INCH);
        newLeftBackTarget = mc.lBackMotor.getCurrentPosition() + (int) (inches * mc.COUNTS_PER_INCH);
        newRightBackTarget = mc.rBackMotor.getCurrentPosition() + (int) (inches * mc.COUNTS_PER_INCH);
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
        mc.lFrontMotor.setPower(power1 - mc.correction);
        mc.lBackMotor.setPower(power1 - mc.correction);
        mc.rBackMotor.setPower(power2 + mc.correction);
        mc.rFrontMotor.setPower(power2 + mc.correction);

        while (op == true &&
                (runtime.seconds() < 30) &&
                (mc.lFrontMotor.isBusy() && mc.lBackMotor.isBusy() || mc.rFrontMotor.isBusy() &&
                        mc.rBackMotor.isBusy())) {
            //TODO The isBusy check is at the beggining of the while opModeIsActive
            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,
//            newRightFrontTarget);
//            telemetry.addData("Path2",  "Running at %7d :%7d:%7d :%7d",
//                    lFrontMotor.getCurrentPosition(),
//                    lBackMotor.getCurrentPosition(),
//                    rBackMotor.getCurrentPosition(),
//                    rFrontMotor.getCurrentPosition());
//            telemetry.update();
        }

        // Stop all motion;
        mc.lFrontMotor.setPower(0);
        mc.lBackMotor.setPower(0);
        mc.rFrontMotor.setPower(0);
        mc.rBackMotor.setPower(0);
    }
}