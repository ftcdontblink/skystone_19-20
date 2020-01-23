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


@Autonomous(name="Park", group="Pushbot")

public class RoboRockettesPark extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor lfm;
    public DcMotor rfm;
    public DcMotor lbm;
    public DcMotor rbm;


    @Override
    public void runOpMode() {
        lfm = hardwareMap.get(DcMotor.class, "lfm"); // left front motor name
        rfm = hardwareMap.get(DcMotor.class, "rfm"); // right front motor name
        lbm = hardwareMap.get(DcMotor.class, "lbm"); // left back motor name
        rbm = hardwareMap.get(DcMotor.class, "rbm"); // right back motor name

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            lfm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rfm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lbm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rbm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lfm.setTargetPosition(200);
            rfm.setTargetPosition(200);
            lbm.setTargetPosition(200);
            rbm.setTargetPosition(200);

            lfm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rfm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(lfm.isBusy() && rfm.isBusy() && lbm.isBusy() && rbm.isBusy()) {
                lfm.setPower(0.5);
                rfm.setPower(0.5);
                lbm.setPower(0.5);
                rbm.setPower(0.5);
            }

            lfm.setPower(0);
            rfm.setPower(0);
            lbm.setPower(0);
            rbm.setPower(0);
        }
    }
}