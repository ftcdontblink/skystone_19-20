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

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="NonDetectAuto", group="Linear Opmode")

public class NoDetectAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Robot robot;
    Methods methods;
    int program = 0;
    double park = 5;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new Robot(hardwareMap, telemetry, lastAngles, imu);
        methods = new Methods();

        while(!opModeIsActive()) {
            if(gamepad1.a) {
                program = 0;
                telemetry.addData("Program: ", "PARK");
            }

            if(gamepad1.b) {
                program = 1;
                telemetry.addData("Program: ", "Foundation RED WALL");
            }

            if(gamepad1.a) {
                program = 2;
                telemetry.addData("Program: ", "Foundation BLUE WALL");
            }

            if(gamepad1.a) {
                program = 3;
                telemetry.addData("Program: ", "Foundation BLUE BRIDGE");
            }

            if(gamepad1.a) {
                program = 4;
                telemetry.addData("Program: ", "Foundation BLUE BRIDGE");
            }

            if(gamepad2.a && program == 0) {
                park++;
                telemetry.addData("Park amount: ", park);
            }

            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            DcMotor[] motors = {robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack};
            Servo[] hooks = {robot.leftFoundation, robot.rightFoundation};
            double[] hooksPos = {0.3, 0.45};

            switch(program) {
                case 0:
                    methods.move(motors, park, this, robot.checkDirection(), 1);
                    break;
                case 1:
                    methods.unhook(hooks, hooksPos);
                    methods.strafeVelEq(motors, -5, this, robot.checkDirection());
                    methods.move(motors, -8, this, robot.checkDirection(), 0.4);
                    methods.strafeVelEq(motors, -25, this, robot.checkDirection());
                    methods.hook(hooks, hooksPos);
                    methods.strafe(motors, 10, this, robot.checkDirection(), 0.4);
                    robot.rotate(90, 1, this, motors);
                    methods.unhook(hooks, hooksPos);
                    methods.move(motors, -20, this, robot.checkDirection(), 1);
                    methods.strafeVelEq(motors, 37.5, this, robot.checkDirection());
                    break;
                case 2:
                    break;
                case 3:
                    break;
                case 4:
                    break;
            }
        }
    }
}
