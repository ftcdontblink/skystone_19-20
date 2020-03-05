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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

@Autonomous(name="NoDetect", group="Linear Opmode")

public class NoDetect extends LinearOpMode {

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    MainClass mc = new MainClass();
    double program = 0;
    double park = 5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        mc.init(hardwareMap, imu, lastAngles);

        while(!opModeIsActive()) {
            if(gamepad1.a) {
                program = 0;
                telemetry.addData("Program: ", "PARK");
            }

            if(gamepad1.b) {
                program = 1;
                telemetry.addData("Program: ", "FOUNDATION BLUE WALL");
            }

            if(gamepad1.x) {
                program = 2;
                telemetry.addData("Program: ", "FOUNDATION RED WALL");
            }

            if(gamepad1.y) {
                program = 3;
                telemetry.addData("Program: ", "FOUNDATION BLUE BRIDGE");
            }

            if(gamepad1.y) {
                program = 4;
                telemetry.addData("Program: ", "FOUNDATION BLUE BRIDGE");
            }

            if(gamepad1.left_bumper) {
                program = 5;
                telemetry.addData("Program: ", "BASIC PARK");
            }

            if(gamepad1.right_bumper) {
                program = 6;
                telemetry.addData("Program: ", "BASIC PARK");
            }

            if(gamepad2.a && program == 0) {
                park++;
                telemetry.addData("Park amount: ", park);
            }

            if(gamepad2.b && program == 0) {
                park--;
                telemetry.addData("Park amount: ", park);
            }

            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            switch((int) program) {
                case 0:
                    mc.EncoderMove(park, this);
                    sleep(30000);
                    break;
                case 1:
                    mc.EncoderStrafe(-15, this);
                    mc.leftFoundation.setPosition(0.75); //up posiiton
                    mc.rightFoundation.setPosition(0.75);
                    mc.EncoderMove(-5, this);
                    mc.EncoderStrafe(-25, this);
                    sleep(400);
                    mc.leftFoundation.setPosition(0.3); //downpos
                    mc.rightFoundation.setPosition(0.3);
                    sleep(1200);
                    mc.EncoderStrafe(30, this);
                    mc.rotate(80, 1, this);
                    sleep(200);
                    mc.leftFoundation.setPosition(0.75); //up posiiton
                    mc.rightFoundation.setPosition(0.75);
                    sleep(200);
                    mc.EncoderMove(-7.5, this);
                    mc.EncoderStrafe(40, this);
                    sleep(30000);
                    break;
                case 2:
                    mc.EncoderStrafe(-15, this);
                    mc.leftFoundation.setPosition(0.75); //up posiiton
                    mc.rightFoundation.setPosition(0.75);
                    mc.EncoderMove(5, this);
                    mc.EncoderStrafe(-25, this);
                    sleep(400);
                    mc.leftFoundation.setPosition(0.3); //downpos
                    mc.rightFoundation.setPosition(0.3);
                    sleep(1200);
                    mc.EncoderStrafe(30, this);
                    mc.rotate(-80, 1, this);
                    sleep(200);
                    mc.leftFoundation.setPosition(0.75); //up posiiton
                    mc.rightFoundation.setPosition(0.75);
                    sleep(200);
                    mc.EncoderMove(15, this);
                    mc.EncoderStrafe(40, this);
                    sleep(30000);
                    break;
                case 3:
                    mc.EncoderStrafe(-15, this);
                    mc.leftFoundation.setPosition(0.75); //up posiiton
                    mc.rightFoundation.setPosition(0.75);
                    mc.EncoderMove(-5, this);
                    mc.EncoderStrafe(-25, this);
                    sleep(400);
                    mc.leftFoundation.setPosition(0.3); //downpos
                    mc.rightFoundation.setPosition(0.3);
                    sleep(1200);
                    mc.EncoderStrafe(30, this);
                    mc.rotate(80, 1, this);
                    sleep(200);
                    mc.leftFoundation.setPosition(0.75); //up posiiton
                    mc.rightFoundation.setPosition(0.75);
                    sleep(200);
                    mc.EncoderMove(15, this);
                    mc.EncoderStrafe(40, this);
                    sleep(30000);
                    break;
                case 4:
                    mc.EncoderStrafe(-15, this);
                    mc.leftFoundation.setPosition(0.75); //up posiiton
                    mc.rightFoundation.setPosition(0.75);
                    mc.EncoderMove(5, this);
                    mc.EncoderStrafe(-25, this);
                    sleep(400);
                    mc.leftFoundation.setPosition(0.3); //downpos
                    mc.rightFoundation.setPosition(0.3);
                    sleep(1200);
                    mc.EncoderStrafe(30, this);
                    mc.rotate(-80, 1, this);
                    sleep(200);
                    mc.leftFoundation.setPosition(0.75); //up posiiton
                    mc.rightFoundation.setPosition(0.75);
                    sleep(200);
                    mc.EncoderMove(-7.5, this);
                    mc.EncoderStrafe(40, this);
                    sleep(30000);
                    break;
                case 5:
                    mc.EncoderMove(-4, 1, this);
                    break;
                case 6:
                    mc.customAngle(-32, -24, this);
                    break;
            }
        }
    }
}
