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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * @author - Aarush Sharma, Abhinav Keswani, Arin Aggarwal, Mahija Mogalipuvvu
 * @version - 9/29/19 - Draft 1.0 */
/**
 * This file is the code for a basic mecanum drive which includes the deadzones and a divisor to
 * ensure that our final speeds stay in the range of -1 to 1. This class will be used for Tele-Op
 * which is a driver controlled period, where there is a driver that inputs certain actions through
 * the gamepad, which is read through this code. We set the deadzone to avoid imperfections in the
 * gamepad, to set the signals to 0 when close to no input are detected. Lastly, we set the motor
 * power to the speeds which allows us to translate and rotate easily.
 */

/**
 * For a mecanum drivetrain, we need to be able to calculate the different inputs of the two joysticks including the axes
 * for the left joystick in particular, as the right joystick only controls the rotation.
 *
 * Left joystick will control the translation of the robot in all directions as this is a mecanum drivetrain
 * Right joystick will control the rotation of the robot from its center
 */

@TeleOp(name="Mechanism_Diagnostic", group="Linear Opmode")
public class Pivot_Mechanism_Test extends LinearOpMode {

    MainClass mc = new MainClass();


    private ElapsedTime runtime = new ElapsedTime();


    double pos = 0.5;
    double pos2 = 0.2;
    int Diagnostic = 1;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mc.init(hardwareMap);

        waitForStart(); // Waiting for the start button to be pushed on the phone
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a){
                Diagnostic++;

            }

            if(gamepad1.b) {
                Diagnostic--;
            }

            if(Diagnostic>2 || Diagnostic<1) //For now, this will keep us from getting an error if 'a' or 'b' is pressed two many times
            {
                Diagnostic = 1;
            }


            switch(Diagnostic) {
                case 1:

                    telemetry.addData("Testing:", "Positions");
                    telemetry.update();

                if (gamepad2.right_bumper) {
                    mc.Flip1.setPosition(pos2); //TODO: Change position Values!
                }
                if (gamepad2.left_bumper) {
                    mc.Flip1.setPosition(pos);
                }
                break;

                case 2:

                    telemetry.addData("Testing:", "Intakes");


                    mc.LeftIntake.setPower(gamepad2.right_trigger);
                    mc.RightIntake.setPower(-gamepad2.right_trigger);


                    mc.LeftIntake.setPower(-gamepad2.left_trigger);
                    mc.RightIntake.setPower(gamepad2.left_trigger);

                break;
            }


            switch (mc.position) {
                case 0:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "0");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition(0);
                    mc.Lift.setPower(0.5);
                    break;
                case 1:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "1");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 1) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;
                case 2:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "2");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 2) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;
                case 3:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "3");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 3) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;
                case 4:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "4");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 4) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;
                case 5:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "5");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 5) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;
                case 6:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "6");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 6) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;
                case 7:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "7");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 7) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;
                case 8:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "8");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 8) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;
                default:
                    mc.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Height: ", "0");
                    telemetry.update();
                    mc.Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    mc.Lift.setTargetPosition((int) (mc.LET * 0) + mc.add);
                    mc.Lift.setPower(0.5);
                    break;//Open for mc.position code (positions for stacking)



        }

    }

}}

