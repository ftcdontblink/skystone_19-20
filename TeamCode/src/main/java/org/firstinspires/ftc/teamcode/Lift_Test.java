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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Lift_Test", group = "Linear Opmode")
//@Disabled
public class Lift_Test extends LinearOpMode {
    boolean Rtrig = false;
    public ElapsedTime runtime = new ElapsedTime(); // Starting an Elapsed Time counter, in seconds
    int lposition = 1; // setting diagnostic state for the switch system
    public DcMotor Lift1; //This will be the left lift motor
    public DcMotor Lift2; //This will be the right lift motor

    MainClass mc = new MainClass();


    @Override
    public void runOpMode() {
        //Initialize the hardware map
        mc.init(hardwareMap);

        //Display the status to the user
        telemetry.addData("Status", "Initialized"); // showing that the robot has been initialized
        telemetry.update();

        //Initialize switch variable
        lposition = 1;

        //Position variables
        int ThirdStone = 17;
        int FourthStone = 22;
        int FifthStone = 27;

        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.a) { //If the a button on gamepad 1 is pressed, add 1 to the current value of lposition
                lposition++;
            }
            if(gamepad1.b) { //If the b button on gamepad 1 is pressed, subtract 1 from the current value of lposition
                lposition--;
            }

            switch(lposition) {
                case 1: //move to the 3rd stone position
                        mc.setLiftTarget(ThirdStone, opModeIsActive());

                    break;
                case 2: //Move to the 4th stone position
                        mc.setLiftTarget(FourthStone, opModeIsActive());
                    break;

                case 3: //Move to the fifth stone positions
                        mc.setLiftTarget(FifthStone, opModeIsActive());
                    break;




            }

            if(gamepad1.dpad_left)
            {
                mc.placeStone();
            }


            }
        }






    }



