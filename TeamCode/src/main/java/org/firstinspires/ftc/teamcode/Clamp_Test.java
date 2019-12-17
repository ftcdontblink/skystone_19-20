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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import java.security.Policy;


/**
 * @author abhinavkeswani
 * @version 1.0
 * ========================================================
 *              Controls
 *   gamepad1.a    |    Adds 0.1 to current position
 *   gamepad1.b    |    Subtracts 0.1 from current position
 *   right_bumper  |    Confirm Movement
 * ========================================================
 *
 * */
@TeleOp(name="Clamp_Test", group="Linear Opmode")
public class Clamp_Test extends LinearOpMode {

    MainClass mc = new MainClass();
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        mc.init(hardwareMap);

        mc.Clamp.setPosition(0.3);
        waitForStart(); // Waiting for the start button to be pushed on the phone
        runtime.reset();

        while (opModeIsActive()) {


            if (gamepad1.a) { //Adds 0.1 to the position so we can get the position we need, and displays in telemetry
                mc.Clamp.setPosition(0.48);
            }
            if (gamepad1.b) //Subtracts 0.1 from the position and displays new value
            {
                mc.Clamp.setPosition(0.3);

            }
        }
    }
}



