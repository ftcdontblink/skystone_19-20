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
import com.qualcomm.robotcore.hardware.DcMotor;

public final class Methods {

    Constants constants = new Constants();

    public void move(DcMotor lfm, DcMotor rfm, DcMotor lbm, DcMotor rbm, double inches, LinearOpMode op, double heading, double power) {
        reset(lfm, rfm, lbm, rbm);
        double tp = constants.motorInch(inches);
        noencoder(lfm, rfm, lbm, rbm);
        double avg = 0;
        double correction;

        while(op.opModeIsActive()) {
            correction = heading;
            avg = (Math.abs(lfm.getCurrentPosition())
                    + Math.abs(lbm.getCurrentPosition())
                    + Math.abs(rfm.getCurrentPosition())
                    + Math.abs(rbm.getCurrentPosition()))/4.0;
            if(inches > 0) {
                lfm.setPower(power - correction);
                rfm.setPower(power + correction);
                lbm.setPower(power - correction);
                rbm.setPower(power + correction);
            } else {
                lfm.setPower(-power - correction);
                rfm.setPower(-power + correction);
                lbm.setPower(-power - correction);
                rbm.setPower(-power + correction);
            }

            if(Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        stop(lfm, rfm, lbm, rbm);
        reset(lfm, rfm, lbm, rbm);
        return;
    }

    // ------------------------------------------------------------------------------------------

    public void strafe(DcMotor lfm, DcMotor rfm, DcMotor lbm, DcMotor rbm, double inches, LinearOpMode op, double heading, double power) {
        reset(lfm, rfm, lbm, rbm);
        double tp = constants.motorInch(inches);
        noencoder(lfm, rfm, lbm, rbm);
        double avg = 0;
        double correction;

        while (op.opModeIsActive()) {
            correction = heading;
            avg = (Math.abs(lfm.getCurrentPosition()) + Math.abs(lbm.getCurrentPosition()) + Math.abs(rfm.getCurrentPosition()) + Math.abs(rbm.getCurrentPosition())) / 4.0;
            if (inches > 0) {
                lfm.setPower(power - correction);
                rfm.setPower(-power + correction);
                lbm.setPower(-power - correction);
                rbm.setPower(power + correction);
            } else {
                lfm.setPower(-power - correction);
                rfm.setPower(power + correction);
                lbm.setPower(power - correction);
                rbm.setPower(-power + correction);
            }

            if (Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        stop(lfm, rfm, lbm, rbm);
        reset(lfm, rfm, lbm, rbm);
        return;
    }

    public void moveVel(DcMotor lfm, DcMotor rfm, DcMotor lbm, DcMotor rbm, double inches, LinearOpMode op, double heading) {
        reset(lfm, rfm, lbm, rbm);
        double tp = constants.motorInch(inches);
        noencoder(lfm, rfm, lbm, rbm);
        double avg = 0;
        double cp;
        double correction;

        while(op.opModeIsActive()) {
            correction = heading;
            avg = (Math.abs(lfm.getCurrentPosition())
                    + Math.abs(lbm.getCurrentPosition())
                    + Math.abs(rfm.getCurrentPosition())
                    + Math.abs(rbm.getCurrentPosition()))/4.0;
            cp = avg/tp;

            if(inches > 0) {
                if(cp < 0.2) {
                    lfm.setPower((cp*10 + 0.2)/2.0 - correction);
                    rfm.setPower((cp*10 + 0.2)/2.0 + correction);
                    lbm.setPower((cp*10 + 0.2)/2.0 - correction);
                    rbm.setPower((cp*10 + 0.2)/2.0 + correction);
                } else if(cp > 0.2 && cp < 0.8) {
                    lfm.setPower(1 - correction);
                    rfm.setPower(1 + correction);
                    lbm.setPower(1 - correction);
                    rbm.setPower(1 + correction);
                } else {
                    lfm.setPower(((1-cp)*10 + 0.2)/2.0 - correction);
                    rfm.setPower(((1-cp)*10 + 0.2)/2.0 + correction);
                    lbm.setPower(((1-cp)*10 + 0.2)/2.0 - correction);
                    rbm.setPower(((1-cp)*10 + 0.2)/2.0 + correction);
                }
            } else {
                if(cp < 0.2) {
                    lfm.setPower((cp*10 + 0.2)/2.0 - correction);
                    rfm.setPower((cp*10 + 0.2)/2.0 + correction);
                    lbm.setPower((cp*10 + 0.2)/2.0 - correction);
                    rbm.setPower((cp*10 + 0.2)/2.0 + correction);
                } else if(cp > 0.2 && cp < 0.8) {
                    lfm.setPower(1 - correction);
                    rfm.setPower(1 + correction);
                    lbm.setPower(1 - correction);
                    rbm.setPower(1 + correction);
                } else {
                    lfm.setPower(((1-cp)*10 + 0.2)/2.0 - correction);
                    rfm.setPower(((1-cp)*10 + 0.2)/2.0 + correction);
                    lbm.setPower(((1-cp)*10 + 0.2)/2.0 - correction);
                    rbm.setPower(((1-cp)*10 + 0.2)/2.0 + correction);
                }
            }

            if(Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        stop(lfm, rfm, lbm, rbm);
        reset(lfm, rfm, lbm, rbm);
        return;
    }



    public void reset(DcMotor lfm, DcMotor rfm, DcMotor lbm, DcMotor rbm) {
        lfm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        return;
    }

    public void stop(DcMotor lfm, DcMotor rfm, DcMotor lbm, DcMotor rbm) {
        lfm.setPower(0);
        rfm.setPower(0);
        lbm.setPower(0);
        rbm.setPower(0);

        return;
    }

    public void noencoder(DcMotor lfm, DcMotor rfm, DcMotor lbm, DcMotor rbm) {
        lfm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return;
    }
}