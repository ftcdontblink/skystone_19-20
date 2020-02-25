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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public final class Methods {

    Constants constants = new Constants();

    public void move(DcMotor[] ms, double inches, LinearOpMode op, double heading, double power) {
        DcMotor lfm = ms[0];
        DcMotor lbm = ms[1];
        DcMotor rfm = ms[2];
        DcMotor rbm = ms[3];

        reset(lfm, rfm, lbm, rbm);
        double tp = constants.motorInch(inches);
        noencoder(lfm, rfm, lbm, rbm);
        double avg = 0;
        double correction;

        while (op.opModeIsActive()) {
            correction = heading;
            avg = (Math.abs(lfm.getCurrentPosition())
                    + Math.abs(lbm.getCurrentPosition())
                    + Math.abs(rfm.getCurrentPosition())
                    + Math.abs(rbm.getCurrentPosition())) / 4.0;
            if (inches > 0) {
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

            if (Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        stop(lfm, rfm, lbm, rbm);
        reset(lfm, rfm, lbm, rbm);
        return;
    }

    // ------------------------------------------------------------------------------------------

    public void strafe(DcMotor[] ms, double inches, LinearOpMode op, double heading, double power) {
        DcMotor lfm = ms[0];
        DcMotor lbm = ms[1];
        DcMotor rfm = ms[2];
        DcMotor rbm = ms[3];

        reset(lfm, rfm, lbm, rbm);
        double tp = constants.motorInch(inches);
        noencoder(lfm, rfm, lbm, rbm);
        double avg = 0;

        while (op.opModeIsActive()) {
            avg = (Math.abs(lfm.getCurrentPosition()) + Math.abs(lbm.getCurrentPosition()) + Math.abs(rfm.getCurrentPosition()) + Math.abs(rbm.getCurrentPosition())) / 4.0;
            if (inches > 0) {
                lfm.setPower(power - heading);
                rfm.setPower(-power + heading);
                lbm.setPower(-power - heading);
                rbm.setPower(power + heading);
            } else {
                lfm.setPower(-power - heading);
                rfm.setPower(power + heading);
                lbm.setPower(power - heading);
                rbm.setPower(-power + heading);
            }

            if (Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        stop(lfm, rfm, lbm, rbm);
        reset(lfm, rfm, lbm, rbm);
        return;
    }

    public void moveVel(DcMotor[] ms, double inches, LinearOpMode op, double heading) {
        DcMotor lfm = ms[0];
        DcMotor lbm = ms[1];
        DcMotor rfm = ms[2];
        DcMotor rbm = ms[3];

        reset(lfm, rfm, lbm, rbm);
        double tp = constants.motorInch(inches);
        noencoder(lfm, rfm, lbm, rbm);
        double avg = 0;
        double cp;
        double correction;

        while (op.opModeIsActive()) {
            correction = heading;
            avg = (Math.abs(lfm.getCurrentPosition())
                    + Math.abs(lbm.getCurrentPosition())
                    + Math.abs(rfm.getCurrentPosition())
                    + Math.abs(rbm.getCurrentPosition())) / 4.0;
            cp = avg / tp;

            if (inches > 0) {
                if (cp < 0.2) {
                    lfm.setPower((cp * 10 + 0.2) / 2.0 - correction);
                    rfm.setPower((cp * 10 + 0.2) / 2.0 + correction);
                    lbm.setPower((cp * 10 + 0.2) / 2.0 - correction);
                    rbm.setPower((cp * 10 + 0.2) / 2.0 + correction);
                } else if (cp > 0.2 && cp < 0.8) {
                    lfm.setPower(1 - correction);
                    rfm.setPower(1 + correction);
                    lbm.setPower(1 - correction);
                    rbm.setPower(1 + correction);
                } else {
                    lfm.setPower(((1 - cp) * 10 + 0.2) / 2.0 - correction);
                    rfm.setPower(((1 - cp) * 10 + 0.2) / 2.0 + correction);
                    lbm.setPower(((1 - cp) * 10 + 0.2) / 2.0 - correction);
                    rbm.setPower(((1 - cp) * 10 + 0.2) / 2.0 + correction);
                }
            } else {
                if (cp < 0.2) {
                    lfm.setPower((cp * 10 + 0.2) / 2.0 - correction);
                    rfm.setPower((cp * 10 + 0.2) / 2.0 + correction);
                    lbm.setPower((cp * 10 + 0.2) / 2.0 - correction);
                    rbm.setPower((cp * 10 + 0.2) / 2.0 + correction);
                } else if (cp > 0.2 && cp < 0.8) {
                    lfm.setPower(1 - correction);
                    rfm.setPower(1 + correction);
                    lbm.setPower(1 - correction);
                    rbm.setPower(1 + correction);
                } else {
                    lfm.setPower(((1 - cp) * 10 + 0.2) / 2.0 - correction);
                    rfm.setPower(((1 - cp) * 10 + 0.2) / 2.0 + correction);
                    lbm.setPower(((1 - cp) * 10 + 0.2) / 2.0 - correction);
                    rbm.setPower(((1 - cp) * 10 + 0.2) / 2.0 + correction);
                }
            }

            if (Math.abs(avg) > Math.abs(tp)) {
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

//    y=-\left|\left(x-5\right)\right|+5


    public void moveVelEq(DcMotor[] ms, double inches, LinearOpMode op, double heading) {
        DcMotor lfm = ms[0];
        DcMotor lbm = ms[1];
        DcMotor rfm = ms[2];
        DcMotor rbm = ms[3];

        reset(lfm, rfm, lbm, rbm);
        double tp = constants.motorInch(inches);
        noencoder(lfm, rfm, lbm, rbm);
        double avg = 0;
        double cp;
        double correction;

        while (op.opModeIsActive()) {
            correction = heading;
            avg = (Math.abs(lfm.getCurrentPosition())
                    + Math.abs(lbm.getCurrentPosition())
                    + Math.abs(rfm.getCurrentPosition())
                    + Math.abs(rbm.getCurrentPosition())) / 4.0;
            cp = avg / Math.abs(tp);

            if (inches > 0) {
                lfm.setPower(Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) + 0.2 - correction);
                rfm.setPower(Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) + 0.2 + correction);
                lbm.setPower(Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) + 0.2 - correction);
                rbm.setPower(Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) + 0.2 + correction);

//                lfm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) + 0.2);
//                rfm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) + 0.2);
//                lbm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) + 0.2);
//                rbm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) + 0.2);





            } else {
                lfm.setPower(-Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) - 0.2 - correction);
                rfm.setPower(-Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) - 0.2 + correction);
                lbm.setPower(-Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) - 0.2 - correction);
                rbm.setPower(-Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) - 0.2 + correction);

//                lfm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) - 0.2);
//                rfm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) - 0.2);
//                lbm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) - 0.2);
//                rbm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) - 0.2);
            }

            // y = -Math.abs(.5(x-5)) + 2.5


            if (Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        stop(lfm, rfm, lbm, rbm);
        reset(lfm, rfm, lbm, rbm);
        return;
    }

    public void strafeVelEq(DcMotor[] ms, double inches, LinearOpMode op, double heading) {
        DcMotor lfm = ms[0];
        DcMotor lbm = ms[1];
        DcMotor rfm = ms[2];
        DcMotor rbm = ms[3];

        reset(lfm, rfm, lbm, rbm);
        double tp = constants.motorInch(inches);
        noencoder(lfm, rfm, lbm, rbm);
        double avg = 0;
        double cp;
        double correction;

        while (op.opModeIsActive()) {
            correction = heading;
            avg = (Math.abs(lfm.getCurrentPosition())
                    + Math.abs(lbm.getCurrentPosition())
                    + Math.abs(rfm.getCurrentPosition())
                    + Math.abs(rbm.getCurrentPosition())) / 4.0;
            cp = avg / Math.abs(tp);

            if (inches > 0) {
                lfm.setPower((Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) + 0.2) - correction);
                rfm.setPower((-Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) - 0.2) + correction);
                lbm.setPower(0.7 * (-Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1)) - 0.2 - correction);
                rbm.setPower(0.7 * (Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1)) + 0.2 + correction);

//                lfm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) + 0.2);
//                rfm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) - 0.2);
//                lbm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) - 0.2);
//                rbm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) + 0.2);
            } else {
                lfm.setPower(((-Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) - 0.2)) - correction);
                rfm.setPower((Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1) + 0.2) + correction);
                lbm.setPower(0.7 * (Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1)) + 0.2 - correction);
                rbm.setPower(0.7 * (-Range.clip((-Math.abs(0.25 * ((cp * 10) - 5))) + 1.25, -1, 1)) - 0.2 + correction);

//                lfm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) - 0.2);
//                rfm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) + 0.2);
//                lbm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) + 0.2);
//                rbm.setPower(Range.clip(((-1/24.0) * cp * (cp - 10)), -1,1) - 0.2);
            }

            // y = -Math.abs(.5(x-5)) + 2.5


            if (Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        stop(lfm, rfm, lbm, rbm);
        reset(lfm, rfm, lbm, rbm);
        return;
    }

    public void hook(Servo[] fh, double[] pos) {
        Servo fl = fh[0];
        Servo fr = fh[1];

        fl.setPosition(pos[0]);
        fr.setPosition(pos[0]);
    }

    public void unhook(Servo[] fh, double[] pos) {
        Servo fl = fh[0];
        Servo fr = fh[1];

        fl.setPosition(pos[1]);
        fr.setPosition(pos[1]);
    }

    public void plough(Servo[] claw, double[] hp, double[] cp) {
        Servo hook = claw[0];
        Servo clamp = claw[1];

        clamp.setPosition(cp[0]);
        hook.setPosition(hp[1]);
    }

    public void clamp(Servo[] claw, double[] hp, double[] cp) {
        Servo hook = claw[0];
        Servo clamp = claw[1];

        hook.setPosition(hp[2]);
        clamp.setPosition(cp[1]);
    }

    public void raise(Servo[] claw, double[] hp, double[] cp) {
        Servo hook = claw[0];
        Servo clamp = claw[1];

        clamp.setPosition(cp[1]);
        hook.setPosition(hp[0]);
    }

    public void intake(DcMotor[] drive, DcMotor[] intake, LinearOpMode op, Servo kicker, double[] kpos, Servo clamp, double[] clamppos, double heading) {
        moveVelEq(drive, 8, op, heading);
        intake[0].setPower(1);
        intake[1].setPower(-1);
        move(drive, 2, op, heading, 0.4);
        kicker.setPosition(kpos[1]);
        clamp.setPosition(clamppos[1]);
        intake[0].setPower(0.2);
        intake[1].setPower(-0.2);
    }

    public void outtake(CRServo ext, LinearOpMode op) {
        ElapsedTime time = new ElapsedTime();
        while (time.milliseconds() < 2100)
            ext.setPower(1);
    }

    public void release(Servo clamp, double[] clamppos) {
        clamp.setPosition(clamppos[0]);
    }
}