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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="GyroTesting", group="Pushbot")

public class MoveForwardTest extends LinearOpMode {

    MainClass mc =         new MainClass();
    public ElapsedTime     runtime = new ElapsedTime();
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        mc.init(hardwareMap, imu, lastAngles);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
//            correction = checkDirection();
//
//            mc.lFrontMotor.setPower(0.2 - correction);
//            mc.rFrontMotor.setPower(0.2 + correction);
//            mc.lBackMotor.setPower(0.2 - correction);
//            mc.rBackMotor.setPower(0.2 + correction);

            mc.correction = mc.checkDirection();

            mc.lFrontMotor.setPower(0.2 - mc.correction);
            mc.rFrontMotor.setPower(-0.2 + mc.correction);
            mc.lBackMotor.setPower(-0.2 - mc.correction);
            mc.rBackMotor.setPower(0.2 + mc.correction);

        }


    }

//    private void resetAngle()
//    {
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        globalAngle = 0;
//    }
//
//    /**
//     * Get current cumulative angle rotation from last reset.
//     * @return Angle in degrees. + = left, - = right.
//     */
//    public double getAngle()
//    {
//        // We experimentally determined the Z axis is the axis we want to use for heading angle.
//        // We have to process the angle because the imu works in euler angles so the Z axis is
//        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }
//
//    /**
//     * See if we are moving in a straight line and if not return a power correction value.
//     * @return Power adjustment, + is adjust left - is adjust right.
//     */
//    public double checkDirection()
//    {
//        // The gain value determines how sensitive the correction is to direction changes.
//        // You will have to experiment with your robot to get small smooth direction changes
//        // to stay on a straight line.
//        double correction, angle, gain = 0.1;
//
//        angle = getAngle();
//
//        if (angle == 0)
//            correction = 0;             // no adjustment.
//        else
//            correction = -angle;        // reverse sign of angle for correction.
//
//        correction = correction * gain;
//
//        return correction;
//    }
//
//    /**
//     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
//     * @param degrees Degrees to turn, + is left - is right
//     */
//
//    public void rotate(int degrees, double power, boolean op)
//    {
//        double  leftPower, rightPower;
//
//        // restart imu movement tracking.
//
//        resetAngle();
//
//        /***
//         * Very important for the MoveForwardTest - Try removing the reset angle towards the start
//         * here, so that it takes the initial angle as 0 but every rotate call after that has to be
//         * in relation to what the robot is already at.
//         */
//
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        if (degrees < 0)
//        {   // turn right.
//            leftPower = power;
//            rightPower = -power;
//        }
//        else if (degrees > 0)
//        {   // turn left.
//            leftPower = -power;
//            rightPower = power;
//        }
//        else return;
//
//        // set power to rotate.
//        mc.lFrontMotor.setPower(leftPower);
//        mc.lBackMotor.setPower(leftPower);
//        mc.rFrontMotor.setPower(rightPower);
//        mc.rBackMotor.setPower(rightPower);
//
//        // rotate until turn is completed.
//        if (degrees < 0)
//        {
//            // On right turn we have to get off zero first.
//            while (op == true && getAngle() == 0) {}
//
//            while (op == true && getAngle() > degrees) {}
//        }
//        else    // left turn.
//            while (op == true && getAngle() < degrees) {}
//
//        // turn the motors off.
//        mc.lFrontMotor.setPower(0);
//        mc.lBackMotor.setPower(0);
//        mc.rFrontMotor.setPower(0);
//        mc.rBackMotor.setPower(0);
//
//        // wait for rotation to stop.
//        sleep(1000);
//
//        // reset angle tracking on new heading.
//        resetAngle();
//    }
}