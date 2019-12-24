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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="GyroTesting", group="Pushbot")

public class MoveForwardTest extends LinearOpMode {

    MainClass mc =         new MainClass();
    public ElapsedTime     runtime = new ElapsedTime();
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 26.9;
    static final double FINAL_DRIVE_REDUCTION = 2.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;

    // TODO fix these constants holy

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * FINAL_DRIVE_REDUCTION *
            DRIVE_GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER_INCHES);

    // cpi * pi * diameter / final * drive

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        mc.init(hardwareMap, imu, lastAngles);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Heading:", checkOrientation());
            telemetry.update();

            mc.EncoderMove(20, this);
            mc.EncoderStrafe(20, this);
            mc.rotate(90, 0.5, this);
            resetAngle();
        }
    }

    private double checkOrientation() {
        Orientation angles;

        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;

//        this.imu.getPosition();
//        double curHeading = angles.firstAngle;
//        return curHeading;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .01;

        angle = checkOrientation();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        mc.lFrontMotor.setPower(leftPower);
        mc.lBackMotor.setPower(leftPower);
        mc.rFrontMotor.setPower(rightPower);
        mc.rBackMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && checkOrientation() == 0) {}

            while (opModeIsActive() && checkOrientation() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && checkOrientation() < degrees) {}

        // turn the motors off.
        mc.lFrontMotor.setPower(0);
        mc.lBackMotor.setPower(0);
        mc.rFrontMotor.setPower(0);
        mc.rBackMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void EncoderMove(int inches) {

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = mc.lFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightFrontTarget = mc.rFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newLeftBackTarget = mc.lBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightBackTarget = mc.rBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
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

        // cpi * pi * diameter / final * drive

        while(opModeIsActive()) {
//        while(mc.lFrontMotor.getCurrentPosition()  < ((inches * Math.PI * WHEEL_DIAMETER_INCHES) / (FINAL_DRIVE_REDUCTION * DRIVE_GEAR_REDUCTION)) &&
//              mc.lBackMotor.getCurrentPosition()  < ((inches * Math.PI * WHEEL_DIAMETER_INCHES) / (FINAL_DRIVE_REDUCTION * DRIVE_GEAR_REDUCTION)) &&
//              mc.rFrontMotor.getCurrentPosition()  < ((inches * Math.PI * WHEEL_DIAMETER_INCHES) / (FINAL_DRIVE_REDUCTION * DRIVE_GEAR_REDUCTION)) &&
//              mc.rBackMotor.getCurrentPosition()  < ((inches * Math.PI * WHEEL_DIAMETER_INCHES) / (FINAL_DRIVE_REDUCTION * DRIVE_GEAR_REDUCTION))) {
            correction = checkDirection();
            mc.lFrontMotor.setPower(0.2 - correction);
            mc.lBackMotor.setPower(0.2 - correction);
            mc.rBackMotor.setPower(0.2 + correction);
            mc.rFrontMotor.setPower(0.2 + correction);
        }

        while (opModeIsActive() == true &&
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

        mc.lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mc.lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mc.rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mc.rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}