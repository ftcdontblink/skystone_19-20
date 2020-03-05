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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

public class Robot {


    // wheel motors

    public final double WHEEL_CIRCUMFERENCE = Math.PI * 4; // wheel circumference
    public final double WHEEL_TICKS = 28 * 27.4; // ticks with reduction
    public final double COUNTS_PER_INCH = WHEEL_TICKS / WHEEL_CIRCUMFERENCE; // ticks per rev

    public final double LIFT_CIRCUMFERENCE = Math.PI * 4; // lift winch circumference
    public final double LIFT_TICKS = 288; // lift with gear reduction
    public final double LIFT_COUNTS_PER_INCH = LIFT_TICKS / LIFT_CIRCUMFERENCE; // lift ticks per rev

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    // intake motors and intake servos
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public Servo leftIntakeServo;
    public Servo rightIntakeServo;

    // foundation hooks and autonomous clamps
    public Servo leftFoundation;
    public Servo rightFoundation;
    public Servo autonHook;
    public Servo autonClamp;

    // lift mechanism
    public DcMotor leftLift;
    public DcMotor rightLift;
    public Servo liftClamp;
    public CRServo liftExtension;
    public Servo kicker;

//    Drive drive;
//    FoundationHooks foundationHooks;
//    Intake intake;
//    Lift lift;
//    Claw claw;

    // imu
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle, correction;

    public Robot(HardwareMap hwMap, Telemetry telemetry, Orientation o, BNO055IMU i) { // get hardware map from that program
        lastAngles = o;
        imu = i;

        leftFront = hwMap.get(DcMotor.class, "lfm");
        rightFront = hwMap.get(DcMotor.class, "rfm");
        leftBack = hwMap.get(DcMotor.class, "lbm");
        rightBack = hwMap.get(DcMotor.class, "rbm");

        leftLift = hwMap.get(DcMotor.class, "ll");
        rightLift = hwMap.get(DcMotor.class, "rl");

        leftIntake = hwMap.get(DcMotor.class, "ilm");
        rightIntake = hwMap.get(DcMotor.class, "irm");
        leftIntakeServo = hwMap.get(Servo.class, "ils");
        rightIntakeServo = hwMap.get(Servo.class, "irs");

        leftFoundation = hwMap.get(Servo.class, "lf");
        rightFoundation = hwMap.get(Servo.class, "rf");

        autonHook = hwMap.get(Servo.class, "auto");
        autonClamp = hwMap.get(Servo.class, "aclamp");
//
        liftClamp = hwMap.get(Servo.class, "clamp");
        liftExtension = hwMap.get(CRServo.class, "extension");
        kicker = hwMap.get(Servo.class, "kicker");
//
//        imu = hwMap.get(BNO055IMU.class, "imu");

//      -----------------------------------------------------------------------------------------

        // set motor directions

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // stop and reset the encoders

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the mode to brake; no power = stop immediately

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set lift direction and stop and reset lift motor encoders

        rightLift.setDirection(DcMotor.Direction.REVERSE);
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set mode to brake

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

//
//

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.initialize(parameters);

        // run using encoders

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        drive = new Drive(leftFront, rightFront, leftBack, rightBack);
//        foundationHooks = new FoundationHooks(leftFoundation, rightFoundation);
//        intake = new Intake(leftIntakeServo, rightIntakeServo, leftIntake, rightIntake);
//        lift = new Lift(leftLift, rightLift, liftClamp, liftExtension, kicker);
//        claw = new Claw(autonHook, autonClamp);

        telemetry.addData("Initialized: ", "YAY");
        telemetry.update();

        resetAngle();
    }

    public double checkOrientation() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .04;

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
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power, LinearOpMode op, DcMotor[] motors) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        motors[0].setPower(leftPower);
        motors[2].setPower(leftPower);
        motors[1].setPower(rightPower);
        motors[3].setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (op.opModeIsActive() == true && checkOrientation() == 0) {
            }

            while (op.opModeIsActive() == true && checkOrientation() > degrees) {
            }
        } else    // left turn.
            while (op.opModeIsActive() == true && checkOrientation() < degrees) {
            }

        // turn the motors off.
        motors[0].setPower(0);
        motors[2].setPower(0);
        motors[1].setPower(0);
        motors[3].setPower(0);
        // wait for rotation to stop.
        op.sleep(300);

        // reset angle tracking on new heading.
        resetAngle();
    }

//    public void run(Gamepad gamepad1, Gamepad gamepad2, LinearOpMode op) {
//        drive.drive(gamepad1, op);
//        lift.control(gamepad2, op);
//        foundationHooks.control(gamepad2, op);
//        claw.control(gamepad1, op);
//        intake.control(gamepad2, op);
//    }


    public void liftMove(double inches, LinearOpMode op) {
        double tp = inches * LIFT_COUNTS_PER_INCH;

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        leftLift.setPower(0);
        rightLift.setPower(0);
    }

    public void EncoderStrafe(double inches, double power, LinearOpMode op) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double tp = inches * COUNTS_PER_INCH;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;

        while(op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(leftFront.getCurrentPosition())
                    + Math.abs(leftBack.getCurrentPosition())
                    + Math.abs(rightFront.getCurrentPosition())
                    + Math.abs(rightBack.getCurrentPosition()))/4.0;
            if(inches > 0) {
                leftFront.setPower(power - correction);
                rightFront.setPower(-power + correction);
                leftBack.setPower(-power - correction);
                rightBack.setPower(power + correction);
            } else {
                leftFront.setPower(-power - correction);
                rightFront.setPower(power + correction);
                leftBack.setPower(power - correction);
                rightBack.setPower(-power + correction);
            }

            if(Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
