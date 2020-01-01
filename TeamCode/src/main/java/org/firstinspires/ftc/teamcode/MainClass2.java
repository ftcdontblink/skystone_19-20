package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in
 * either the autonomous or the teleop period of an FTC match. The names of OpModes appear on the
 * menu of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new
 * name. Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode
 *list
 */

public class MainClass2 extends LinearOpMode {

    // Defining Motors
    public DcMotor lFrontMotor;
    public DcMotor rFrontMotor;
    public DcMotor rBackMotor;
    public DcMotor lBackMotor;
    public Servo ServoLeft;
    public Servo ServoRight;
    public Servo ServoStone;
    public DcMotor LeftIntake;
    public DcMotor RightIntake;
    public DcMotor Pivot;
    public Servo FlipLeft;
    public Servo FlipRight;
    public DcMotor Lift;
    static final double COUNTS_PER_MOTOR_REV_LET = 28;
    static final double DRIVE_GEAR_REDUCTION_LET = 26.9;
    static final double FINAL_DRIVE_REDUCTION_LET = 2.0;
    static final double WHEEL_DIAMETER_INCHES_LET = 4.0;
    static final double LET = (COUNTS_PER_MOTOR_REV_LET * FINAL_DRIVE_REDUCTION_LET *
            DRIVE_GEAR_REDUCTION_LET) / (Math.PI * WHEEL_DIAMETER_INCHES_LET);

    //Defining Variables
    public double lFrontSpeed;
    public double lBackSpeed;
    public double rFrontSpeed;
    public double rBackSpeed;
    public final double FperS = 1.68;

    public double translateY; // -gamepad1.left_stick_y
    public double translateX; // -gamepad1.left_stick_x
    public double rotate;     // -gamepad1.right_stick_x
    public double deadzone = 0.05; // deadzone
    public int motorScale;

    public double leftstartAngle = 0.1;
    public double rightStartAngle = 0.65;
    public double leftterminalAngle = 0.6;
    public double rightterminalAngle = 0.15;
    public double stoneStartAngle = 0.35;
    public double stoneterminalAngle = 0.9;
    public int PivotArmAngle; //TODO: Add/change values for this to be accurate (Line 231
    public int PivotArmTerminalAngle; //TODO: Add.change values - may need to cast in Mecanum_Drive

    public int flip = 1;

    public int firstflipstart;
    public int firstflipterm;
    public int secondflipstart;
    public int secondflipterm;

    public int position = 0;
    public int add = 0;         //subjective to change

    //Setting Motor values
    public ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 26.9;
    static final double FINAL_DRIVE_REDUCTION = 2.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * FINAL_DRIVE_REDUCTION *
            DRIVE_GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER_INCHES);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.2;
    static final double FLIP_LEFT_DOWN_ANGLE = 0.29;
    static final double FLIP_RIGHT_DOWN_ANGLE = 0.63;
    static final double FLIP_LEFT_UP_ANGLE = 0.4;
    static final double FLIP_RIGHT_UP_ANGLE = 0.520;
    static final double FLIP_RIGHT_SPIT_ANGLE = 0.5575;
    static final double FLIP_LEFT_SPIT_ANGLE = 0.362;

    HardwareMap asn;

    int move = 0;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    public MainClass2() {

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void init(HardwareMap h) {
        HardwareMap hwMap = h;
        lFrontMotor = hwMap.get(DcMotor.class, "left_Front_Motor");
        rFrontMotor = hwMap.get(DcMotor.class, "right_Front_Motor");
        lBackMotor = hwMap.get(DcMotor.class, "left_Back_Motor");
        rBackMotor = hwMap.get(DcMotor.class, "right_Back_Motor");
        ServoLeft = hwMap.get(Servo.class, "servo_left");
        ServoRight = hwMap.get(Servo.class, "servo_right");
        ServoStone = hwMap.get(Servo.class, "servo_stone");
        FlipLeft = hwMap.get(Servo.class, "flip_left");
        FlipRight = hwMap.get(Servo.class, "flip_right"); //TODO: Uncomment
        LeftIntake = hwMap.get(DcMotor.class, "left_intake");
        RightIntake = hwMap.get(DcMotor.class, "right_intake");


        //Set left motors to reverse
        rFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rBackMotor.setDirection(DcMotor.Direction.REVERSE);

        rFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set motor power to zero.
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);

        ServoLeft.setPosition(leftstartAngle);
        ServoRight.setPosition(rightStartAngle);

        ServoStone.setPosition(stoneStartAngle);

//        FlipLeft.setPosition(FLIP_LEFT_UP_ANGLE);
//        FlipRight.setPosition(FLIP_RIGHT_UP_ANGLE);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//        imu.initialize(parameters);
//        resetAngle();
    }

    public void buildingZoneRed(boolean op) {
        EncoderMove(-5, op);
        EncoderStrafe(-8, op);
        EncoderMove(-24, op);
        sleep(1000);
        ServoLeft.setPosition(leftterminalAngle);
        ServoRight.setPosition(rightterminalAngle);
        sleep(1000);
        EncoderMove(32, op);
        sleep(500);
        ServoLeft.setPosition(leftstartAngle);
        ServoRight.setPosition(rightStartAngle);
        sleep(500);
        EncoderStrafe(20, op);
        EncoderMove(-16, op);
        EncoderStrafe(-10, op);
        EncoderStrafe(25, op);
    }


    public void buildingZoneBlue(boolean op) {
        EncoderMove(-5, op);
        EncoderStrafe(8, op);
        EncoderMove(-24, op);
        sleep(1000);
        ServoLeft.setPosition(leftterminalAngle);
        ServoRight.setPosition(rightterminalAngle);
        sleep(1000);
        EncoderMove(32, op);
        sleep(500);
        ServoLeft.setPosition(leftstartAngle);
        ServoRight.setPosition(rightStartAngle);
        sleep(500);
        EncoderStrafe(-20, op);
        EncoderMove(-16, op);
        EncoderStrafe(4, op);
        EncoderStrafe(-18, op);
    }

    public void loadingZoneRed(boolean op) {
        EncoderStrafe(-26, opModeIsActive());
        EncoderMove(36, opModeIsActive());
    }

    public void loadingZoneBlue(boolean op) {
        EncoderStrafe(-33, opModeIsActive());
        sleep(1000);
        ServoStone.setPosition(0.95);
        sleep(1000);
        EncoderStrafe(30, opModeIsActive());
        EncoderMove(-38, opModeIsActive());
        sleep(1000);
        ServoStone.setPosition(0.5);
        sleep(1000);
        EncoderMove(46, opModeIsActive());
        EncoderStrafe(-32, opModeIsActive());
        sleep(1000);
        ServoStone.setPosition(0.95);
        sleep(1000);
        EncoderStrafe(28, opModeIsActive());
        sleep(1000);
        ServoStone.setPosition(0.5);
        sleep(1000);
        EncoderMove(16, opModeIsActive());

    }

    public void SafetyZoneRed() {
    }

    public void SafetyZoneBlue() {
    }

    public void EncoderMove(int inches, boolean opMode) {
        boolean op = opMode;

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = lFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightFrontTarget = rFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newLeftBackTarget = lBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightBackTarget = rBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        lFrontMotor.setTargetPosition(newLeftFrontTarget);
        lBackMotor.setTargetPosition(newLeftBackTarget);
        rBackMotor.setTargetPosition(newRightBackTarget);
        rFrontMotor.setTargetPosition(newRightFrontTarget);

        // Turn On RUN_TO_POSITION
        lFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        //TODO Wouldnt this actually run the motors and be the motion in the program?
        runtime.reset();
        lFrontMotor.setPower(0.6);
        lBackMotor.setPower(0.6);
        rBackMotor.setPower(0.6);
        rFrontMotor.setPower(0.6);

        while (op == true &&
                (runtime.seconds() < 30) &&
                (lFrontMotor.isBusy() && lBackMotor.isBusy() || rFrontMotor.isBusy() &&
                        rBackMotor.isBusy())) {
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
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void EncoderStrafe(int inches, boolean op) {

        int newLeftFrontTarget, newLeftBackTarget;
        int newRightFrontTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = lFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        newRightFrontTarget = rFrontMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        newLeftBackTarget = lBackMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
        newRightBackTarget = rBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        lFrontMotor.setTargetPosition(newLeftFrontTarget);
        lBackMotor.setTargetPosition(newLeftBackTarget);
        rBackMotor.setTargetPosition(newRightBackTarget);
        rFrontMotor.setTargetPosition(newRightFrontTarget);

        // Turn On RUN_TO_POSITION
        lFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        //TODO Wouldnt this actually run the motors and be the motion in the program?
        runtime.reset();

        lFrontMotor.setPower(0.5);
        lBackMotor.setPower(0.5);
        rBackMotor.setPower(0.5);
        rFrontMotor.setPower(0.5);

        while (op == true &&
                (runtime.seconds() < 30) &&
                (lFrontMotor.isBusy() && lBackMotor.isBusy() && rFrontMotor.isBusy() &&
                        rBackMotor.isBusy())) {
            //TODO The isBusy check is at the beggining of the while opModeIsActive
            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,
//                                                              newRightFrontTarget);
//            telemetry.addData("Path2",  "Running at %7d :%7d:%7d :%7d",
//                    lFrontMotor.getCurrentPosition(),
//                    lBackMotor.getCurrentPosition(),
//                    rBackMotor.getCurrentPosition(),
//                    rFrontMotor.getCurrentPosition());
//            telemetry.update();
        }

        // Stop all motion;
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }






    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 0.1;

        angle = getAngle();

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

    public void rotate(int degrees, double power, boolean op)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetAngle();

        /***
         * Very important for the MoveForwardTest - Try removing the reset angle towards the start
         * here, so that it takes the initial angle as 0 but every rotate call after that has to be
         * in relation to what the robot is already at.
         */


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
        lFrontMotor.setPower(leftPower);
        lBackMotor.setPower(leftPower);
        rFrontMotor.setPower(rightPower);
        rBackMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (op == true && getAngle() == 0) {}

            while (op == true && getAngle() > degrees) {}
        }
        else    // left turn.
            while (op == true && getAngle() < degrees) {}

        // turn the motors off.
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}