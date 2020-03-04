package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.BNO055IMUImpl;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

        import java.util.Base64;


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

public class MainClass extends LinearOpMode {


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

        // wheel motors


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

        public final double WHEEL_CIRCUMFERENCE = Math.PI * 4; // wheel circumference
        public final double WHEEL_TICKS = 28 * 27.4; // ticks with reduction
        public final double COUNTS_PER_INCH = WHEEL_TICKS / WHEEL_CIRCUMFERENCE; // ticks per rev

        public final double LIFT_CIRCUMFERENCE = Math.PI * 4; // lift winch circumference
        public final double LIFT_TICKS = 288; // lift with gear reduction
        public final double LIFT_COUNTS_PER_INCH = LIFT_TICKS / LIFT_CIRCUMFERENCE; // lift ticks per rev

        public MainClass() {}

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void init(HardwareMap hwMap, BNO055IMU i, Orientation o) { // get hardware map from that program
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

            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

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

            rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set mode to brake

            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftIntake.setDirection(DcMotor.Direction.REVERSE);
            rightIntake.setDirection(DcMotor.Direction.REVERSE);


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);

            globalAngle = 0;
            // run using encoders

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            resetAngle();
    }

    public void foundationup(LinearOpMode op) {
        if(op.opModeIsActive()) {
            leftFoundation.setPosition(0.429); //up posiiton
            rightFoundation.setPosition(0.6);
        }
    }

    public void foundationdown(LinearOpMode op){
        if(op.opModeIsActive()) {
            leftFoundation.setPosition(0.32); //downpos
            rightFoundation.setPosition(0.491);
        }
    }


    public void EncoderMove(double inches, LinearOpMode op) {

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
                leftFront.setPower(0.6 - correction);
                rightFront.setPower(0.6 + correction);
                leftBack.setPower(0.6 - correction);
                rightBack.setPower(0.6 + correction);
            } else {
                leftFront.setPower(-0.6 - correction);
                rightFront.setPower(-0.6 + correction);
                leftBack.setPower(-0.6 - correction);
                rightBack.setPower(-0.6 + correction);
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

    public void EncoderMove(double inches, double power, LinearOpMode op) {
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
                rightFront.setPower(power + correction);
                leftBack.setPower(power - correction);
                rightBack.setPower(power + correction);
            } else {
                leftFront.setPower(-power - correction);
                rightFront.setPower(-power + correction);
                leftBack.setPower(-power - correction);
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

    public void liftMove(double inches, LinearOpMode op) {
        double tp = inches * LIFT_COUNTS_PER_INCH;

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;

        while(op.opModeIsActive()) {
            avg = (Math.abs(leftLift.getCurrentPosition()) + Math.abs(rightLift.getCurrentPosition())) / 2.0;
            if(inches > 0) {
                leftLift.setPower(-1);
                rightLift.setPower(1);
            } else {
                leftLift.setPower(1);
                rightLift.setPower(-1);
            }

            if(Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        leftLift.setPower(0);
        rightLift.setPower(0);
    }

    public void EncoderStrafe(double inches, double power, LinearOpMode op) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double tp = inches * COUNTS_PER_INCH;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;

        while (op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition())) / 4.0;
            if (inches > 0) {
                leftFront.setPower(0.6 - correction);
                rightFront.setPower(-0.6 + correction);
                leftBack.setPower(-0.6 - correction);
                rightBack.setPower(0.6 + correction);
            } else {
                leftFront.setPower(-0.6 - correction);
                rightFront.setPower(0.6 + correction);
                leftBack.setPower(0.6 - correction);
                rightBack.setPower(-0.6 + correction);
            }

            if (Math.abs(avg) > Math.abs(tp)) {
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

    public void EncoderStrafe(double inches, LinearOpMode op) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double tp = inches * COUNTS_PER_INCH;

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;

        while (op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition())) / 4.0;
            if (inches > 0) {
                leftFront.setPower(0.6 - correction);
                rightFront.setPower(-0.6 + correction);
                leftBack.setPower(-0.6 - correction);
                rightBack.setPower(0.6 + correction);
            } else {
                leftFront.setPower(-0.6 - correction);
                rightFront.setPower(0.6 + correction);
                leftBack.setPower(0.6 - correction);
                rightBack.setPower(-0.6 + correction);
            }

            if (Math.abs(avg) > Math.abs(tp)) {
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

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
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
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power, LinearOpMode op)
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

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (op.opModeIsActive() == true && checkOrientation() == 0) {}

            while (op.opModeIsActive() == true && checkOrientation() > degrees) {}
        }
        else    // left turn.
            while (op.opModeIsActive() == true && checkOrientation() < degrees) {}

        // turn the motors off.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // wait for rotation to stop.
        sleep(300);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rotateBack(double degrees, double power, LinearOpMode op)
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

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
//        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
//        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (op.opModeIsActive() == true && checkOrientation() == 0) {}

            while (op.opModeIsActive() == true && checkOrientation() > degrees) {}
        }
        else    // left turn.
            while (op.opModeIsActive() == true && checkOrientation() < degrees) {}

        // turn the motors off.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // wait for rotation to stop.
        sleep(300);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rotateFront(double degrees, double power, LinearOpMode op)
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

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        leftFront.setPower(leftPower);
//        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
//        rightBack.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (op.opModeIsActive() == true && checkOrientation() == 0) {}

            while (op.opModeIsActive() == true && checkOrientation() > degrees) {}
        }
        else    // left turn.
            while (op.opModeIsActive() == true && checkOrientation() < degrees) {}

        // turn the motors off.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // wait for rotation to stop.
        sleep(300);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rotateLeft(double degrees, double power, LinearOpMode op)
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

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
//        rightFront.setPower(rightPower);
//        rightBack.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (op.opModeIsActive() == true && checkOrientation() == 0) {}

            while (op.opModeIsActive() == true && checkOrientation() > degrees) {}
        }
        else    // left turn.
            while (op.opModeIsActive() == true && checkOrientation() < degrees) {}

        // turn the motors off.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // wait for rotation to stop.
        sleep(300);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rotateRight(double degrees, double power, LinearOpMode op)
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

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
//        leftFront.setPower(leftPower);
//        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (op.opModeIsActive() == true && checkOrientation() == 0) {}

            while (op.opModeIsActive() == true && checkOrientation() > degrees) {}
        }
        else    // left turn.
            while (op.opModeIsActive() == true && checkOrientation() < degrees) {}

        // turn the motors off.
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // wait for rotation to stop.
        sleep(300);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void EncoderStrafeVel(double inches, LinearOpMode op) {
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
        double percentCP;

        while(op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition()))/4.0;
            percentCP = Math.abs(avg)/Math.abs(tp);
            if(inches > 0) {
                if(percentCP < 0.1) {
                    leftFront.setPower((percentCP*10 - 0.4) - correction);
                    rightFront.setPower((-percentCP*10 + 0.4) + correction);
                    leftBack.setPower((-percentCP*10 + 0.4) - correction);
                    rightBack.setPower((percentCP*10 - 0.4) + correction);
                } else if(percentCP > 0.1 && percentCP < 0.9) {
                    leftFront.setPower(-1 - correction);
                    rightFront.setPower(1 + correction);
                    leftBack.setPower(1 - correction);
                    rightBack.setPower(-1 + correction);
                } else {
                    leftFront.setPower(((1-percentCP)*10) - correction - 0.4);
                    rightFront.setPower((-(1-percentCP)*10) + correction + 0.4);
                    leftBack.setPower((-(1-percentCP)*10) - correction + 0.4);
                    rightBack.setPower(((1-percentCP)*10) + correction - 0.4);
                }
            } else {
                if (percentCP < 0.1) {
                    leftFront.setPower((-percentCP * 10 - 0.4) - correction);
                    rightFront.setPower((percentCP * 10 + 0.4) + correction);
                    leftBack.setPower((percentCP * 10 + 0.4) - correction);
                    rightBack.setPower((-percentCP * 10 - 0.4) + correction);
                } else if (percentCP > 0.1 && percentCP < 0.9) {
                    leftFront.setPower(-1 - correction);
                    rightFront.setPower(1 + correction);
                    leftBack.setPower(1 - correction);
                    rightBack.setPower(-1 + correction);
                } else {
                    leftFront.setPower((-(1 - percentCP) * 10) - correction - 0.4);
                    rightFront.setPower(((1 - percentCP) * 10) + correction + 0.4);
                    leftBack.setPower(((1 - percentCP) * 10) - correction + 0.4);
                    rightBack.setPower((-(1 - percentCP) * 10) + correction - 0.4);
                }
            }
            if(Math.abs(avg) > Math.abs(tp)) {
                break;
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

    public void EncoderMoveVel(double inches, LinearOpMode op) {
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
        double percentCP;

        while(op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition()))/4.0;
            percentCP = avg/tp;
            if(inches > 0) {
                if(percentCP < 0.1) {
                    leftFront.setPower((percentCP*10 + 0.4) - correction);
                    rightFront.setPower((percentCP*10 + 0.4) + correction);
                    leftBack.setPower((percentCP*10 + 0.4) - correction);
                    rightBack.setPower((percentCP*10 + 0.4) + correction);
                } else if(percentCP > 0.1 && percentCP < 0.9) {
                    leftFront.setPower(1 - correction);
                    rightFront.setPower(1 + correction);
                    leftBack.setPower(1 - correction);
                    rightBack.setPower(1 + correction);
                } else {
                    leftFront.setPower(((1-percentCP)*10) - correction + 0.4);
                    rightFront.setPower(((1-percentCP)*10) + correction + 0.4);
                    leftBack.setPower(((1-percentCP)*10) - correction + 0.4);
                    rightBack.setPower(((1-percentCP)*10) + correction + 0.4);
                }
            } else {
                if (percentCP < 0.1) {
                    leftFront.setPower((-percentCP * 10 - 0.4) - correction);
                    rightFront.setPower((-percentCP * 10 - 0.4) + correction);
                    leftBack.setPower((-percentCP * 10 - 0.4) - correction);
                    rightBack.setPower((-percentCP * 10 - 0.4) + correction);
                } else if (percentCP > 0.1 && percentCP < 0.9) {
                    leftFront.setPower(-1 - correction);
                    rightFront.setPower(-1 + correction);
                    leftBack.setPower(-1 - correction);
                    rightBack.setPower(-1 + correction);
                } else {
                    leftFront.setPower((-(1 - percentCP) * 10) - correction - 0.4);
                    rightFront.setPower((-(1 - percentCP) * 10) + correction - 0.4);
                    leftBack.setPower((-(1 - percentCP) * 10) - correction - 0.4);
                    rightBack.setPower((-(1 - percentCP) * 10) + correction - 0.4);
                }
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

    public void plough(LinearOpMode op) {
        if(op.opModeIsActive()) {
            autonClamp.setPosition(0.35);
            autonHook.setPosition(0.27);
        }
    }

    public void release(LinearOpMode op) {
        if(op.opModeIsActive()) {
            EncoderStrafe(-7.5, this);
            autonClamp.setPosition(0.35);
            autonHook.setPosition(0.27);
            EncoderStrafe(7.5, this);
        }
    }

    public void release(LinearOpMode op, boolean no) {
        if(op.opModeIsActive()) {
            autonClamp.setPosition(0.35);
            autonHook.setPosition(0.27);
            EncoderStrafe(7.5, this);
        }
    }

    public void clamp(LinearOpMode op) {
        if(op.opModeIsActive()) {
            autonHook.setPosition(0.33);
            autonClamp.setPosition(0.75);
            sleep(700);
        }
    }

    public void raise(LinearOpMode op) {
        if(op.opModeIsActive()) {
            autonHook.setPosition(0);
            autonClamp.setPosition(0.75);
        }
    }

    public void quarry1(LinearOpMode op) {
        plough(op);
        EncoderStrafe(-30, this);
    }

    public void quarry2(LinearOpMode op) {
        plough(op);
        EncoderStrafe(-7.5, this);
        clamp(op);
        raise(op);
        EncoderStrafe(7.5, this);
    }

    public void park(LinearOpMode op) {
        if(op.opModeIsActive()) {
            EncoderStrafe(30, this);
            rotate(-90, 1, this);
            rightIntakeServo.setPosition(0.27);
            leftIntakeServo.setPosition(0.64);
        }
    }

    public void intakeOn(LinearOpMode op) {
        if(op.opModeIsActive()) {
            leftIntake.setPower(-1);
            rightIntake.setPower(1);
        }
    }

    public void intakeOff(LinearOpMode op) {
        if(op.opModeIsActive()) {
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        }
    }
}
