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
        public final double LIFT_MOTOR_TICKS = LIFT_TICKS / LIFT_CIRCUMFERENCE; // lift ticks per rev

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


//    public void buildingZoneBlueIn(LinearOpMode op, Telemetry t) {
//        EncoderMove(-5, op);
//        EncoderStrafe(8, op);
//        EncoderMove(-12.5, op);
//        sleep(500);
//        leftFoundation.setPosition(1);
//        rightFoundation.setPosition(0);
//        sleep(500);
//        EncoderMove(18, op);
//        rotate(100,0.6, op);
//        leftFoundation.setPosition(0);
//        rightFoundation.setPosition(1);
//        sleep(500);
//        EncoderMove(-8, op);
//        EncoderStrafe(-14, op);
//        sleep(13000);
//        EncoderMove(29, op);
//    }
//
//    public void buildingZoneBlueOut(LinearOpMode op, Telemetry t) {
//        EncoderMove(-5, op);
//        EncoderStrafe(8, op);
//        EncoderMove(-12.5, op);
//        sleep(500);
//        ServoLeft.setPosition(1);
//        ServoRight.setPosition(0);
//        sleep(500);
//        EncoderMove(18, op);
//        rotate(100,0.6, op);
//        ServoLeft.setPosition(0);
//        ServoRight.setPosition(1);
//        sleep(500);
//        EncoderMove(-8, op);
//        EncoderStrafe(10, op);
//        sleep(13000);
//        EncoderMove(29, op);
//    }
//
//    public void buildingZoneRedIn(LinearOpMode op, Telemetry t) {
//        EncoderMove(-5, op);
//        EncoderStrafe(-8, op);
//        EncoderMove(-12, op);
//        sleep(500);
//        ServoLeft.setPosition(1);
//        ServoRight.setPosition(0);
//        sleep(500);
//        EncoderMove(18, op);
//        rotate(-90,1, op);
//        ServoLeft.setPosition(0);
//        ServoRight.setPosition(1);
//        sleep(500);
//        EncoderMove(-8, op);
//        EncoderStrafe(6, op);
//        sleep(13000);
//        EncoderMove(29, op);
//    }
//
//    public void buildingZoneRedOut(LinearOpMode op, Telemetry t) {
//        EncoderMove(-5, op);
//        EncoderStrafe(-8, op);
//        EncoderMove(-12, op);
//        sleep(500);
//        ServoLeft.setPosition(1);
//        ServoRight.setPosition(0);
//        sleep(500);
//        EncoderMove(18, op);
//        rotate(-90,1, op);
//        ServoLeft.setPosition(0);
//        ServoRight.setPosition(1);
//        sleep(500);
//        EncoderMove(-8, op);
//        EncoderStrafe(-12, op);
//        sleep(13000);
//        EncoderMove(29, op);
//    }
//
//    public void bluepos1Super(LinearOpMode op) {
//        EncoderStrafe(-24.25, op);
//        pickUpSeq(op);
//        EncoderStrafe(5, op);
//        EncoderMove(-46, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove(67.25, 1, op);
//        sleep(150);
//        EncoderStrafe(-9, op);
//        pickUpSeq(op);
//        EncoderStrafe(5.5, op);
//        EncoderMove(-73.5, 1, op);
//        sleep(150);
//        rotate(-90, 1, op);
//        EncoderMove(-5, op);
//        ServoLeft.setPosition(1);
//        ServoRight.setPosition(0);
//        EncoderMove(15, 1, op);
//        rotate(90, 1, op);
//        ServoLeft.setPosition(0);
//        ServoRight.setPosition(1);
//        rotate(90, 1, op);
//        releaseSeqLess(op);
//        EncoderMove(15, 1, op);
//        EncoderStrafe(35, op);
//        sleep(200);
//
//        ServoLeft.setPosition(0);
//        ServoRight.setPosition(1);
//        EncoderStrafe(-12, op);
//        EncoderMove(-20, op);
//        EncoderStrafe(-15, op);
//        sleep(60000);
//    }
//
//    public void bluepos2Super(LinearOpMode op) {
//        EncoderStrafe(-24, op);
//        EncoderMove(5.5, op);
//        pickUpSeq(op);
//        EncoderStrafe(3, op);
//        EncoderMove(-50, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove((65.5+4), 1, op);
//        sleep(150);
//        EncoderStrafe(-5.5, op);
//        pickUpSeq(op);
//        EncoderStrafe(5, op);
//        EncoderMove(-(73.5+4), 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        rotate(90, 1, op);
//        EncoderMove(-5, op);
//        ServoLeft.setPosition(1);
//        ServoRight.setPosition(0);
//        EncoderMove(30, 1, op);
//        ServoLeft.setPosition(0);
//        ServoRight.setPosition(1);
//        EncoderStrafe(-50, op);
//        sleep(60000);
//    }
//
//    public void bluepos3Super(LinearOpMode op) {
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        sleep(200);
//        EncoderStrafe(-24, op);
//        EncoderMove(10, op);
//        pickUpSeq(op);
//        EncoderStrafe(3, op);
//        EncoderMove(-56, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove((64), 1, op);
//        sleep(150);
//        EncoderStrafe(-17, op);
//        LeftIntake.setPower(0.8);
//        RightIntake.setPower(-0.8);
//        sleep(100);
//        EncoderMove(9, 1, op);
//        LeftIntake.setPower(0);
//        RightIntake.setPower(0);
//        EncoderMove(-5, 1, op);
//        FlipRight.setPosition(0.5575);
//        FlipLeft.setPosition(0.362);
//        EncoderStrafe(17, op);
//        EncoderMove(-70, 1, op);
//        rotate(90, 1, op);
//        EncoderMove(5, op);
//        sleep(400);
//        LeftIntake.setPower(-1);
//        RightIntake.setPower(1);
//        sleep(400);
//        rotate(180, 1, op);
//        EncoderMove(-10, op);
//        ServoLeft.setPosition(1);
//        ServoRight.setPosition(0);
//        EncoderMove(35, 1, op);
//        ServoLeft.setPosition(0);
//        ServoRight.setPosition(1);
//        EncoderStrafe(-50, op);
//        sleep(60000);
//    }
//
//    public void bluepos1(LinearOpMode op) {
//        EncoderStrafe(-23.5, op);
//        EncoderMove(-1, op);
//        pickUpSeq(op);
//        EncoderStrafe(8, op);
//        EncoderMove(-46,1, op);
//        EncoderStrafe(-3, op);
//        sleep(150);
//        releaseSeqLess(op);
//        sleep(100);
//        EncoderStrafe(-3, op);
//        EncoderMove(65, 1, op);
//        sleep(150);
//        EncoderStrafe(-4.5, op);
//        pickUpSeq(op);
//        EncoderStrafe(9, op);
//        EncoderMove(-73.5, 1, op);
//        EncoderStrafe(-4, op);
//        sleep(150);
//        releaseSeqLess(op);
//        sleep(100);
//        EncoderStrafe(-3, op);
//        EncoderMove(35, 1, op);
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        sleep(60000);
////        park(op);
//    }
//
//    public void bluepos2(LinearOpMode op) {
//        EncoderStrafe(-23.5, op);
//        EncoderMove(4.25, op);
//        pickUpSeq(op);
//        EncoderStrafe(8, op);
//        EncoderMove(-50, 1, op);
//        sleep(150);
//        EncoderStrafe(-9, op);
//        releaseSeqLess(op);
//        EncoderMove((65.5+4), 1, op);
//        sleep(150);
//        EncoderStrafe(-4.25, op);
//        pickUpSeq(op);
//        EncoderStrafe(5, op);
//        EncoderMove(-74.5, 1, op);
//        sleep(150);
//        EncoderStrafe(-3, op);
//        releaseSeqLess(op);
//        EncoderStrafe(4, op);
//        EncoderMove((30),  1, op);
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        sleep(60000);
////        park(op);
//    }
//
//    public void bluepos3(LinearOpMode op) {
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        sleep(200);
//        EncoderStrafe(-23.5, op);
//        EncoderMove(10, op);
//        pickUpSeq(op);
//        EncoderStrafe(8, op);
//        EncoderMove(-56, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderStrafe(-1, op);
//        EncoderMove((58), 1,  op);
//        sleep(150);
//        EncoderStrafe(-19, op);
//        LeftIntake.setPower(0.8);
//        RightIntake.setPower(-0.8);
//        sleep(100);
//        EncoderMove(4, op);
//        LeftIntake.setPower(0);
//        RightIntake.setPower(0);
//        EncoderMove(-5,  op);
//        FlipRight.setPosition(0.5575);
//        FlipLeft.setPosition(0.362);
//        EncoderStrafe(17, op);
//        EncoderMove(-70,  1, op);
//        rotate(90, 1, op);
//        EncoderMove(3, op);
//        sleep(400);
//        LeftIntake.setPower(-1);
//        RightIntake.setPower(1);
//        sleep(400);
//        rotate(-90, 1, op);
//        EncoderMove(35, 1, op);
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        sleep(60000);
////        park(op);
//    }
//
//    public void bluepos1NP(LinearOpMode op){
//        EncoderStrafe(-24, op);
//        pickUpSeq(op);
//        EncoderStrafe(3, op);
//        EncoderMove(-46+safety, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove(65.5-safety, 1, op);
//        sleep(150);
//        EncoderStrafe(-5.15, op);
//        pickUpSeq(op);
//        EncoderStrafe(5, op);
//        EncoderMove(-73.5+safety, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove(35-safety, 1, op);
//        sleep(60000);
//        park(op);
//    }
//    public void bluepos2NP(LinearOpMode op) {
//        EncoderStrafe(-24, op);
//        EncoderMove(5.5, op);
//        pickUpSeq(op);
//        EncoderStrafe(3, op);
//        EncoderMove(-50+safety, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove((65.5+4)-safety, 1, op);
//        sleep(150);
//        EncoderStrafe(-5.5, op);
//        pickUpSeq(op);
//        EncoderStrafe(5, op);
//        EncoderMove(-(73.5+4)+safety, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove((35)-safety, 1, op);
//        sleep(60000);
//        park(op);
//    }
//    public void bluepos3NP(LinearOpMode op){
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        sleep(200);
//        EncoderStrafe(-24, op);
//        EncoderMove(10, op);
//        pickUpSeq(op);
//        EncoderStrafe(3, op);
//        EncoderMove(-56+safety, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove((64-safety), 1, op);
//        sleep(150);
//        EncoderStrafe(-17, op);
//        LeftIntake.setPower(0.8);
//        RightIntake.setPower(-0.8);
//        sleep(100);
//        EncoderMove(9, 1, op);
//        LeftIntake.setPower(0);
//        RightIntake.setPower(0);
//        EncoderMove(-5, 1, op);
//        FlipRight.setPosition(0.5575);
//        FlipLeft.setPosition(0.362);
//        EncoderStrafe(17, op);
//        EncoderMove(-70+safety, 1, op);
//        rotate(180, 1, op);
//        sleep(400);
//        LeftIntake.setPower(-1);
//        RightIntake.setPower(1);
//        sleep(400);
//        EncoderMove(-25, op);
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        sleep(60000);
//
//    }
//
//
//    public void redpos1(LinearOpMode op) {
//        EncoderStrafe(-22.5, op);
//        EncoderMove(10, op);
//        pickUpSeq(op);
//        EncoderStrafe(8, op);
//        EncoderMove(49, 1, op);
//        sleep(150);
//        EncoderStrafe(-3, op);
//        releaseSeqLess(op);
//        EncoderMove(-(65.5), 1, op);
//        sleep(150);
//        EncoderStrafe(-6.5, op);
//        pickUpSeq(op);
//        EncoderStrafe(8, op);
//        EncoderMove((74), 1, op);
//        sleep(150);
//        EncoderStrafe(-4, op);
//        releaseSeqLess(op);
//        EncoderMove(-(39), 1, op);
//        sleep(60000);
//    }
//
//    public void redpos2(LinearOpMode op) {
//        EncoderStrafe(-22.5, op);
//        EncoderMove(5.75, op);
//        pickUpSeq(op);
//        EncoderStrafe(8, op);
//        EncoderMove(52.5, 1, op);
//        sleep(150);
//        EncoderStrafe(-3, op);
//        releaseSeqLess(op);
//        sleep(100);
//        EncoderStrafe(1, op);
//        EncoderMove(-(67), 1, op);
//        sleep(150);
//        EncoderStrafe(-7.5, op);
//        pickUpSeq(op);
//        EncoderStrafe(5.5, op);
//        EncoderMove((73.5+5), 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        EncoderMove(-(35), 1, op);
//        sleep(60000);
//    }
//
//    public void redpos3(LinearOpMode op) {
//        FlipRight.setPosition(0.62); //Changing position of intake servos (in order to exit the sizing grid limit)
//        FlipLeft.setPosition(0.29);
//        sleep(200); //Wait for .02 seconds before
//        EncoderStrafe(-22.5, op);
//        EncoderMove(-0.5, op);
//        pickUpSeq(op);
//        EncoderStrafe(6, op);
//        EncoderMove(60.5, 1, op);
//        sleep(150);
//        releaseSeqLess(op);
//        sleep(100);
//        EncoderStrafe(1, op);
//        EncoderMove(-66, 1, op);
//        rotate(178, 1, op);
//        sleep(150);
//        EncoderStrafe(19, op);
//        LeftIntake.setPower(0.8);
//        RightIntake.setPower(-0.8);
//        sleep(100);
//        EncoderMove(5, op);
//        LeftIntake.setPower(0);
//        RightIntake.setPower(0);
//        EncoderMove(-5,  op);
//        FlipRight.setPosition(0.5575);
//        FlipLeft.setPosition(0.362);
//        EncoderStrafe(-18, op);
//        EncoderMove(-78,  1, op);
//        rotate(-90, 1, op);
//        EncoderMove(5, op);
//        sleep(200);
//        LeftIntake.setPower(-1);
//        RightIntake.setPower(1);
//        sleep(200);
//        rotate(90, 1, op);
//        EncoderMove(32.5, 1, op);
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        sleep(60000);
//    }
//
//    public void pickUpSeq(LinearOpMode op) {
//        ServoStone.setPosition(stoneterminalAngle);
//        sleep(800);
//        aClamp.setPosition(aCin);
//        sleep(800);
//        ServoStone.setPosition(stoneStartAngle);
//    }
//
//    public void park(LinearOpMode op){
//        ServoStone.setPosition(stoneterminalAngle);
//        FlipRight.setPosition(0.62);
//        FlipLeft.setPosition(0.29);
//        ServoLeft.setPosition(1);
//        ServoRight.setPosition(0);
//
//    }
//
//    public void releaseSeqLess(LinearOpMode op) {
//        EncoderStrafe(-9.5, op);
//        ServoStone.setPosition(stoneterminalAngle-0.1);
//        sleep(500);
//        aClamp.setPosition(aCout);
//        sleep(500);
//        ServoStone.setPosition(stoneStartAngle);
//        sleep(400);
//        EncoderStrafe(9.5, op);
//    }
//
//    public void releaseSeq(LinearOpMode op) {
//        EncoderStrafe(-10, op);
//        ServoStone.setPosition(stoneterminalAngle-0.1);
//        sleep(500);
//        aClamp.setPosition(aCout);
//        sleep(500);
//        ServoStone.setPosition(stoneStartAngle);
//        sleep(400);
//        EncoderStrafe(10, op);
//    }
//
        public void foundationup(LinearOpMode op) {
            leftFoundation.setPosition(0.429); //up posiiton
            rightFoundation.setPosition(0.6);
        }

        public void foundationdown(LinearOpMode op){
            leftFoundation.setPosition(0.32); //downpos
            rightFoundation.setPosition(0.491);
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
}
