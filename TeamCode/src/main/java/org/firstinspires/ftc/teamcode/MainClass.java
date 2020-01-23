package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.BNO055IMUImpl;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
    public Servo FlipLeft;
    public Servo FlipRight;
    static final double COUNTS_PER_MOTOR_REV_LET = 28;
    static final double DRIVE_GEAR_REDUCTION_LET = 19.2;
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
    public double stoneStartAngle = 0;
    public double stoneterminalAngle = 0.48;
    public int PivotArmAngle; //TODO: Add/change values for this to be accurate (Line 231
    public int PivotArmTerminalAngle; //TODO: Add.change values - may need to cast in Mecanum_Drive
    public Servo aClamp;
    public double safety = 16;

    public int flip = 1;

    public int firstflipstart;
    public int firstflipterm;
    public int secondflipstart;
    public int secondflipterm;

    public int position = 0;
    public int add = 0;         //subjective to change

    //Setting Motor values
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime autoMs = new ElapsedTime();

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
    static final double FLIP_LEFT_UP_ANGLE = 0.47;
    static final double FLIP_RIGHT_UP_ANGLE = 0.45;
    static final double FLIP_RIGHT_SPIT_ANGLE = 0.5575;
    static final double FLIP_LEFT_SPIT_ANGLE = 0.362;
    static final double aCout = 0;
    static final double aCin = 0.4;

    int scenario;

    HardwareMap asn;
    double inchToTick;
    int move = 0;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;


    public MainClass() {

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void init(HardwareMap h, BNO055IMU i, Orientation o) {
        imu = i;
        lastAngles = o;
        HardwareMap hwMap = h;
        lFrontMotor = hwMap.get(DcMotor.class, "left_Front_Motor"); // Defining Motors
        rFrontMotor = hwMap.get(DcMotor.class, "right_Front_Motor");
        lBackMotor = hwMap.get(DcMotor.class, "left_Back_Motor");
        rBackMotor = hwMap.get(DcMotor.class, "right_Back_Motor");
        ServoLeft = hwMap.get(Servo.class, "stone_left");      // Defining Servos
        ServoRight = hwMap.get(Servo.class, "stone_right");
        ServoStone = hwMap.get(Servo.class, "servo_stone");
        LeftIntake = hwMap.get(DcMotor.class, "left_intake");
        RightIntake = hwMap.get(DcMotor.class, "right_intake");
        FlipRight = hwMap.get(Servo.class, "flip_right");
        FlipLeft = hwMap.get(Servo.class, "flip_left");
        aClamp = hwMap.get(Servo.class, "aClamp");


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

        FlipRight.setPosition(0.50);
        FlipLeft.setPosition(0.43);

        ServoStone.setPosition(stoneStartAngle);

        aClamp.setPosition(aCout);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
        imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        imu.initialize(parameters);
        resetAngle();
    }

    public void buildingZoneBlueIn(LinearOpMode op, Telemetry t) {
        EncoderMove(-5, op);
        EncoderStrafe(8, op);
        EncoderMove(-12.5, op);
        sleep(500);
        ServoLeft.setPosition(1);
        ServoRight.setPosition(0);
        sleep(500);
        EncoderMove(18, op);
        rotate(100,0.6, op);
        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        sleep(500);
        EncoderMove(-8, op);
        EncoderStrafe(-14, op);
        sleep(13000);
        EncoderMove(29, op);
    }

    public void buildingZoneBlueOut(LinearOpMode op, Telemetry t) {
        EncoderMove(-5, op);
        EncoderStrafe(8, op);
        EncoderMove(-12.5, op);
        sleep(500);
        ServoLeft.setPosition(1);
        ServoRight.setPosition(0);
        sleep(500);
        EncoderMove(18, op);
        rotate(100,0.6, op);
        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        sleep(500);
        EncoderMove(-8, op);
        EncoderStrafe(10, op);
        sleep(13000);
        EncoderMove(29, op);
    }

    public void buildingZoneRedIn(LinearOpMode op, Telemetry t) {
        EncoderMove(-5, op);
        EncoderStrafe(-8, op);
        EncoderMove(-12, op);
        sleep(500);
        ServoLeft.setPosition(1);
        ServoRight.setPosition(0);
        sleep(500);
        EncoderMove(18, op);
        rotate(-90,1, op);
        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        sleep(500);
        EncoderMove(-8, op);
        EncoderStrafe(6, op);
        sleep(13000);
        EncoderMove(29, op);
    }

    public void buildingZoneRedOut(LinearOpMode op, Telemetry t) {
        EncoderMove(-5, op);
        EncoderStrafe(-8, op);
        EncoderMove(-12, op);
        sleep(500);
        ServoLeft.setPosition(1);
        ServoRight.setPosition(0);
        sleep(500);
        EncoderMove(18, op);
        rotate(-90,1, op);
        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        sleep(500);
        EncoderMove(-8, op);
        EncoderStrafe(-12, op);
        sleep(13000);
        EncoderMove(29, op);
    }

    public void bluepos1Super(LinearOpMode op) {
        EncoderStrafe(-24.25, op);
        pickUpSeq(op);
        EncoderStrafe(5, op);
        EncoderMove(-46, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove(67.25, 1, op);
        sleep(150);
        EncoderStrafe(-9, op);
        pickUpSeq(op);
        EncoderStrafe(5.5, op);
        EncoderMove(-73.5, 1, op);
        sleep(150);
        rotate(-90, 1, op);
        EncoderMove(-5, op);
        ServoLeft.setPosition(1);
        ServoRight.setPosition(0);
        EncoderMove(15, 1, op);
        rotate(90, 1, op);
        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        rotate(90, 1, op);
        releaseSeqLess(op);
        EncoderMove(15, 1, op);
        EncoderStrafe(35, op);
        sleep(200);

        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        EncoderStrafe(-12, op);
        EncoderMove(-20, op);
        EncoderStrafe(-15, op);
        sleep(60000);
    }

    public void bluepos2Super(LinearOpMode op) {
        EncoderStrafe(-24, op);
        EncoderMove(5.5, op);
        pickUpSeq(op);
        EncoderStrafe(3, op);
        EncoderMove(-50, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove((65.5+4), 1, op);
        sleep(150);
        EncoderStrafe(-5.5, op);
        pickUpSeq(op);
        EncoderStrafe(5, op);
        EncoderMove(-(73.5+4), 1, op);
        sleep(150);
        releaseSeqLess(op);
        rotate(90, 1, op);
        EncoderMove(-5, op);
        ServoLeft.setPosition(1);
        ServoRight.setPosition(0);
        EncoderMove(30, 1, op);
        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        EncoderStrafe(-50, op);
        sleep(60000);
    }

    public void bluepos3Super(LinearOpMode op) {
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        sleep(200);
        EncoderStrafe(-24, op);
        EncoderMove(10, op);
        pickUpSeq(op);
        EncoderStrafe(3, op);
        EncoderMove(-56, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove((64), 1, op);
        sleep(150);
        EncoderStrafe(-17, op);
        LeftIntake.setPower(0.8);
        RightIntake.setPower(-0.8);
        sleep(100);
        EncoderMove(9, 1, op);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
        EncoderMove(-5, 1, op);
        FlipRight.setPosition(0.5575);
        FlipLeft.setPosition(0.362);
        EncoderStrafe(17, op);
        EncoderMove(-70, 1, op);
        rotate(90, 1, op);
        EncoderMove(5, op);
        sleep(400);
        LeftIntake.setPower(-1);
        RightIntake.setPower(1);
        sleep(400);
        rotate(180, 1, op);
        EncoderMove(-10, op);
        ServoLeft.setPosition(1);
        ServoRight.setPosition(0);
        EncoderMove(35, 1, op);
        ServoLeft.setPosition(0);
        ServoRight.setPosition(1);
        EncoderStrafe(-50, op);
        sleep(60000);
    }

    public void bluepos1(LinearOpMode op) {
        EncoderStrafe(-23.5, op);
        EncoderMove(-1, op);
        pickUpSeq(op);
        EncoderStrafe(8, op);
        EncoderMove(-46,1, op);
        EncoderStrafe(-3, op);
        sleep(150);
        releaseSeqLess(op);
        sleep(100);
        EncoderStrafe(-3, op);
        EncoderMove(66, 1, op);
        sleep(150);
        EncoderStrafe(-6.5, op);
        pickUpSeq(op);
        EncoderStrafe(9, op);
        EncoderMove(-73.5, 1, op);
        EncoderStrafe(-4, op);
        sleep(150);
        releaseSeqLess(op);
        sleep(100);
        EncoderStrafe(-3, op);
        EncoderMove(35, 1, op);
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        sleep(60000);
//        park(op);
    }

    public void bluepos2(LinearOpMode op) {
        EncoderStrafe(-23.5, op);
        EncoderMove(4.25, op);
        pickUpSeq(op);
        EncoderStrafe(8, op);
        EncoderMove(-50, 1, op);
        sleep(150);
        EncoderStrafe(-3, op);
        releaseSeqLess(op);
        EncoderMove((65.5+4), 1, op);
        sleep(150);
        EncoderStrafe(-9.1, op);
        pickUpSeq(op);
        EncoderStrafe(5, op);
        EncoderMove(-74.5, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove((25),  1, op);
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        sleep(60000);
//        park(op);
    }

    public void bluepos3(LinearOpMode op) {
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        sleep(200);
        EncoderStrafe(-23.5, op);
        EncoderMove(10, op);
        pickUpSeq(op);
        EncoderStrafe(8, op);
        EncoderMove(-56, 1, op);
        sleep(150);
        EncoderStrafe(-3, op);
        releaseSeqLess(op);
        EncoderMove((61.5), 1,  op);
        sleep(150);
        EncoderStrafe(-17, op);
        LeftIntake.setPower(0.8);
        RightIntake.setPower(-0.8);
        sleep(100);
        EncoderMove(4, op);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
        EncoderMove(-5,  op);
        FlipRight.setPosition(0.5575);
        FlipLeft.setPosition(0.362);
        EncoderStrafe(17, op);
        EncoderMove(-74,  1, op);
        rotate(90, 1, op);
        EncoderMove(5, op);
        sleep(400);
        LeftIntake.setPower(-1);
        RightIntake.setPower(1);
        sleep(400);
        rotate(-90, 1, op);
        EncoderMove(25, 1, op);
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        sleep(60000);
//        park(op);
    }

    public void bluepos1NP(LinearOpMode op){
        EncoderStrafe(-24, op);
        pickUpSeq(op);
        EncoderStrafe(3, op);
        EncoderMove(-46+safety, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove(65.5-safety, 1, op);
        sleep(150);
        EncoderStrafe(-5.15, op);
        pickUpSeq(op);
        EncoderStrafe(5, op);
        EncoderMove(-73.5+safety, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove(35-safety, 1, op);
        sleep(60000);
        park(op);
    }
    public void bluepos2NP(LinearOpMode op) {
        EncoderStrafe(-24, op);
        EncoderMove(5.5, op);
        pickUpSeq(op);
        EncoderStrafe(3, op);
        EncoderMove(-50+safety, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove((65.5+4)-safety, 1, op);
        sleep(150);
        EncoderStrafe(-5.5, op);
        pickUpSeq(op);
        EncoderStrafe(5, op);
        EncoderMove(-(73.5+4)+safety, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove((35)-safety, 1, op);
        sleep(60000);
        park(op);
    }
    public void bluepos3NP(LinearOpMode op){
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        sleep(200);
        EncoderStrafe(-24, op);
        EncoderMove(10, op);
        pickUpSeq(op);
        EncoderStrafe(3, op);
        EncoderMove(-56+safety, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove((64-safety), 1, op);
        sleep(150);
        EncoderStrafe(-17, op);
        LeftIntake.setPower(0.8);
        RightIntake.setPower(-0.8);
        sleep(100);
        EncoderMove(9, 1, op);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
        EncoderMove(-5, 1, op);
        FlipRight.setPosition(0.5575);
        FlipLeft.setPosition(0.362);
        EncoderStrafe(17, op);
        EncoderMove(-70+safety, 1, op);
        rotate(180, 1, op);
        sleep(400);
        LeftIntake.setPower(-1);
        RightIntake.setPower(1);
        sleep(400);
        EncoderMove(-25, op);
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        sleep(60000);

    }


    public void redpos1(LinearOpMode op) {
        EncoderStrafe(-23.25, op);
        EncoderMove(11, op);
        pickUpSeq(op);
        EncoderStrafe(8, op);
        EncoderMove(46, 1, op);
        sleep(150);
        EncoderStrafe(-3, op);
        releaseSeqLess(op);
        EncoderMove(-(63), 1, op);
        sleep(150);
        EncoderStrafe(-6, op);
        pickUpSeq(op);
        EncoderStrafe(8, op);
        EncoderMove((70), 1, op);
        sleep(150);
        EncoderStrafe(-2, op);
        releaseSeqLess(op);
        EncoderMove(-(35), 1, op);
        sleep(60000);
    }

    public void redpos2(LinearOpMode op) {
        EncoderStrafe(-23.5, op);
        EncoderMove(4.25, op);
        pickUpSeq(op);
        EncoderStrafe(6, op);
        EncoderMove(52.5, 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove(-(67), 1, op);
        sleep(150);
        EncoderStrafe(-9.5, op);
        pickUpSeq(op);
        EncoderStrafe(5, op);
        EncoderMove((73.5+5), 1, op);
        sleep(150);
        releaseSeqLess(op);
        EncoderMove(-(35), 1, op);
        sleep(60000);

    }

    public void redpos3(LinearOpMode op) {
        FlipRight.setPosition(0.62); //Changing position of intake servos (in order to exit the sizing grid limit)
        FlipLeft.setPosition(0.29);
        sleep(200); //Wait for .02 seconds before
        EncoderStrafe(-24, op); //Move towards stones
        EncoderMove(-10, op);
        pickUpSeq(op);//Pick up stone
        EncoderStrafe(3, op); //Move away from the stone
        EncoderMove(56, 1, op); //Deliver stone
        sleep(150); //wait
        releaseSeqLess(op); //Release Stone
        EncoderMove((-64), 1, op);
        rotate(180, 1, op);
        sleep(150);
        EncoderStrafe(-17, op);
        LeftIntake.setPower(0.8);
        RightIntake.setPower(-0.8);
        sleep(100);
        EncoderMove(9, 1, op);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
        EncoderMove(-5, 1, op);
        FlipRight.setPosition(0.5575);
        FlipLeft.setPosition(0.362);
        EncoderStrafe(17, op);
        EncoderMove(-70, 1, op);
        rotate(-90, 1, op);
        EncoderMove(5, op);
        sleep(400);
        LeftIntake.setPower(-1);
        RightIntake.setPower(1);
        sleep(400);
        EncoderStrafe(25, op);
        rotate(-90, 1, op);
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        sleep(60000);
    }

    public void pickUpSeq(LinearOpMode op) {
        ServoStone.setPosition(stoneterminalAngle);
        sleep(800);
        aClamp.setPosition(aCin);
        sleep(800);
        ServoStone.setPosition(stoneStartAngle);
    }

    public void park(LinearOpMode op){
        ServoStone.setPosition(stoneterminalAngle);
        FlipRight.setPosition(0.62);
        FlipLeft.setPosition(0.29);
        ServoLeft.setPosition(1);
        ServoRight.setPosition(0);

    }

    public void releaseSeqLess(LinearOpMode op) {
        EncoderStrafe(-9.5, op);
        ServoStone.setPosition(stoneterminalAngle-0.1);
        sleep(500);
        aClamp.setPosition(aCout);
        sleep(500);
        ServoStone.setPosition(stoneStartAngle);
        sleep(400);
        EncoderStrafe(9.5, op);
    }

    public void releaseSeq(LinearOpMode op) {
        EncoderStrafe(-10, op);
        ServoStone.setPosition(stoneterminalAngle-0.1);
        sleep(500);
        aClamp.setPosition(aCout);
        sleep(500);
        ServoStone.setPosition(stoneStartAngle);
        sleep(400);
        EncoderStrafe(10, op);
    }

    public void EncoderMove1(double inches, LinearOpMode op) {
        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        autoMs.reset();

        // cpi * pi * diameter / final * drive

        while(lFrontMotor.isBusy() && lBackMotor.isBusy() && rFrontMotor.isBusy() && rBackMotor.isBusy() && op.opModeIsActive()) {
            lFrontMotor.setPower(0.6);
            rFrontMotor.setPower(0.6);
            lBackMotor.setPower(0.6);
            rBackMotor.setPower(0.6);
        }

        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void Encoderstrafe(double inches, LinearOpMode op) {
        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        runtime.reset();

        while(lFrontMotor.isBusy()
                && lBackMotor.isBusy()
                && rFrontMotor.isBusy()
                && rBackMotor.isBusy()
                && op.opModeIsActive()
                && runtime.seconds() < 30)
        {
            lFrontMotor.setPower(0.5);
            lBackMotor.setPower(-0.5);
            rFrontMotor.setPower(-0.5);
            rBackMotor.setPower(0.5);
        }

        while (op.opModeIsActive() &&
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

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void EncoderMove(double inches, LinearOpMode op) {

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double tp = inches * COUNTS_PER_INCH;

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;

        while(op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(lFrontMotor.getCurrentPosition())
                    + Math.abs(lBackMotor.getCurrentPosition())
                    + Math.abs(rFrontMotor.getCurrentPosition())
                    + Math.abs(rBackMotor.getCurrentPosition()))/4.0;
            if(inches > 0) {
                lFrontMotor.setPower(0.6 - correction);
                rFrontMotor.setPower(0.6 + correction);
                lBackMotor.setPower(0.6 - correction);
                rBackMotor.setPower(0.6 + correction);
            } else {
                lFrontMotor.setPower(-0.6 - correction);
                rFrontMotor.setPower(-0.6 + correction);
                lBackMotor.setPower(-0.6 - correction);
                rBackMotor.setPower(-0.6 + correction);
            }

            if(Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        lFrontMotor.setPower(0);
        rFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rBackMotor.setPower(0);

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void EncoderMove(double inches, double power, LinearOpMode op) {
        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double tp = inches * COUNTS_PER_INCH;

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;

        while(op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(lFrontMotor.getCurrentPosition())
                    + Math.abs(lBackMotor.getCurrentPosition())
                    + Math.abs(rFrontMotor.getCurrentPosition())
                    + Math.abs(rBackMotor.getCurrentPosition()))/4.0;
            if(inches > 0) {
                lFrontMotor.setPower(power - correction);
                rFrontMotor.setPower(power + correction);
                lBackMotor.setPower(power - correction);
                rBackMotor.setPower(power + correction);
            } else {
                lFrontMotor.setPower(-power - correction);
                rFrontMotor.setPower(-power + correction);
                lBackMotor.setPower(-power - correction);
                rBackMotor.setPower(-power + correction);
            }

            if(Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }

        lFrontMotor.setPower(0);
        rFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rBackMotor.setPower(0);

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void EncoderStrafe(double inches, LinearOpMode op) {
        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double tp = inches * COUNTS_PER_INCH;

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;

        while (op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(lFrontMotor.getCurrentPosition()) + Math.abs(lBackMotor.getCurrentPosition()) + Math.abs(rFrontMotor.getCurrentPosition()) + Math.abs(rBackMotor.getCurrentPosition())) / 4.0;
            if (inches > 0) {
                lFrontMotor.setPower(0.6 - correction);
                rFrontMotor.setPower(-0.6 + correction);
                lBackMotor.setPower(-0.6 - correction);
                rBackMotor.setPower(0.6 + correction);
            } else {
                lFrontMotor.setPower(-0.6 - correction);
                rFrontMotor.setPower(0.6 + correction);
                lBackMotor.setPower(0.6 - correction);
                rBackMotor.setPower(-0.6 + correction);
            }

            if (Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }


        lFrontMotor.setPower(0);
        rFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rBackMotor.setPower(0);

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set power to rotate.
        lFrontMotor.setPower(leftPower);
        lBackMotor.setPower(leftPower);
        rFrontMotor.setPower(rightPower);
        rBackMotor.setPower(rightPower);

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
        lFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rFrontMotor.setPower(0);
        rBackMotor.setPower(0);

        // wait for rotation to stop.
        sleep(300);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void EncoderStrafeVel(double inches, LinearOpMode op) {
        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double tp = inches * COUNTS_PER_INCH;

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;
        double percentCP;

        while(op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(lFrontMotor.getCurrentPosition()) + Math.abs(lBackMotor.getCurrentPosition()) + Math.abs(rFrontMotor.getCurrentPosition()) + Math.abs(rBackMotor.getCurrentPosition()))/4.0;
            percentCP = Math.abs(avg)/Math.abs(tp);
            if(inches > 0) {
                if(percentCP < 0.1) {
                    lFrontMotor.setPower((percentCP*10 - 0.4) - correction);
                    rFrontMotor.setPower((-percentCP*10 + 0.4) + correction);
                    lBackMotor.setPower((-percentCP*10 + 0.4) - correction);
                    rBackMotor.setPower((percentCP*10 - 0.4) + correction);
                } else if(percentCP > 0.1 && percentCP < 0.9) {
                    lFrontMotor.setPower(-1 - correction);
                    rFrontMotor.setPower(1 + correction);
                    lBackMotor.setPower(1 - correction);
                    rBackMotor.setPower(-1 + correction);
                } else {
                    lFrontMotor.setPower(((1-percentCP)*10) - correction - 0.4);
                    rFrontMotor.setPower((-(1-percentCP)*10) + correction + 0.4);
                    lBackMotor.setPower((-(1-percentCP)*10) - correction + 0.4);
                    rBackMotor.setPower(((1-percentCP)*10) + correction - 0.4);
                }
            } else {
                if (percentCP < 0.1) {
                    lFrontMotor.setPower((-percentCP * 10 - 0.4) - correction);
                    rFrontMotor.setPower((percentCP * 10 + 0.4) + correction);
                    lBackMotor.setPower((percentCP * 10 + 0.4) - correction);
                    rBackMotor.setPower((-percentCP * 10 - 0.4) + correction);
                } else if (percentCP > 0.1 && percentCP < 0.9) {
                    lFrontMotor.setPower(-1 - correction);
                    rFrontMotor.setPower(1 + correction);
                    lBackMotor.setPower(1 - correction);
                    rBackMotor.setPower(-1 + correction);
                } else {
                    lFrontMotor.setPower((-(1 - percentCP) * 10) - correction - 0.4);
                    rFrontMotor.setPower(((1 - percentCP) * 10) + correction + 0.4);
                    lBackMotor.setPower(((1 - percentCP) * 10) - correction + 0.4);
                    rBackMotor.setPower((-(1 - percentCP) * 10) + correction - 0.4);
                }
            }
            if(Math.abs(avg) > Math.abs(tp)) {
                break;
            }

            lFrontMotor.setPower(0);
            rFrontMotor.setPower(0);
            lBackMotor.setPower(0);
            rBackMotor.setPower(0);

            lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void EncoderMoveVel(double inches, LinearOpMode op) {
        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double tp = inches * COUNTS_PER_INCH;

        lFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double avg = 0;
        double percentCP;

        while(op.opModeIsActive()) {
            correction = checkDirection();
            avg = (Math.abs(lFrontMotor.getCurrentPosition()) + Math.abs(lBackMotor.getCurrentPosition()) + Math.abs(rFrontMotor.getCurrentPosition()) + Math.abs(rBackMotor.getCurrentPosition()))/4.0;
            percentCP = avg/tp;
            if(inches > 0) {
                if(percentCP < 0.1) {
                    lFrontMotor.setPower((percentCP*10 + 0.4) - correction);
                    rFrontMotor.setPower((percentCP*10 + 0.4) + correction);
                    lBackMotor.setPower((percentCP*10 + 0.4) - correction);
                    rBackMotor.setPower((percentCP*10 + 0.4) + correction);
                } else if(percentCP > 0.1 && percentCP < 0.9) {
                    lFrontMotor.setPower(1 - correction);
                    rFrontMotor.setPower(1 + correction);
                    lBackMotor.setPower(1 - correction);
                    rBackMotor.setPower(1 + correction);
                } else {
                    lFrontMotor.setPower(((1-percentCP)*10) - correction + 0.4);
                    rFrontMotor.setPower(((1-percentCP)*10) + correction + 0.4);
                    lBackMotor.setPower(((1-percentCP)*10) - correction + 0.4);
                    rBackMotor.setPower(((1-percentCP)*10) + correction + 0.4);
                }
            } else {
                if (percentCP < 0.1) {
                    lFrontMotor.setPower((-percentCP * 10 - 0.4) - correction);
                    rFrontMotor.setPower((-percentCP * 10 - 0.4) + correction);
                    lBackMotor.setPower((-percentCP * 10 - 0.4) - correction);
                    rBackMotor.setPower((-percentCP * 10 - 0.4) + correction);
                } else if (percentCP > 0.1 && percentCP < 0.9) {
                    lFrontMotor.setPower(-1 - correction);
                    rFrontMotor.setPower(-1 + correction);
                    lBackMotor.setPower(-1 - correction);
                    rBackMotor.setPower(-1 + correction);
                } else {
                    lFrontMotor.setPower((-(1 - percentCP) * 10) - correction - 0.4);
                    rFrontMotor.setPower((-(1 - percentCP) * 10) + correction - 0.4);
                    lBackMotor.setPower((-(1 - percentCP) * 10) - correction - 0.4);
                    rBackMotor.setPower((-(1 - percentCP) * 10) + correction - 0.4);
                }
            }
            if(Math.abs(avg) > Math.abs(tp)) {
                break;
            }
        }



        lFrontMotor.setPower(0);
        rFrontMotor.setPower(0);
        lBackMotor.setPower(0);
        rBackMotor.setPower(0);

        lFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
