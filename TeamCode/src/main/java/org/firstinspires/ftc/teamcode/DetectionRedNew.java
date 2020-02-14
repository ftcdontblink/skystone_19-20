/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Disabled
@Autonomous(name = "DetectionRed", group = "Concept")
//@Disabled
public class DetectionRedNew extends LinearOpMode {
    MainClass mc = new MainClass();
    public static final int    STONE_LENGTH = 8;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static int POSITION = 1;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AfAUOgP/////AAABmaXSdBrsgUvRt35hypl5ZuxRs09+g9YqRI2l08mENP9PHTufz6Vb/Ba/uv2iLAWuEfa5iwefGmpzxFPuq2QTFSHXB96XGN/vDHSRsOwQjAY671mtOLCvOFMoY0ishmo8wLgDlcf1ciScMlaeYtXWopM+00e+hIWtviKgGnKrxt3yuDDQLupZFexN8UysvEExDx1AefZePzwwgVdzMppKjBU2WRTInJ5q50LzNjoSN1zoEr+PPh2jSLkRYpsJMli9PSF05Sqi2mZXqkKfpbgLG1/Y+5NUQlhGRFWzdNMSMURae+x/3R9N5QXpR8mirmZlhMBQ3aZ8wJbHEVe/xvuf1cHV1+DXH0fsOwWbF3idMzXu";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    int stoneNumber = 0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        mc.init(hardwareMap, imu, lastAngles);
        mc.init(hardwareMap, imu, lastAngles);
        mc.ServoStone.setPosition(mc.stoneStartAngle);
        mc.FlipLeft.setPosition(mc.FLIP_LEFT_UP_ANGLE);
        mc.FlipRight.setPosition(mc.FLIP_RIGHT_UP_ANGLE);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        tfod.setClippingMargins(220, 140, 0, 0);//Reduces the view from the right side

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            POSITION = 1;
            mc.EncoderStrafe(-8, this);//Amount of inches from the wall
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", POSITION), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", POSITION), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", POSITION), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.update();
                            sleep(300);
                            if(recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
                                //This checks whether the stone detected is the skystone and acts accordingly
                                tfod.shutdown();
                                switch(POSITION) {
                                    case 1: //FIRST POSITION
                                        mc.EncoderMove(1.5, this);
                                        mc.EncoderStrafe(-23, this); //Position and plow for grabbing skystone
                                        sleep(500);
                                        mc.ServoStone.setPosition(mc.stoneterminalAngle); //Grab Skystone
                                        sleep(500);
                                        mc.EncoderStrafe(15, this); //Deliver Skystone
                                        mc.EncoderMove(32, 0.4,this);
                                        sleep(500);
                                        mc.ServoStone.setPosition(mc.stoneStartAngle); //Release Skystone
                                        sleep(500);
                                        mc.EncoderMove(-47.5, this);
                                        mc.EncoderStrafe(-15, this); //Move back to positioning
                                        sleep(500);
                                        mc.ServoStone.setPosition(mc.stoneterminalAngle); //Pick up next Skystone
                                        sleep(500);
                                        mc.EncoderStrafe(15, this);
                                        mc.EncoderMove(54, 0.4, this); //Deliver Skystone
                                        sleep(500);
                                        mc.ServoStone.setPosition(mc.stoneStartAngle); //Release Skystone
                                        sleep(500);
                                        mc.EncoderMove(-20, this); //Park and wait
                                        sleep(60000);
                                        break;
                                    case 2: //SECOND POSITION
                                        mc.EncoderMove(3, this); //Plowing Stones
                                        mc.EncoderStrafe(-23, this);
                                        sleep(500);
                                        mc.ServoStone.setPosition(mc.stoneterminalAngle);//Grab Skystone
                                        sleep(500);
                                        mc.EncoderMove(1, this);
                                        mc.EncoderStrafe(17, this);//Move to building zone
                                        mc.EncoderMove(35, 0.4, this);
                                        sleep(500);
                                        mc.ServoStone.setPosition(mc.stoneStartAngle);//Release stone
                                        sleep(500);
                                        mc.EncoderMove(-54, this);
                                        sleep(500);
                                        mc.EncoderStrafe(-15, this);
                                        mc.ServoStone.setPosition(mc.stoneterminalAngle); //Pick up next skystone
                                        mc.EncoderMove(1.5, this);
                                        sleep(500);
                                        mc.EncoderMove(1, this);
                                        mc.EncoderStrafe(17, this);
                                        mc.EncoderMove(54, 0.4, this); //Deliver to building zone
                                        sleep(100);
                                        mc.ServoStone.setPosition(mc.stoneStartAngle); //Release skystone
                                        sleep(100);
                                        mc.EncoderMove(-13, this);
                                        sleep(60000);
                                        break;
                                    case 3://THIRD POSITION
                                        mc.EncoderMove(6, this);
                                        mc.EncoderStrafe(-23, this); //Position for grabbing
                                        sleep(200);
                                        mc.ServoStone.setPosition(mc.stoneterminalAngle); //Grab stone

                                        sleep(200);
                                        mc.EncoderMove(-1, this);

                                        mc.EncoderStrafe(18, this); //Deliver Skystone
                                        mc.EncoderMove(44, 0.4, this);
                                        sleep(200);
                                        mc.ServoStone.setPosition(mc.stoneStartAngle); //Release Skystone
                                        sleep(200);
                                        mc.EncoderMove(-42, 0.7, this);
                                        mc.rotate(180, 1, this);
                                        mc.FlipRight.setPosition(0.64);
                                        mc.FlipLeft.setPosition(0.27);
                                        sleep(900);
                                        mc.EncoderStrafe(20, this);
                                        mc.LeftIntake.setPower(1);
                                        mc.RightIntake.setPower(-1);
                                        mc.EncoderMove(6, this);
                                        mc.LeftIntake.setPower(0.1);
                                        mc.RightIntake.setPower(-0.1);
                                        mc.EncoderMove(-2, this);
                                        mc.EncoderStrafe(-22, this);
                                        mc.EncoderMove(-46, 1, this);
                                        mc.rotate(180, 1,this);
                                        mc.LeftIntake.setPower(-1);
                                        mc.RightIntake.setPower(1);
                                        sleep(200);
                                        mc.EncoderMove(-10, 1,this);
                                        sleep(50000);
                                        break;
                                }
                            } else if(recognition.getLabel().equals(LABEL_FIRST_ELEMENT)){
                                POSITION++;
                                nextStone();
                                sleep(500);
                                if(POSITION == 4) {
                                    mc.EncoderMove(12.5, this);
                                    mc.EncoderStrafe(-23, this); //Position for grabbing
                                    mc.EncoderMove(1, this);
                                    sleep(200);
                                    mc.ServoStone.setPosition(mc.stoneterminalAngle); //Grab stone
                                    sleep(200);
                                    mc.EncoderStrafe(18, this); //Deliver Skystone
                                    mc.EncoderMove(44, this);
                                    sleep(500);
                                    mc.ServoStone.setPosition(mc.stoneStartAngle); //Release Skystone
                                    sleep(500);
                                    mc.EncoderMove(-42, 0.7, this);
                                    mc.rotate(180, 1, this);
                                    mc.FlipRight.setPosition(0.64);
                                    mc.FlipLeft.setPosition(0.27);
                                    sleep(900);
                                    mc.EncoderStrafe(20, this);
                                    mc.LeftIntake.setPower(1);
                                    mc.RightIntake.setPower(-1);
                                    mc.EncoderMove(6, this);
                                    mc.LeftIntake.setPower(0.1);
                                    mc.RightIntake.setPower(-0.1);
                                    mc.EncoderMove(-2, this);
                                    mc.EncoderStrafe(-22, this);
                                    mc.EncoderMove(-46, 1, this);
                                    mc.rotate(180, 1,this);
                                    mc.LeftIntake.setPower(-1);
                                    mc.RightIntake.setPower(1);
                                    sleep(200);
                                    mc.EncoderMove(-10, 1,this);
                                    sleep(50000);
                                    break;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
            if (tfod != null) {
                tfod.shutdown();
            }
        }
    }
    /**
     * This method moves the robot towards the skystone and puts the arm over the skystone
     */
    public void skystone(){
        mc.EncoderMove(4, this);
        mc.EncoderStrafe(-27, this);//The distance we move towards the skystone
        sleep(200);
        mc.ServoStone.setPosition(mc.stoneterminalAngle);//Puts the arm down
    }

    /**
     * Continues to strafe for a certain distance
     */
    public void nextStone(){
        mc.EncoderMove(-6.5, this);//Moves past one stone
    }

    /**
     * This method is for repositioning the robot to the first stone
     * We do not know where we are when we detect a skystone so this method moves our robot to a certain position
     */
    public void reposition(){
        sleep(500);
        mc.EncoderStrafe(20, this);//Moves away from the quarry
        mc.EncoderMove(-STONE_LENGTH*(POSITION-1), this);//Moves to the first stone
    }

    /**
     * This method delivers the Skystone to the building zone from the first stone
     */
    public void deliver(){
        mc.EncoderMove(-18, this);//Delivers the stone into the building zone
        mc.ServoStone.setPosition(mc.stoneStartAngle);//Brings the servo up
    }

    /**
     * This method moves the robot to the alliance sky bridge
     */
    public void navigate(){
        mc.EncoderMove(10, this);//Parks under the alliance sky bridge
    }

    /**
     * This method does all of the actions for going to the second skystone
     * It goes to the skystone and moves it across the alliance skybridge before navigating
     */
    public void deliver2(){
        POSITION += 3;
        mc.EncoderMove(POSITION*4, this);//Goes to the second skystone
        mc.EncoderStrafe(-20, this);//Moves away from the quarry
        mc.ServoStone.setPosition(mc.stoneterminalAngle);//Brings the servo down
        reposition();//Repositioned to the first stone
        mc.EncoderMove(POSITION*-STONE_LENGTH, this);
        deliver();//Delivers the skystone
        mc.EncoderMove(14, this);
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minimumConfidence = 0.7;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
