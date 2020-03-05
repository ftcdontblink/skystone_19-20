package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name= "DetectRedSSThreeStone", group="Sky autonomous")
//@Disabled//comment out this line before using
public class DetectRedSS3stone extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    MainClass mc = new MainClass();

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = .6f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {1.5f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6.75f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        mc.init(hardwareMap, imu, lastAngles);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.


        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.update();

            if(valLeft < 100) {
                mc.autonClamp.setPosition(0.2);
                mc.autonHook.setPosition(0.27);
                mc.EncoderStrafe(-30, this);
                mc.EncoderMove(-(4+8), this);

                sleep(100);
                mc.EncoderStrafe(-6.5, this);
                sleep(100);
                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.8);
                sleep(700);
                mc.autonHook.setPosition(0);
                mc.autonClamp.setPosition(0.8);
                sleep(100);
                mc.EncoderStrafe(8.5, this);

                mc.EncoderMove(94, this);
                mc.EncoderStrafe(-7.5, this);
                mc.autonHook.setPosition(0.27);
                sleep(300);
                mc.autonClamp.setPosition(0.2);
                sleep(200);
                mc.autonHook.setPosition(0);
                mc.EncoderStrafe(7.5, this);
                mc.autonClamp.setPosition(0.7);
                mc.EncoderMove(-94+24, this);
                sleep(100);

                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.2);
                sleep(300);
                mc.EncoderStrafe(-7.5, this);
                sleep(200);
                mc.autonClamp.setPosition(0.8);
                sleep(700);
                mc.autonHook.setPosition(0);
                mc.EncoderStrafe(9.5, this);

                mc.EncoderMove(98-24-4, this);

                mc.EncoderStrafe(-8.5, this);
                mc.autonHook.setPosition(0.27);
                sleep(300);
                mc.autonClamp.setPosition(0.2);
                sleep(200);
                mc.autonHook.setPosition(0);
                mc.EncoderStrafe(8.5, this);
                mc.autonClamp.setPosition(0.7);
                mc.EncoderMove(-94+24+4-8, this);

                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.2);
                sleep(300);
                mc.EncoderStrafe(-8.5, this);
                sleep(200);
                mc.autonClamp.setPosition(0.8);
                sleep(700);
                mc.autonHook.setPosition(0);
                mc.EncoderStrafe(9.5, this);

                mc.leftFoundation.setPosition(0.65); //up posiiton
                mc.rightFoundation.setPosition(0.65);
                sleep(300);
                mc.EncoderStrafe(-15, this);
                mc.leftFoundation.setPosition(0.2); //downpos
                mc.rightFoundation.setPosition(0.2);
                sleep(500);
                mc.EncoderStrafe(30, this);
                mc.rotate(-80, 1, this);
                mc.leftFoundation.setPosition(0.625); //up posiiton
                mc.rightFoundation.setPosition(0.625);
                sleep(700);
                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.2);
                sleep(300);
                mc.autonClamp.setPosition(0.2);
                mc.EncoderStrafe(-10, this);
                mc.EncoderMove(3, this);
                mc.EncoderStrafe(50, this);
                sleep(10000000);
            } else if(valMid < 100) {               // positions: 2 and 5
                mc.autonClamp.setPosition(0.2);
                mc.autonHook.setPosition(0.27);
                mc.EncoderStrafe(-31.5, 0.65, this);
                mc.EncoderMove(-(4+8+8+6), this);

                mc.rotateFront(25, 0.5, this);
                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.8);
                sleep(700);
                mc.rotateRight(-25, 0.5, this);
                mc.autonHook.setPosition(0);
                mc.autonClamp.setPosition(0.8);
                sleep(100);
                mc.EncoderStrafe(8.5, 0.65,this);

                mc.EncoderMove(94+8+6+4, 0.7,this);
                mc.EncoderStrafe(-7.5, this);
                mc.autonHook.setPosition(0.27);
                sleep(300);
                mc.autonClamp.setPosition(0.2);
                sleep(200);
                mc.autonHook.setPosition(0);
                mc.EncoderStrafe(7.5, this);
                mc.autonClamp.setPosition(0.7);
                mc.EncoderMove(-94+24-6-8-8-4, 0.7,this);
                sleep(100);

                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.2);
                sleep(300);
                mc.EncoderStrafe(-7.5, this);
                sleep(200);
                mc.autonClamp.setPosition(0.8);
                sleep(700);
                mc.autonHook.setPosition(0);
                mc.EncoderStrafe(9.5, this);

                mc.EncoderMove(106-24, 1, this);

                mc.leftFoundation.setPosition(0.65); //up posiiton
                mc.rightFoundation.setPosition(0.65);
                sleep(600);
                mc.EncoderStrafe(-13, 1,this);
                mc.leftFoundation.setPosition(0.2); //downpos
                mc.rightFoundation.setPosition(0.2);
                sleep(400);
                mc.EncoderStrafe(35, 1,this);
                mc.rotate(-80, 1, this);
                mc.leftFoundation.setPosition(0.625); //up posiiton
                mc.rightFoundation.setPosition(0.625);
                sleep(700);
                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.2);
                sleep(300);
                mc.autonClamp.setPosition(0.2);
                mc.EncoderStrafe(-10, 1, this);
                mc.EncoderMove(3, 1,this);
                mc.EncoderStrafe(50, 1,this);
                sleep(10000000);


            } else {                                // positions: 3 and 6
                mc.autonClamp.setPosition(0.2);
                mc.autonHook.setPosition(0.27);
                mc.EncoderStrafe(-30, this);
                mc.EncoderMove(-(4+8+8), this);

                mc.EncoderStrafe(-6.5, this);
                sleep(100);
                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.8);
                sleep(900);
                mc.autonHook.setPosition(0);
                mc.autonClamp.setPosition(0.8);
                sleep(100);
                mc.EncoderStrafe(8.5, this);

                mc.EncoderMove(94+8, this);
                mc.EncoderStrafe(-7.5, this);
                mc.autonHook.setPosition(0.27);
                sleep(300);
                mc.autonClamp.setPosition(0.2);
                sleep(200);
                mc.autonHook.setPosition(0);
                mc.EncoderStrafe(7.5, this);
                mc.autonClamp.setPosition(0.7);
                mc.EncoderMove(-94+24-6, this);
                sleep(100);

                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.2);
                sleep(300);
                mc.EncoderStrafe(-7.5, this);
                sleep(200);
                mc.autonClamp.setPosition(0.8);
                sleep(700);
                mc.autonHook.setPosition(0);
                mc.EncoderStrafe(9.5, this);

                mc.EncoderMove(100-24, this);

                mc.leftFoundation.setPosition(0.625); //up posiiton
                mc.rightFoundation.setPosition(0.625);
                sleep(300);
                mc.EncoderStrafe(-15, this);
                mc.leftFoundation.setPosition(0.2); //downpos
                mc.rightFoundation.setPosition(0.2);
                sleep(500);
                mc.EncoderStrafe(30, this);
                mc.rotate(-80, 1, this);
                mc.leftFoundation.setPosition(0.625); //up posiiton
                mc.rightFoundation.setPosition(0.625);
                sleep(700);
                mc.autonHook.setPosition(0.27);
                mc.autonClamp.setPosition(0.2);
                sleep(300);
                mc.autonClamp.setPosition(0.2);
                mc.EncoderStrafe(-10, this);
                mc.EncoderMove(3, this);
                mc.EncoderStrafe(50, this);
                sleep(10000000);
            }

            while(isStopRequested()) {
                return;
            }
        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

        public int getSkystonePos(int pos) {
            return pos;
        }

    }
}