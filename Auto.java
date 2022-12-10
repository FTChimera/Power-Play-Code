/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.reflect.Array;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.imgcodecs.Imgcodecs;


@Autonomous
public class Auto_NoDrop extends LinearOpMode
{
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null; 
    private DcMotor motorBackRight = null;
    private DcMotor leftArm = null;
    private Servo gripServo = null;
    private DcMotor rightArm = null;
    private Servo leftGripServo;
    private Servo rightGripServo;
    private ColorSensor colorBack = null;
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    OpenCvWebcam webcam;
    SignalPipeline pipeline;
    private ElapsedTime timers = new ElapsedTime();
    private void Drive(double power, double FrontLeftInches, double FrontRightInches, double BackLeftInches, double BackRightInches) {
        int FrontLeftTarget = motorFrontLeft.getCurrentPosition()+(int)(FrontLeftInches*DRIVE_COUNTS_PER_IN)*-1;
        motorFrontLeft.setTargetPosition(FrontLeftTarget);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int FrontRightTarget = motorFrontRight.getCurrentPosition()+(int)(FrontRightInches*DRIVE_COUNTS_PER_IN)*-1;
        motorFrontRight.setTargetPosition(FrontRightTarget);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int BackLeftTarget = motorBackLeft.getCurrentPosition()+(int)(BackLeftInches*DRIVE_COUNTS_PER_IN)*-1;
        motorBackLeft.setTargetPosition(BackLeftTarget);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int BackRightTarget = motorBackRight.getCurrentPosition()+(int)(BackRightInches*DRIVE_COUNTS_PER_IN)*-1;
        motorBackRight.setTargetPosition(BackRightTarget);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        while (motorBackRight.isBusy() && opModeIsActive()){
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    @Override
    public void runOpMode()
    {
        motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        leftArm  = hardwareMap.get(DcMotor.class, "leftArm");
        colorBack = hardwareMap.get(ColorSensor.class, "colorBack");
        leftGripServo = hardwareMap.get(Servo.class, "leftGripServo");
        rightGripServo = hardwareMap.get(Servo.class, "rightGripServo");
        rightArm  = hardwareMap.get(DcMotor.class, "rightArm");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        rightGripServo.setPosition(0.5);
        leftGripServo.setPosition(0.5);
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        
        telemetry.addLine("Waiting for start");
        telemetry.update();
        
        waitForStart();
        
        telemetry.addLine("Started");
        telemetry.update();
        
        Drive(0.25, -8, -8, -8, -8);
        
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SignalPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        int timer = 0;
        int signalPosition = 0;
        while (signalPosition == 0 && opModeIsActive()) {
            signalPosition = pipeline.signalPosition();
            timer++; 
            sleep (10);
        }
        
        {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Time took", timer*10);
            telemetry.addData("Signal Position",signalPosition);
            telemetry.update();
            
            Drive(0.25, -16, -16, -16, -16);
            sleep(250);
            Drive(0.25, 1.5, 1.5, 1.5, 1.5);
            sleep(250);
            if (signalPosition == 1)
                Drive(0.25, 25, -25, -25, 25);
            if (signalPosition == 3)
                Drive(0.25, -24, 24, 24, -24);
                
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    class SignalPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        int position = 0;

        @Override
        public Mat processFrame(Mat input)
        {
               
               if (position != 0) {
                   return input;
               }
               
               List<Mat> rgb = new ArrayList<>();
               Core.split(input, rgb);
               
               int red = 0, green = 0, blue = 0;
               int minValue = 128;
               int maxValue = 128;
               
               for (int i=0; i < rgb.get(0).rows(); i++) {
                   for (int j=0; j < rgb.get(0).cols(); j++) {
                       if (rgb.get(0).get(i,j)[0] > maxValue && rgb.get(1).get(i,j)[0] < minValue && rgb.get(2).get(i,j)[0] < minValue){
                            red++;
                            input.put(i, j, 0, 255, 0, 0); 
                        }
                        else if (rgb.get(0).get(i,j)[0] < minValue && rgb.get(1).get(i,j)[0] > maxValue && rgb.get(2).get(i,j)[0] < minValue){
                            green++;
                            input.put(i, j, 0, 0, 255, 0); 
                        }
                        else if (rgb.get(0).get(i,j)[0] < minValue && rgb.get(1).get(i,j)[0] < minValue && rgb.get(2).get(i,j)[0] > maxValue){
                            blue++;
                            input.put(i, j, 255, 0, 0, 0);
                        }
                   }
                
               }
               
               int g = 0,b = 0,r =0;
               if (red > blue && red > green)
               {
                    r = 255;
                    position = 1;
               }
               else if (green > red && green > blue)
               {
                    g = 255;
                    position = 3;
                }
                else if (blue > red && blue > green)
                {
                    b = 255;
                    position=2;
                }
                
                return input;
        }
        public int signalPosition() 
        {
            return position;
        }
        @Override
        public void onViewportTapped()
        {

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
