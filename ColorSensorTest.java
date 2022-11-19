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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.graphics.Color;

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

@TeleOp(name="ColorSensorTest", group="Linear Opmode")

public class ColorSensorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null; 
    private DcMotor motorBackRight = null;
    private DcMotor leftArm = null;
    private Servo GripServo = null;
    private DcMotor rightArm = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DistanceSensor DistanceSensor = null;
    private ColorSensor colorBack = null;
    static int redValue;
    static int greenValue;
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    private void Drive(double vibhu, int FrontLeft, int FrontRight, int BackLeft, int BackRight) {
       int FrontLeftTarget = motorFrontLeft.getCurrentPosition()+(int)(FrontLeft*DRIVE_COUNTS_PER_IN);
       motorFrontLeft.setTargetPosition(FrontLeftTarget);
       motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       int FrontRightTarget = motorFrontRight.getCurrentPosition()+(int)(FrontRight*DRIVE_COUNTS_PER_IN);
       motorFrontRight.setTargetPosition(FrontRightTarget);
       motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       int BackLeftTarget = motorBackLeft.getCurrentPosition()+(int)(BackLeft*DRIVE_COUNTS_PER_IN);
       motorFrontLeft.setTargetPosition(FrontRightTarget);
       motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       int BackRightTarget = motorBackRight.getCurrentPosition()+(int)(BackRight*DRIVE_COUNTS_PER_IN);
       motorBackRight.setTargetPosition(BackLeftTarget);
       motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       
       motorFrontLeft.setPower(vibhu);
       motorFrontRight.setPower(vibhu);
       motorBackLeft.setPower(vibhu);
       motorBackRight.setPower(vibhu);
     }
     

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        leftArm  = hardwareMap.get(DcMotor.class, "leftArm");
        colorBack = hardwareMap.get(ColorSensor.class, "colorBack");
        DistanceSensor = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        GripServo = hardwareMap.get(Servo.class, "GripServo");
        leftIntake  = hardwareMap.get(DcMotor.class, "leftIntake");
        rightArm  = hardwareMap.get(DcMotor.class, "rightArm");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        int color;
       
        
        
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        color = Color.rgb(colorBack.red(), colorBack.alpha(), colorBack.blue());
       
            redValue = colorBack.red();
        
       
            greenValue = colorBack.blue(); 
        
        
         if (redValue > greenValue){  
            
            telemetry.addData("red", redValue);
         }
        else if (greenValue > redValue){
            
            telemetry.addData("green", greenValue);
         }
         else {
             rightIntake.setPower(0);
         }
                 
             
        telemetry.update();    
        
        }  
    }
}
