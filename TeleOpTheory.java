package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class TeleOpTheory extends LinearOpMode {
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public Servo frontGrip;
    public Servo frontFlip;
    public Servo frontTurn;
    public Servo backGrip;
    public Servo backFlip;
    public Servo backTurn;

    double frontFlipPosition = 0.1;
    double backFlipPosition = 0.05;
    double backTurnPosition = 0.528;
    double servoSpeed = 0.0005;

    boolean startMode = true;
    boolean psMode = false;
    boolean backMode = false;

    int highPosition;
    int midPosition;
    int lowPosition;
    int resetPosition;

    int coneInGrip = 0;

    int backFlippedPosition;
    int backNormalPosition;
    int backTurnedLeftPosition;
    int backTurnedRightPosition;
    int backNotTurnedPosition;

    private void motorToPosition(DcMotor motor, int position, double power){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
    @Override
    public void runOpMode(){
        motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        leftArm  = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm  = hardwareMap.get(DcMotor.class, "rightArm");
        frontGrip = hardwareMap.get(Servo.class, "frontGrip");
        frontFlip = hardwareMap.get(Servo.class, "frontFlip");
        frontTurn = hardwareMap.get(Servo.class, "frontTurn");
        backGrip = hardwareMap.get(Servo.class, "backGrip");
        backFlip = hardwareMap.get(Servo.class, "backFlip");
        backTurn = hardwareMap.get(Servo.class, "backTurn");
        
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        waitForStart();
        while (opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*0.985;
            double rx = -gamepad1.right_stick_x;
            double ty = -gamepad2.left_stick_y;
            double trx = -gamepad2.left_stick_x;
            double tx = gamepad2.left_stick_x*0.985;

            double denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));

            if (ty != 0 || tx != 0 || trx != 0) {
                if (gamepad2.left_stick_button){
                    trx = -gamepad2.left_stick_x;
                    tx = 0;

                }
                else {
                    tx = gamepad2.left_stick_x*0.985;
                    trx = 0;
                }
                y = ty;
                x= tx;
                rx=trx;
                double speedBoost = 4;
                if (gamepad2.right_stick_button) {
                    speedBoost = 1.5;
                }
                denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)*speedBoost);
            }
            double frontLeftPower = ((y + x - rx) / (denominator));
            double backLeftPower = ((y - x - rx) / (denominator));
            double frontRightPower = (((y - x + rx) / (denominator))*1.1);
            double backRightPower = (((y + x + rx) / (denominator)));
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            if (startMode) {

                /*

                Start (Manual) Mode

                 */





                if (gamepad2.dpad_up) {
                    leftArm.setPower(1);
                    rightArm.setPower(1);
                } else if (gamepad2.dpad_down && leftArm.getCurrentPosition()>=0) {
                    leftArm.setPower(-0.5);
                    rightArm.setPower(-0.5);
                } else {
                    leftArm.setPower(0.1);
                    rightArm.setPower(0.1);
                }

                

                /*
                Servo manual control
                */


                
                
                
                
                if (gamepad2.left_bumper){
                    backGrip.setPosition(0.4);
                }
                if (gamepad2.right_bumper){
                    backGrip.setPosition(0.1);
                }
                /*if (gamepad2.y) {
                    backFlipPosition = backFlipPosition + servoSpeed;
                } else if (gamepad2.a) {
                    backFlipPosition = backFlipPosition - servoSpeed;
                }*/
                
                if (gamepad2.y){
                    backFlipPosition = 0.2;
                }
                else if (gamepad2.a){
                    backFlipPosition = 0.05;
                }
                backFlipPosition = Range.clip(backFlipPosition, 0, 1);
                backFlip.setPosition(backFlipPosition);
                if (gamepad2.dpad_left) {
                    backTurnPosition = backTurnPosition + servoSpeed;
                } else if (gamepad2.dpad_right) {
                    backTurnPosition = backTurnPosition - servoSpeed;
                }
                
                if (gamepad2.left_trigger>0){
                    backTurnPosition = 0.19;
                }
                else if (gamepad2.right_trigger>0){
                    backTurnPosition = 0.875;
                }
                else if (gamepad2.x){
                    backTurnPosition = 0.528;
                }
                
                backTurnPosition = Range.clip(backTurnPosition, 0.19, 0.875);
                backTurn.setPosition(backTurnPosition);
                
                telemetry.addData("bt", backTurnPosition);
                telemetry.addData("bf", backFlipPosition);
                telemetry.update();
            }
            else if (psMode){

                /*

                Set Positions, But not fully automated
                flips and turns not automated

                 */


            }
            else {

                /*

                Fully Automated

                 */

                if (coneInGrip == 0 || coneInGrip == 1){
                    
                }


                if (gamepad2.dpad_up) {
                    motorToPosition(leftArm, highPosition, 0.5);
                    motorToPosition(rightArm, highPosition, 0.5);
                }
                else if (gamepad2.dpad_down) {
                    motorToPosition(leftArm, resetPosition, 0.5);
                    motorToPosition(rightArm, resetPosition, 0.5);
                }
                else if (gamepad2.dpad_left) {
                    motorToPosition(leftArm, midPosition, 0.5);
                    motorToPosition(rightArm, midPosition, 0.5);
                }
                else if (gamepad2.dpad_right) {
                    motorToPosition(leftArm, lowPosition, 0.5);
                    motorToPosition(rightArm, lowPosition, 0.5);
                }

                



            }



            if (gamepad2.start){
                psMode = false;
                startMode = true;
                backMode = false;
            }
            else if (gamepad2.ps){
                psMode = true;
                startMode = false;
                backMode = false;
            }
            else if (gamepad2.back){
                psMode = false;
                startMode = false;
                backMode = true;
            }


        }
    }
}
