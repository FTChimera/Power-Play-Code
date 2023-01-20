package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpNew")
public class ATest_Copy extends LinearOpMode {
    public DcMotor motorFrontRight;
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor Arm;
    public DcMotor turnArm;
    public Servo gripServo;
    public Servo flipServo;

    Boolean a = true;
    Boolean b = true;
    Boolean c = false;
    int d;

    int ArmPosition1 = 10;
    int ArmPosition2 = 205;
    int ArmPosition3 = 400;
    int ArmPosition4 = 520;
    int ArmPosition = 1;


    int turnArmPosition1 = -1800;
    int turnArmPosition3 = 0;
    int turnArmPosition5 = 1800;
    int turnArmPosition = 3;


    double flipPosition1 = 0;
    double flipPosition2 = 0.37;
    double flipPosition3 = 0.41;
    double flipPosition4 = 0.5;
    double flipPosition5 = 0.59;
    double flipPosition = 0;
    double flipSpeed = 0.005;

    private void motorToPosition(DcMotor motor, int position, double power){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        
        motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        Arm  = hardwareMap.get(DcMotor.class, "Arm");
        turnArm  = hardwareMap.get(DcMotor.class, "turnArm");
        gripServo = hardwareMap.get(Servo.class, "gripServo");
        flipServo = hardwareMap.get(Servo.class, "flipServo");
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.REVERSE);
        turnArm.setDirection(DcMotor.Direction.REVERSE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gripServo.setPosition(0.1);
        
        double e = 10/7;
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*0.985;
            double rx = -gamepad1.right_stick_x;
            double ty = -gamepad2.left_stick_y;
            double tx = gamepad2.left_stick_x*0.985;
            double trx = -gamepad2.right_stick_x;
            if (gamepad1.left_bumper){
                e = 10/3;
            }
            else {
                e = 10/7;
            }
            double denominator = (e)*(Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));

            if (ty != 0 || tx != 0 || trx != 0) {
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
            double frontRightPower = (((y - x + rx) / (denominator))*0.9);
            double backRightPower = (((y + x + rx) / (denominator))*0.9);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            if (gamepad2.guide){
                ArmPosition4 = 590;
                flipPosition5 = 0.42;
                motorToPosition(turnArm, 0, 1);
            }
            else if (gamepad2.back){
                ArmPosition4 = 520;
                flipPosition5 = 0.55;
                turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                
            }
            
            if (gamepad2.left_bumper){
                gripServo.setPosition(0.4);
            }
            if (gamepad2.right_bumper){
                gripServo.setPosition(0.1);
            }

            if (gamepad2.dpad_up){
                motorToPosition(Arm, ArmPosition4, 0.3);
                ArmPosition = 4;
                if (flipPosition <=0.2){
                    flipPosition = flipPosition2;
                }
                c = true;
            }
            else if (gamepad2.dpad_left){
                motorToPosition(Arm, ArmPosition3, 0.3);
                ArmPosition = 3;
                flipPosition = flipPosition4;
            }
            else if (gamepad2.dpad_right){
                motorToPosition(Arm, ArmPosition2, 0.3);
                ArmPosition = 2;
                flipPosition = flipPosition3;
            }
            else if (gamepad2.dpad_down){
                motorToPosition(Arm, ArmPosition1, 0.15);
                ArmPosition = 1;
                flipPosition = flipPosition2;
            }

            if (c && Arm.getCurrentPosition()>= 300){
                flipPosition = flipPosition5;
                c = false;
            }

            if (gamepad2.x) {
                flipPosition = flipPosition1;
            }


            if (gamepad2.left_trigger>0.1) {
                if (turnArm.getCurrentPosition()>=turnArmPosition1) {
                    /*if (turnArmPosition == 3) {
                        motorToPosition(turnArm, turnArmPosition1, 0.2);
                        turnArmPosition = 1;
                    } else if (turnArmPosition == 5) {
                        motorToPosition(turnArm, turnArmPosition3, 0.2);
                        turnArmPosition = 3;
                    }
                    b = false;
                    timer.reset();*/
                    turnArm.setPower(-0.5*gamepad2.left_trigger);
                }
                else  {
                    turnArm.setPower(0);
                }
            }
            else if (gamepad2.right_trigger>0.1) {
                if (turnArm.getCurrentPosition()<=turnArmPosition5) {
                    /*if (turnArmPosition == 1) {
                        motorToPosition(turnArm, turnArmPosition3, 0.2);
                        turnArmPosition = 3;
                    } else if (turnArmPosition == 3) {
                        motorToPosition(turnArm, turnArmPosition5, 0.2);
                        turnArmPosition = 5;
                    }
                    b = false;
                    timer.reset();*/
                    turnArm.setPower(0.5*gamepad2.right_trigger);
                }
                else  {
                    turnArm.setPower(0);
                }
            }
            else  {
                turnArm.setPower(0);
            }

            if (gamepad2.a){
                flipPosition = flipPosition + flipSpeed;
            }
            else if (gamepad2.y){
                flipPosition = flipPosition - flipSpeed;
            }
            if (timer.seconds()>0.2 && !b){
                b = true;
            }
            
            if (Arm.isBusy() && Arm.getPower() >= 0 && ArmPosition == 4 && (Math.abs(Arm.getTargetPosition()-Arm.getCurrentPosition())<=200)){
                motorToPosition(Arm, Arm.getTargetPosition(), 0.1);
            }
            
            flipPosition = Range.clip(1, 0, flipPosition);

            flipServo.setPosition(flipPosition);
            telemetry.addData("a", Arm.getCurrentPosition());
            telemetry.addData("t", turnArm.getCurrentPosition());
            telemetry.addData("p", flipPosition);
            telemetry.update();
        }
    }
}
