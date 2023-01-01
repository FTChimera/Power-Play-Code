package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpNew")
public class ATest_Copy extends LinearOpMode {

  RobotHardware chaos = new RobotHardware();
  public DcMotor motorFrontRight;
  public DcMotor motorFrontLeft;
  public DcMotor motorBackRight;
  public DcMotor motorBackLeft;
  public DcMotor Arm;
  public DcMotor turnArm;
  public Servo gripServo;
  public Servo flipServo;
  
  Boolean a = false;
  Boolean b = true;
  Boolean c = true;
  int d;
  
  int ArmPosition1 = 0;
  int ArmPosition2 = 205;
  int ArmPosition3 = 375;
  int ArmPosition4 = 505;
  int ArmPosition = 1;
  
  
  int turnArmPosition1 = 505;
  int turnArmPosition2 = 205;
  int turnArmPosition3 = 0;
  int turnArmPosition4 = -205;
  int turnArmPosition5 = -505;
  int turnArmPosition = 3;
  
  
  double flipPosition1 = 0;
  double flipPosition2 = 0.345;
  double flipPosition3 = 0.4;
  double flipPosition4 = 0.5;
  double flipPosition5 = 0.67;
  double flipPosition = 2;
  
  private void motorToPosition(DcMotor motor, int position, double power){
    motor.setTargetPosition(position);
    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    motor.setPower(power);
  }
  
  private ElapsedTime timer = new ElapsedTime();
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
    motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
    motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
    motorBackRight.setDirection(DcMotor.Direction.REVERSE);
    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    turnArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    // Put initialization blocks here.
    waitForStart();
    while (opModeIsActive()) {
      
      double y = gamepad1.left_stick_y;
      double x = -gamepad1.left_stick_x*0.985;
      double rx = -gamepad1.right_stick_x;
      double ty = gamepad2.left_stick_y;
      double tx = -gamepad2.left_stick_x*0.985;
      double trx = -gamepad2.right_stick_x;
      double denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));
        
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
      double frontRightPower = (((y - x + rx) / (denominator))*1.1);
      double backRightPower = (((y + x + rx) / (denominator)));
      motorFrontLeft.setPower(frontLeftPower);
      motorBackLeft.setPower(backLeftPower);
      motorFrontRight.setPower(frontRightPower);
      motorBackRight.setPower(backRightPower);
      if (gamepad2.left_bumper){
        gripServo.setPosition(0.5);
      }
      if (gamepad2.right_bumper){
        gripServo.setPosition(0);
      }
      if (gamepad2.dpad_up){
        motorToPosition(Arm, ArmPosition4, 0.3);
        flipServo.setPosition(flipPosition2);
        ArmPosition = 4;
        a = true;
        d = 250;
      }
      else if (gamepad2.dpad_down){
        motorToPosition(Arm, ArmPosition1, 0.2);
        flipServo.setPosition(flipPosition2);
        ArmPosition = 0;
      }
      if (a == true && Arm.getCurrentPosition()>=d){
        flipServo.setPosition(flipPosition5);
        flipPosition = 5;
        a = false;
      }
      if (Arm.isBusy() && Arm.getPower() >= 0 &&(Math.abs(Arm.getTargetPosition()-Arm.getCurrentPosition())<=200)){
        motorToPosition(Arm, Arm.getTargetPosition(), 0.2);
      }
      
      
      
      if (gamepad2.dpad_left && b){
        if (turnArmPosition == 2){
          motorToPosition(turnArm, turnArmPosition1, 0.2);
          turnArmPosition = 1;
        }
        else if (turnArmPosition == 3){
          motorToPosition(turnArm, turnArmPosition2, 0.2);
          turnArmPosition = 2;
        }
        else if (turnArmPosition == 4){
          motorToPosition(turnArm, turnArmPosition3, 0.2);
          turnArmPosition = 3;
        }
        else if (turnArmPosition == 5){
          motorToPosition(turnArm, turnArmPosition4, 0.2);
          turnArmPosition = 4;
        }
        b = false;
        timer.reset();
      }
      else if (gamepad2.dpad_right){
        motorToPosition(turnArm, turnArmPosition3, 0.2);
        turnArmPosition = 3;
      }
      if (turnArm.isBusy() &&(Math.abs(turnArm.getTargetPosition()-turnArm.getCurrentPosition())<=100)){
        motorToPosition(turnArm, turnArm.getTargetPosition(), 0.05);
      }
      
      if (timer.seconds()>0.5){
        b = true;
      }
      
      telemetry.addData("arm", Arm.isBusy());
      telemetry.addData("p", Arm.getPower());
      telemetry.update();
    }
  }
}
