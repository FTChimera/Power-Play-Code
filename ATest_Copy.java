package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
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
  
  Boolean a = true;
  Boolean b = true;
  Boolean c = true;
  int d;
  
  int ArmPosition1 = 0;
  int ArmPosition2 = 205;
  int ArmPosition3 = 375;
  int ArmPosition4 = 480;
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
  double flipPosition5 = 0.6;
  double flipPosition = 0.345;
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
      if (gamepad2.dpad_up && a){
        if (ArmPosition == 1){
          motorToPosition(Arm, ArmPosition2, 0.3);
          ArmPosition = 2;
          flipPosition = flipPosition3;
        }
        else if (ArmPosition == 2){
          motorToPosition(Arm, ArmPosition3, 0.3);
          ArmPosition = 3;
          flipPosition = flipPosition4;
        }
        a = false;
        timer2.reset();
      }
      else if (gamepad2.dpad_down && a){
        if (ArmPosition == 2){
          motorToPosition(Arm, ArmPosition1, 0.2);
          ArmPosition = 1;
          flipPosition = flipPosition2;
        }
        else if (ArmPosition == 3){
          motorToPosition(Arm, ArmPosition2, 0.2);
          ArmPosition = 2;
          flipPosition = flipPosition3;
        }
        a = false;
        timer2.reset();
      }
      if (a == false && timer2.seconds()>0.2){
        a = true;
      }
      if (Arm.isBusy() && Arm.getPower() >= 0 &&(Math.abs(Arm.getTargetPosition()-Arm.getCurrentPosition())<=100)){
        motorToPosition(Arm, Arm.getTargetPosition(), 0.1);
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
      else if (gamepad2.dpad_right && b){
        if (turnArmPosition == 1){
          motorToPosition(turnArm, turnArmPosition2, 0.2);
          turnArmPosition = 2;
        }
        else if (turnArmPosition == 2){
          motorToPosition(turnArm, turnArmPosition3, 0.2);
          turnArmPosition = 3;
        }
        else if (turnArmPosition == 3){
          motorToPosition(turnArm, turnArmPosition4, 0.2);
          turnArmPosition = 4;
        }
        else if (turnArmPosition == 4){
          motorToPosition(turnArm, turnArmPosition5, 0.2);
          turnArmPosition = 5;
        }
        b = false;
        timer.reset();
      }
      if (turnArm.isBusy() &&(Math.abs(turnArm.getTargetPosition()-turnArm.getCurrentPosition())<=175)){
        motorToPosition(turnArm, turnArm.getTargetPosition(), 0.05);
      }
      if (gamepad2.y){
        flipPosition = flipPosition + flipSpeed;
      }
      else if (gamepad2.x){
        flipPosition = flipPosition - flipSpeed;
      }
      if (timer.seconds()>0.2 && b == false){
        b = true;
      }
      flipPosition = Range.clip(1, 0, flipPosition);
      
      flipServo.setPosition(flipPosition);
      telemetry.addData("arm", Arm.isBusy());
      telemetry.addData("p", Arm.getPower());
      telemetry.update();
    }
  }
}
