package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp")
public class ATest extends LinearOpMode {

  RobotHardware chaos = new RobotHardware();
  public DcMotor motorFrontRight;
  public DcMotor motorFrontLeft;
  public DcMotor motorBackRight;
  public DcMotor motorBackLeft;
  public DcMotor leftArm;
  public DcMotor rightArm;
  public DcMotor leftIntake;
  public DcMotor rightIntake;
  public Servo gripServo;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
    motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
    motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");
    motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
    leftArm  = hardwareMap.get(DcMotor.class, "leftArm");
    leftIntake  = hardwareMap.get(DcMotor.class, "leftIntake");
    rightArm  = hardwareMap.get(DcMotor.class, "rightArm");
    rightIntake = hardwareMap.get(DcMotor.class, "rightIntake"); 
    gripServo = hardwareMap.get(Servo.class, "gripServo");
    motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    leftArm.setDirection(DcMotor.Direction.REVERSE);
    leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
    // Put initialization blocks here.
    waitForStart();
    while (opModeIsActive()) {
      
      double y = gamepad1.left_stick_y;
      double x = -gamepad1.left_stick_x*1.1;
      double rx = gamepad1.right_stick_x;
      double ty = gamepad2.left_stick_y;
      double tx = -gamepad2.left_stick_x*1.1;
      double trx = gamepad2.right_stick_x;
      double denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)*1.5);
        
      if (ty != 0 || tx != 0 || trx != 0) {
        y = ty;
        x= tx;
        rx=trx;
        double a_number = 4;
        if (gamepad2.right_stick_button) {
          a_number = 1.5;
        }
         denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)*a_number);
      }
      double frontLeftPower = (y + x - rx) / (denominator);
      double backLeftPower = (y - x - rx) / (denominator);
      double frontRightPower = (y - x + rx) / (denominator);
      double backRightPower = (y + x + rx) / (denominator);
      motorFrontLeft.setPower(frontLeftPower);
      motorBackLeft.setPower(backLeftPower);
      motorFrontRight.setPower(frontRightPower);
      motorBackRight.setPower(backRightPower);
      if (gamepad2.left_stick_button){
        leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      }
      if (gamepad2.left_bumper){
        gripServo.setPosition(0.5);
        
      }
      if (gamepad2.right_bumper){
        gripServo.setPosition(0);
      }
      if (gamepad2.dpad_up){
        leftArm.setPower(0.3);
        rightArm.setPower(0.3);
      }
      else if (gamepad2.dpad_down){
        leftArm.setPower(-0.3);
        rightArm.setPower(-0.3);
      }
      else {
        leftArm.setPower(0.1);
        rightArm.setPower(0.1);
        
      }
      telemetry.addData("key", leftIntake.getCurrentPosition());
      telemetry.addData("powet", leftIntake.getPower());
      telemetry.update();
    }
  }
}
