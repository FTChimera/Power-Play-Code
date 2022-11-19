package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "armtest (Blocks to Java)")
public class armtest extends LinearOpMode {

  RobotHardware chaos = new RobotHardware();

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

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
      chaos.motorFrontLeft.setPower(frontLeftPower);
      chaos.motorBackLeft.setPower(backLeftPower);
      chaos.motorFrontRight.setPower(frontRightPower);
      chaos.motorBackRight.setPower(backRightPower);
      telemetry.addData("key", chaos.leftIntake.getCurrentPosition());
      telemetry.update();
    }
  }
}
