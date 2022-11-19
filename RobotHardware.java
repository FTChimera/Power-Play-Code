package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {
    /* Public OpMode members. */
   public DcMotor motorFrontRight;
   public DcMotor motorFrontLeft;
   public DcMotor motorBackRight;
   public DcMotor motorBackLeft;
   public DcMotor leftArm;
   public DcMotor rightArm;
   public DcMotor leftIntake;
   public DcMotor rightIntake;
   

    /* local OpMode members. */
    HardwareMap hardwareMap =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map

        // Define and Initialize Motors
        motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        leftArm  = hardwareMap.get(DcMotor.class, "leftArm");
        leftIntake  = hardwareMap.get(DcMotor.class, "leftIntake");
        rightArm  = hardwareMap.get(DcMotor.class, "rightArm");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake"); 
        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // Define and initialize ALL installed servos.
    }
 }

