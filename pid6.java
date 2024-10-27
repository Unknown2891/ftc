
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.Map;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous

public class Luqman extends LinearOpMode {

   DcMotor motor1 = null;
   DcMotor motor2 = null;
   DcMotor motor3 = null;
   DcMotor motor4 = null;
   double kp = 0.02;
   private BNO055IMU imu;
   ElapsedTime time = new ElapsedTime();
   double integralSum = 0;
   double ki = 0;
   double kd = 0;
   double previousPropError = 0;
   
   @Override
   public void runOpMode() {

       motor1 = hardwareMap.get(DcMotor.class, "frontright");
       motor2 = hardwareMap.get(DcMotor.class, "frontleft");
       motor3 = hardwareMap.get(DcMotor.class, "backright");
       motor4 = hardwareMap.get(DcMotor.class, "backleft");
       imu = hardware.get(BNO055IMU.class, "imu");
 
       motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
       motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
       motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
       motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);

       motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       
       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(
        
           new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
                RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);

                ));
     
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        
       parameters.mode = BNO055IMU.SensorMode.IMU;
       parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
       imu.initialize(parameters);
       
       YawPitchRollAngles robotOrientation;
       robotOrientation = imu.getRobotYawPitchRollAngles();

       motor1.setDirection(DcMotor.Direction.REVERSE);
       motor2.setDirection(DcMotor.Direction.REVERSE);

       telemetry.addData("Status", "Initialized");
       telemetry.update();
       waitForStart();

       while (opModeIsActive()) {

           double motor1power = PIDcontroller(1000, motor1.getCurrentPosition());
           double motor2power = PIDcontroller(1000, motor2.getCurrentPosition());
           double motor3power = PIDcontroller(1000, motor3.getCurrentPosition());
           double motor4power = PIDcontroller(1000, motor4.getCurrentPosition());

           motor1.setPower(motor1power);
           motor2.setPower(motor2power);
           motor3.setPower(motor3power);
           motor4.setPower(motor4power);

           telemetry.addData("motor1 error", motor1power);
           telemetry.addData("motor2 error", motor2power);
           telemetry.addData("motor3 error", motor3power);
           telemetry.addData("motor4 error", motor4power);
           
             telemetry.update();

       }
   }
  
   public double PIDcontroller(double TargetPoint, double PositionRobot) {
       double proportionError = TargetPoint - PositionRobot;
       integralSum  += proportionError * time.seconds();
       integralSum *= ki;
       proportionError *= kp;
       double derivativeError = (proportionError - previousPropError)/time.time();
       time.reset();
       derivativeError *= kd;
       double output = proportionError + integralSum + derivativeError;
       return output;
   }
  
  public double anglewrap(double radians) {
    while (radians > 180) {
        radians -= 360;
      }
   while (radians < -180) {
        radians += 360;
      }
      return radians;
  }

 }
