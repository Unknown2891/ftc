
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import java.util.Map;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous

public class Luqman extends LinearOpMode {

  DcMotor motor1 = null;
  DcMotor motor2 = null;
  DcMotor motor3 = null;
  DcMotor motor4 = null;
  double kp = 0.02;
  IMU imu;
  ElapsedTime time = new ElapsedTime();
  double integralSum = 0;
  double ki = 0;
  double kd = 0;
  double previousPropError = 0;
   
  @Override
  public void runOpMode() {
    
    imu = hardwareMap.get(IMU.class, "imu");
    
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

      imu.initialize(new IMU.Parameters(orientationOnRobot));

      motor1 = hardwareMap.get(DcMotor.class, "frontRight");
      motor2 = hardwareMap.get(DcMotor.class, "frontLeft");
      motor3 = hardwareMap.get(DcMotor.class, "midRight");
      motor4 = hardwareMap.get(DcMotor.class, "midLeft");
 
      motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
      motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
      motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
      motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);

      motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       
      motor1.setDirection(DcMotor.Direction.REVERSE);
      motor2.setDirection(DcMotor.Direction.REVERSE);
       
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
     

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
           
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();

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
