package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "Luqman(tank.java)")

public class Luqman extends LinearOpMode {

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    double power1 = 0;
    double power2 = 0;
    double power3 = 0;
    double power4 = 0;
    double leftstick = 0;
    double rightstick = 0;

    @Override
    public void runOpMode() {

    motor1 = hardwareMap.get(DcMotor.class, "frontright");
    motor2 = hardwareMap.get(DcMotor.class, "frontleft");
    motor3 = hardwareMap.get(DcMotor.class, "backright");
    motor4 = hardwareMap.get(DcMotor.class, "backleft");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
           
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            
            motor1.setDirection(DcMotor.Direction.REVERSE);
            motor2.setDirection(DcMotor.Direction.FORWARD);
            motor3.setDirection(DcMotor.Direction.REVERSE);
            motor4.setDirection(DcMotor.Direction.FORWARD);

            power1 = drive + turn;
            power2 = drive - turn;
            power3 = drive + turn;
            power4 = drive - turn;
            
            motor1.setPower(power1);    
            motor2.setPower(power2);
            motor3.setPower(power3);
            motor4.setPower(power4);

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Stick:", leftstick);
            telemetry.addData("Right Stick:", rightstick);
            telemetry.update();

        }
    }
}
