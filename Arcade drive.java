package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class GroupProject extends LinearOpMode {

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    double motor1power;
    double motor2power;
    double motor3power;
    double motor4power;
    double leftstick = 0;
    double rightstick = 0;
    double drive = 0;
    double turn =0;
    
    @Override
    public void runOpMode() {

    motor1 = hardwareMap.get(DcMotor.class, "frontright");
    motor2 = hardwareMap.get(DcMotor.class, "frontleft");
    motor3 = hardwareMap.get(DcMotor.class, "backright");
    motor4 = hardwareMap.get(DcMotor.class, "backleft");
    
    motor1.setDirection(DcMotor.Direction.REVERSE);
    motor2.setDirection(DcMotor.Direction.FORWARD);
    motor3.setDirection(DcMotor.Direction.REVERSE);
    motor4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            
            motor1power = (drive + turn);
            motor2power = (drive - turn);
            motor3power = (drive + turn);
            motor4power = (drive - turn);

            motor2.setPower(leftstick);
            motor4.setPower(-leftstick);
            motor1.setPower(rightstick);
            motor3.setPower(-rightstick);
            
            motor1.setPower(motor1power);
            motor2.setPower(motor2power);
            motor3.setPower(motor3power);
            motor4.setPower(motor4power);

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Stick:", leftstick);
            telemetry.addData("Right Stick:", rightstick);
            telemetry.update();

        }
    }
}
    
