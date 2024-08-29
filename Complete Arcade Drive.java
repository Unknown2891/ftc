package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class GroupProject extends LinearOpMode {

    DcMotor frontRight = null;
    DcMotor frontLeft = null;
    DcMotor backRight = null;
    DcMotor backLeft = null;
    double frontRightPower;
    double frontLeftPower;
    double backRightPower;
    double backLeftPower;
    double leftstick = 0;
    double rightstick = 0;
    double drive = 0;
    double turn = 0;
    double strafe = 0;
    
    @Override
    public void runOpMode() {

    frontRight = hardwareMap.get(DcMotor.class, "frontright");
    frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
    backRight = hardwareMap.get(DcMotor.class, "backright");
    backLeft = hardwareMap.get(DcMotor.class, "backleft");
    
    frontRight.setDirection(DcMotor.Direction.FORWARD);
    frontLeft.setDirection(DcMotor.Direction.FORWARD);
    backRight.setDirection(DcMotor.Direction.REVERSE);
    backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            frontRightPower = (strafe - drive + turn);
            frontLeftPower = (strafe + drive + turn);
            backRightPower = (-strafe - drive + turn);
            backLeftPower = (-strafe + drive + turn);
            
            frontRight.setPower(frontRightPower);
            frontLeft.setPower(frontLeftPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(backLeftPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Stick:", leftstick);
            telemetry.addData("Right Stick:", rightstick);
            telemetry.update();

        }
    }
}

    
