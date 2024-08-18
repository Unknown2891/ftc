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
            leftstick = gamepad1.left_stick_y;
            rightstick = gamepad1.right_stick_y;

            motor2.setPower(leftstick);
            motor4.setPower(-leftstick);
            motor1.setPower(rightstick);
            motor3.setPower(-rightstick);

            telemetry.addData("Status", "Running");
            telemetry.addData("Left Stick:", leftstick);
            telemetry.addData("Right Stick:", rightstick);
            telemetry.update();

        }
    }
}
