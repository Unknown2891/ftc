package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp

public class ServoTelyop extends LinearOpMode {
    
     Servo servo = null;
     
    @Override
    public void runOpMode() {
        
        servo = hardwareMap.get(Servo.class,"wrist");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            
            if(gamepad1.y){
                servo.setPosition(0.2);
            }
            else if(gamepad1.b){
                servo.setPosition(0.5);
            }

                
        }
            telemetry.addData("Status", "Running");
            telemetry.update();
    }
}
