/*
Copyright 2024 FIRST Tech Challenge Team FTC

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="testmode")

public class Opmode extends LinearOpMode {

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    Servo servo1 = null;
    ColorSensor colorsensor = null;
    
    @Override
    public void runOpMode() {

        motor1 = hardwareMap.get(DcMotor.class, "backleft");
        motor2 = hardwareMap.get(DcMotor.class, "frontleft");
        motor3 = hardwareMap.get(DcMotor.class, "frontright");
        motor4 = hardwareMap.get(DcMotor.class, "backright");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motor1.setTargetPosition(1000);
        motor2.setTargetPosition(-1000);
        motor3.setTargetPosition(1000);
        motor4.setTargetPosition(-1000);
        
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(0.5);
        motor2.setPower(0.5);
        motor3.setPower(0.5);
        motor4.setPower(0.5);

        servo1.setPosition(1);
        
