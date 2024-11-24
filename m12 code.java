package org.firstinspires.ftc.teamcode.M12;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com. qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import android.app.Activity;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp()
public class AM12TeleOp extends LinearOpMode {
    
    //motor and servos
    DcMotorEx intake, Lextend, Rextend, arm, fl, fr, bl, br;
    NormalizedColorSensor colorSensor;
    AnalogInput ranger;
    Servo wrist, hooks;
    
    double strafemulti;
    double forwardmulti;
    double turnmulti;
    //targets for motors
    int armTarget = 0;
    int extendTarget = 0;
    
    boolean holdingTarget = false;
    boolean resetEncoder = false;
    boolean gameElement = false;
    boolean clipIntake = false;
    boolean hookOutAfterRelease = false;
    
    
    int intakeOverCurrentCycles = 0;
    

    @Override
    public void runOpMode() {
        
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        Lextend = hardwareMap.get(DcMotorEx.class, "Lelevator");
        Rextend = hardwareMap.get(DcMotorEx.class, "Relevator");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        ranger = hardwareMap.get(AnalogInput.class, "ranger");
        
        Lextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        intake.setPower(0);
        
        //reverse drivetrainmotors
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        
        arm.setTargetPosition(-700);
        arm.setPower(1);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(-700); //it doesn't crash when we keep this in so idk
        
        //Lextend.setDirection(DcMotorEx.Direction.REVERSE);
        //Rextend.setDirection(DcMotorEx.Direction.REVERSE)
        
        Lextend.setTargetPosition(0);
        Lextend.setPower(1);
        Lextend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Lextend.setTargetPosition(0);
        
        Rextend.setTargetPosition(0);
        Rextend.setPower(1);
        Rextend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Rextend.setTargetPosition(0);
        
        wrist = hardwareMap.get(Servo.class, "wrist");
        hooks = hardwareMap.get(Servo.class, "hooks");
        
        wrist.setPosition(0.5);
        hooks.setPosition(1);
        //0.1 is home
        //0.5 is deployed
    
        waitForStart();

        while(opModeIsActive()) {
            
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            
            gameElement = false;
            
            double intakePower = 0;
            
            //game element detection
            if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 2.0)
            {
                gameElement = true;
            }
            
            holdingTarget = false;
            
            if(gamepad1.a || gamepad2.a || (gameElement && armTarget > 200))
            {
                //HOME
                armTarget = 125;
                extendTarget = 630;
                wrist.setPosition(0.5);
            }
            
            if(((gamepad1.right_trigger > 0.05) || (gamepad2.right_trigger > 0.05)) && extendTarget < (int)(551*1.4))
            {
                //Intake Pre
                
                armTarget = 450;
                extendTarget = 760;
                wrist.setPosition(0.5);
            }
            
            if(gamepad1.y || gamepad2.y)
            {
                //Full Extend Up
                
                armTarget = -1900;
                extendTarget = 3500;
                wrist.setPosition(0.5);
                holdingTarget = true;
            }
            
            if(gamepad1.dpad_up || gamepad2.dpad_up)
            {
                //1
                extendTarget = (int)(1325*1.4);
                armTarget = -3200+500;
                wrist.setPosition(0.5);
                hooks.setPosition(1);
            }
            
            if(gamepad1.dpad_down || gamepad2.dpad_down)
            {
                //2
                extendTarget = 0;
                armTarget = -2500+500;
                hookOutAfterRelease = true;
            }
        
            
            if(gamepad1.dpad_left || gamepad2.dpad_left)
            {
                //3
                arm.setTargetPosition(-2700+500);
                Rextend.setTargetPosition((int)(200*1.4));
                Lextend.setTargetPosition((int)(-200*1.4));
                
                
                extendTarget = (int)(1900*1.4);
                armTarget = -2700+500;
                holdingTarget = true;
            }
            
            if(gamepad1.dpad_right || gamepad2.dpad_right)
            {
                //4
                arm.setTargetPosition(-3300+700);
                Rextend.setTargetPosition((int)(1900*1.4));
                Lextend.setTargetPosition((int)(-1900*1.4));
                
                
                extendTarget = 0;
                armTarget = -3300+500;
                holdingTarget = true;
            }
            
            if(gamepad1.guide || gamepad2.guide)
            {
                //4
                arm.setTargetPosition(-2300+500);
                Rextend.setTargetPosition(0);
                Lextend.setTargetPosition(0);
                
            
                extendTarget = 0;
                armTarget = -2300+500;
                holdingTarget = true;
                hooks.setPosition(1);
                hookOutAfterRelease = true;
            }
            
            if(gamepad1.b)
            {
                hooks.setPosition(0);
                
            }
            
            
            //Climbing Hooks
            if(hookOutAfterRelease && !(gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.guide || gamepad2.guide))
            {
                hooks.setPosition(0);
                hookOutAfterRelease = false;
            }
            
            
            
            //Manual Extend / Retract
            //Retract
            if(gamepad1.left_bumper || gamepad2.left_bumper)
            {
                extendTarget = extendTarget-(int)(80*1.4);
                
                armTarget = (int)(-((extendTarget)-760)/13) + 450;
                intakePower = 1;
            }
            
            //Extend
            if(gamepad1.right_bumper || gamepad2.right_bumper)
            {
                extendTarget = extendTarget+(int)(80*1.4);
                
                if(extendTarget > 2300){extendTarget = 2300;}
                armTarget = (int)(-((extendTarget)-760)/13) + 450;
                intakePower = -1;
            }
            
            
            if(extendTarget < 0){extendTarget = 0;}
            
            
          intakePower = -0.25;
            
            //Intake Code
            if((gamepad1.left_trigger > 0.05) || (gamepad2.left_trigger > 0.05))
            {
                wrist.setPosition(0.3);
                intakePower = -0.5;
            }
            
            if((gamepad1.right_trigger > 0.05) || (gamepad2.right_trigger > 0.05) || clipIntake)
            {   
                intakePower = -1;
                wrist.setPosition(0.5);
            }
            
            if(gamepad1.x || gamepad2.x)
            {
                intakePower = 0.6;
            }
            
            intake.setPower(intakePower);
            
            if(!holdingTarget)
            {
                arm.setTargetPosition(armTarget);
                Rextend.setTargetPosition(extendTarget);
                Lextend.setTargetPosition(-extendTarget);
            }
            
            
            strafemulti = 1;
            forwardmulti = 1;
            turnmulti = 1;
            
            //Drivetrain slower
            if (extendTarget > 2000 ){
                strafemulti = 0.5;
                forwardmulti = 0.5;
                turnmulti = 0.5;
            }
            
            
            //DRIVE CODE
            double leftx = -strafemulti*(gamepad1.left_stick_x); //strafe left and right
            double lefty = -forwardmulti*(gamepad1.left_stick_y + gamepad1.right_stick_y); //forward and backward
            double rightx = -turnmulti*(gamepad1.right_stick_x); //turn left and right
                
            fl.setPower(lefty - leftx - rightx);
            fr.setPower(lefty + leftx + rightx);
            bl.setPower(lefty + leftx - rightx);
            br.setPower(lefty - leftx + rightx);
            
            //Reset Encoders
            if(gamepad1.back)
            {
                Lextend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                Rextend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                //arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                
                Lextend.setPower(0.3);
                Rextend.setPower(-0.3);
                //arm.setPower(0.2);
                resetEncoder = true;
            }
            
            if(resetEncoder && !gamepad1.back)
            {
                Lextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Rextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
                Lextend.setTargetPosition(0);
                Rextend.setTargetPosition(0);
                extendTarget = 0;
                Lextend.setPower(1.0);
                Rextend.setPower(1.0);
                Lextend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                Rextend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                
                //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //arm.setTargetPosition(0);
                //armTarget = 0;
                //arm.setPower(1.0);
                //arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                resetEncoder = false;
            }
            
            
            if(true) {
                
                //telemetry.addData("Climbstate: ", climbState);
                telemetry.addData("Lextend Pos: ", Lextend.getCurrentPosition());
                telemetry.addData("Rextend Pos: ", Rextend.getCurrentPosition());
                telemetry.addData("fl", fl.getCurrentPosition());
                telemetry.addData("fl", fr.getCurrentPosition());
                telemetry.addData("fl", bl.getCurrentPosition());
                telemetry.addData("fl", br.getCurrentPosition());
                telemetry.addData("Arm Pos: ", arm.getCurrentPosition());
                telemetry.addData("Motor Current (Amps):", intake.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Raw Voltage",    ranger.getVoltage());
                telemetry.addData("br encoder", br.getCurrentPosition());
                telemetry.addData("fr encoder", fr.getCurrentPosition());
                telemetry.addData("bl encoder", bl.getCurrentPosition());
                telemetry.addData("fl encoder", fl.getCurrentPosition());

                telemetry.update();
            }
        }
    }
}
