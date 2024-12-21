package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "team1TeleOp", group = "LinearOpMode")

public class Team1 extends LinearOpMode {
    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor frontright = null;
    private DcMotor backright = null;
    

    @Override
    public void runOpMode() {
        
        frontleft = hardwareMap.get(DcMotor.class, "fl");
        frontright = hardwareMap.get(DcMotor.class, "fr");
        backleft = hardwareMap.get(DcMotor.class, "bl");
        backright = hardwareMap.get(DcMotor.class, "br");
        
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double drive, turn, strafe;
        double flpower, frpower, blpower, brpower;
        waitForStart();

        while (opModeIsActive()) {
            
            drive =-gamepad1.left_stick_y;
            strafe =gamepad1.left_stick_x;
            turn =gamepad1.right_stick_x;
            
            frpower = drive - turn - strafe;
            brpower = drive - turn + strafe;
            flpower = drive + turn + strafe;
            blpower = drive + turn - strafe;
            
            double maxPower = Math.max(+-Math.abs(frpower),Math.max(Math.abs(brpower),Math.max(Math.abs(flpower), Math.abs(blpower))));
            if(maxPower > 1){
                frpower /= maxPower;
                brpower /= maxPower;
                flpower /= maxPower;
                blpower /= maxPower;
            }
            
            frontright.setPower(frpower);
            backright.setPower(brpower);
            frontleft.setPower(flpower);
            backleft.setPower(blpower);
          
            telemetry.addData("Status", "Running");
            telemetry.addData("frpower", frpower);
            telemetry.addData("flpower", flpower);
            telemetry.addData("blpower", blpower);
            telemetry.addData("brpower", brpower);
            telemetry.update();

        }
    }
}
