package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;

@TeleOp

public class Luqman extends LinearOpMode {
    
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    double kp = 0.02;
    ElapsedTime time = new ElapsedTime();
    double integralSum = 0;
    double ki = 0;
    double kd = 0;
    double previousPropError = 0;

    @Override
    public void runOpMode() {
        
        initialize();
        waitForStart();
        
        if (opModeIsActive()){
            while (opModeIsActive()){
                
                telemetry.addData("Status", "Initialized");
                telemetry.update();
                waitForStart();
                telemetry.addData("Status", "Running");
                telemetry.update();
            
                telemetryAprilTag();
            }
        }
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
    }
    
        public void initialize() {
        
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
    
        visionPortal = visionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        
        }
    
    private void telemetryAprilTag() {
        
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        
        for (AprilTagDetection detection : currentDetections) {
            
            if (detection.metadata != null) {
                
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                
            } else {
                
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                
            }
        }
        
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
    
    public double PIDcontroller(double TargetPoint, double PositionRobot) {
        
         double proportionError = TargetPoint - PositionRobot;
         proportionError *= kp; 
         return proportionError;
         
     }
}
