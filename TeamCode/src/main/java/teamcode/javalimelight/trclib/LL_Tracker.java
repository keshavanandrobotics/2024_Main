// Technical Turbulence
// Limelight Tracker version 0.1
// 2025-05-26

package teamcode.javalimelight.trclib;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import static java.lang.Math.abs;

import teamcode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.security.spec.ECField;
import java.util.List;

import teamcode.javalimelight.trclib.pathdrive.TrcPose2D;
import teamcode.javalimelight.trclib.pathdrive.TrcPose3D;

public class LL_Tracker {

    private final double MAX_SEARCH_TIME = 2.0; // seconds
    // Moves passenger side or right during search
    // Positive is driver's side or left
    // Negative is passenger's side or right
    private final double STRAFE_SEARCH_SPEED = -0.65;

    private Robot robot;
    private static MultipleTelemetry TELE;
    private PIDController strafePID, backupPID;
    public final double S_PID_P = 0.5, S_PID_I = 0.1, S_PID_D = 0.05;
    public final double B_PID_P = 0.5, B_PID_I = 0.1, B_PID_D = 0.05;

    private final boolean STRAFE_DEBUG_TEXT = false;
    private final boolean BACKUP_DEBUG_TEXT = false;

    public LL_Tracker () {

    }

    // Initialize Limelight
    // Returns False if limelight not found
    public boolean Init (Robot myRobot, HardwareMap myHardwareMap, MultipleTelemetry myTele) {
        boolean initStatus = true;

        robot = myRobot;
        TELE = myTele;
        strafePID = new PIDController(S_PID_P, S_PID_I, S_PID_D);
        backupPID = new PIDController(B_PID_P, B_PID_I, B_PID_D);


        try {
            // Limelight Init
            robot.limelight = myHardwareMap.get(Limelight3A.class, "limelight");
            TELE.setMsTransmissionInterval(11);
            robot.limelight.pipelineSwitch(9);
            robot.limelight.start();
        }
        catch (Exception e) {
            initStatus = false;
            TELE.addLine("LimeLight failed to init.");
        }

        TELE.update();
        return initStatus;
    }

    // Turns Off Limielight Processing
    public boolean Stop () {
        boolean stopStatus = true;

        try {
            robot.limelight.stop();
        }
        catch (Exception e) {
            stopStatus = false;
            TELE.addLine("LimeLight failed to init.");
        }

        TELE.update();
        return stopStatus;
    }

    // Test procedure to return all results
    public void getLimelightResults () {
        TrcPose3D cameraPose = new TrcPose3D(0.0,0.0,10.0, 0.0, 90.0, 0.0);

        LLStatus status = robot.limelight.getStatus();
        TELE.addData("Name", "%s",
                status.getName());
        TELE.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        TELE.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = robot.limelight.getLatestResult();
        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            TELE.addData("LL Latency", captureLatency + targetingLatency);
            TELE.addData("Parse Latency", parseLatency);
            TELE.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
            TELE.addData("tx", result.getTx());

            if (result.isValid()) {
                TELE.addData("tx", result.getTx());
                TELE.addData("txnc", result.getTxNC());
                TELE.addData("ty", result.getTy());
                TELE.addData("tync", result.getTyNC());

                TELE.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    TELE.addData("Barcode", "Data: %s", br.getData());
                }

                // Access classifier results
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    TELE.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    TELE.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    TELE.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    TELE.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }

        } else {
            TELE.addData("Limelight", "Nodata available");
        }

        //getTargetPose(cameraPose, result);

        TELE.update();
    }

    // Simple procedure to drive the motors in a strafe pattern
    // Positive moves drivers side or left
    // Negative moves passengers side or right
    private void strafeMotors (double power) {
        robot.frontLeftMotor.setPower(power);
        robot.backLeftMotor.setPower(-power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(-power);
    }

    // Drive forward or backwards
    private void forwardMotors (double power) {
        robot.frontLeftMotor.setPower(-power);
        robot.backLeftMotor.setPower(-power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(power);
    }

    // Stop all Motor Movement
    private void stopMotors () {
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    private boolean strafeAlign (LLResult llResult) {
        boolean aligned = false;
        double targetTx = llResult.getTx();
        double drivePower = 0.0;
        final double MIN_DRIVE_POWER = 0.15;
        final double TARGET_POSITION_TOLERANCE = 5.0;
        final double HORIZONTAL_FOV_RANGE = 26.0;
        final double DRIVE_POWER_REDUCTION = 2.0;

        if (abs(targetTx) < TARGET_POSITION_TOLERANCE) {
            aligned = true;
        }

        // Only with valid data and if too far off target
        if ((llResult.getPythonOutput()[0]==1) && !aligned)
        {

            // Adjust Robot Speed based on how far the target is located
            // Only drive at half speed max
            // switched to PID but original formula left for reference in comments
            //drivePower = targetTx/HORIZONTAL_FOV_RANGE / DRIVE_POWER_REDUCTION;
            drivePower = -(strafePID.calculate(targetTx/HORIZONTAL_FOV_RANGE, 0.0));

            // Make sure we have enough power to actually drive the wheels
            if (abs(drivePower) < MIN_DRIVE_POWER) {
                if (drivePower > 0.0) {
                    drivePower = MIN_DRIVE_POWER;
                } else {
                    drivePower = -MIN_DRIVE_POWER;
                }

            }
            if (STRAFE_DEBUG_TEXT) {
                TELE.addData("drivePower: ", drivePower);
            }

            // Move Left or Right until centered on target
            strafeMotors(drivePower);
        }
        else {
            // We are done
            if (STRAFE_DEBUG_TEXT) {
                TELE.addData("drivePower", "No Target or Movement Needed");
            }
            stopMotors();
        }
        if (STRAFE_DEBUG_TEXT) {
            TELE.update();
        }

        return aligned;
    }


    private boolean backupAlign (LLResult llResult) {
        boolean aligned = false;
        double targetTy = llResult.getTy();
        double drivePower = 0.0;
        final double MIN_DRIVE_POWER = 0.15;
        final double TARGET_POSITION_TOLERANCE = 5.0;
        final double HORIZONTAL_FOV_RANGE = 26.0;
        final double DRIVE_POWER_REDUCTION = 2.0;

        // TBD: Remove ABS to only go backwards.
        if (abs(targetTy) < TARGET_POSITION_TOLERANCE) {
            aligned = true;
        }

        // Only with valid data and if too far off target
        if ((llResult.getPythonOutput()[0]==1) && !aligned)
        {

            // Adjust Robot Speed based on how far the target is located
            // Only drive at half speed max
            // switched to PID but original formula left for reference in comments
            //drivePower = targetTx/HORIZONTAL_FOV_RANGE / DRIVE_POWER_REDUCTION;
            drivePower = strafePID.calculate(targetTy/HORIZONTAL_FOV_RANGE, 0.0);

            // Make sure we have enough power to actually drive the wheels
            if (abs(drivePower) < MIN_DRIVE_POWER) {
                if (drivePower > 0.0) {
                    drivePower = MIN_DRIVE_POWER;
                } else {
                    drivePower = -MIN_DRIVE_POWER;
                }

            }
            if (BACKUP_DEBUG_TEXT) {
                TELE.addData("backupdrivePower: ", drivePower);
            }

            // Move Left or Right until centered on target
            forwardMotors(drivePower);
        }
        else {
            // We are done
            if (BACKUP_DEBUG_TEXT) {
                TELE.addData("backupdrivePower", "No Target or Movement Needed");
            }
            stopMotors();
        }
        if (BACKUP_DEBUG_TEXT) {
            TELE.update();
        }

        return aligned;
    }


    // Strafe sideways until target located
    // Lock on target and adjust side to side to center laterally
    // then move front to back to center in longitude
    public boolean Track () {
        boolean strafeAligned = false;
        boolean backupAligned = false;

        LLStatus status = robot.limelight.getStatus();
        TELE.addData("Name", "%s",
                status.getName());
        TELE.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        TELE.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = robot.limelight.getLatestResult();
        TELE.addData("tx", result.getTx());
        TELE.addData("ty", result.getTy());

        if (result.getPythonOutput()[0] == 0.0) {
            // No Target so slow strafe right to look


            strafeMotors(STRAFE_SEARCH_SPEED);
        }
        else {
            // Target Found, perform fine alignment
            // This algorithm allows the motors to bounce back and forth
            // between strafe align and front to back align.
            // Other implementations could use a state machine if we didn't want
            // continued tracking in both directions or we could combine both
            // movements to try to speed this up.
            // This algorithm was chosen for simplicity and ease of debugging.

            // Side to Side Tracking
            strafeAligned = strafeAlign(result);

            // Front to Back Tracking
            // So the thought is to have two different procedure for front
            // and back due to sing different mechanisms for movement.
            // Right now backupAlign is doing both.

            // Move back with wheel motors
            if (strafeAligned) {
                backupAligned = backupAlign(result);
            }
            // Move forward with crane
            // TBD procedure with slight algorithm change to backup to
            // make it only check for movement needed in the backwards direction.
        }

        return strafeAligned && backupAligned;

    }

    // Get the sample angle after alignment 
    public double getSampleAngle () {
        LLResult result = robot.limelight.getLatestResult();
        double sampleAngle = 0.0;

        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            sampleAngle = pythonOutputs[3];
        }

        return sampleAngle;
    }

}
