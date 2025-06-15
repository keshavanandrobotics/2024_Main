package teamcode.javalimelight.trclib;

import static java.lang.Math.*;

import static teamcode.Teleop.Singletons.VARS.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Robot;
import teamcode.javalimelight.trclib.LL_Tracker;
import teamcode.javalimelight.trclib.pathdrive.TrcPose2D;
import teamcode.javalimelight.trclib.pathdrive.TrcPose3D;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


@Config
@TeleOp
public class Limelight_Test extends LinearOpMode {
    private MultipleTelemetry TELE;

    Robot robot;

    LL_Tracker llTracker;
    private void Initialize () {

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Base Robot must be initialized first.
        robot = new Robot(hardwareMap);

        //robot.drive = new PinpointDrive(hardwareMap, AUTON_END_POSE);

        // Motors
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.clawPivot.setPosition(PIVOT_ALL_OUT);
        robot.clawLeftMove.setPosition(MOVE_ALL_OUT);
        robot.clawRightMove.setPosition(1-MOVE_ALL_OUT);


        // Limelight Init
//        robot.limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        robot.limelight.pipelineSwitch(9);
//        robot.limelight.start();
        llTracker = new LL_Tracker();
        llTracker.Init(robot, hardwareMap, TELE);
    }
    private void slowstrafeRight (double power) {
        robot.frontLeftMotor.setPower(power);
        robot.backLeftMotor.setPower(-power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(-power);
    }

    private void slowstrafeLeft (double power) {
        robot.frontLeftMotor.setPower(-power);
        robot.backLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(-power);
        robot.backRightMotor.setPower(power);
    }

    private void stopstrafe () {
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    private void strafeAlign (LLResult llResult) {
        double targetTx = llResult.getTx();
        double drivePower = 0.0;
        final double MIN_DRIVE_POWER = 0.15;
        final double TARGET_POSITION_TOLERANCE = 3.0;
        final double HORIZONTAL_FOV_RANGE = 26.0;
        final double DRIVE_POWER_REDUCTION = 2.0;

        // Only with valid data and if too far off target
        if ((llResult.getPythonOutput()[0]==1) && (Math.abs(targetTx) > TARGET_POSITION_TOLERANCE))
        {

            // Adjust Robot Speed based on how far the target is located
            // Only drive at half speed max
            drivePower = targetTx/HORIZONTAL_FOV_RANGE / DRIVE_POWER_REDUCTION;

            // Make sure we have enough power to actually drive the wheels
            if (abs(drivePower) < MIN_DRIVE_POWER) {
                if (drivePower > 0.0) {
                    drivePower = MIN_DRIVE_POWER;
                } else {
                    drivePower = -MIN_DRIVE_POWER;
                }

            }
            telemetry.addData("drivePower: ", drivePower);

            if (drivePower > 0.0) {
                slowstrafeLeft(abs(drivePower));
            } else
            {
                slowstrafeRight(abs(drivePower));
            }

        }
        else {
            telemetry.addData("drivePower", "No Target or Movement Needed");
            stopstrafe();
        }
        telemetry.update();
    }

    private TrcPose2D getTargetPose(TrcPose3D cameraPose, LLResult llResult )
    {
        TrcPose2D targetPose;
        double camPitchRadians = Math.toRadians(cameraPose.pitch);
        double targetPitchDegrees = llResult.getTy();
        double targetYawDegrees = llResult.getTx();
        double targetPitchRadians = Math.toRadians(targetPitchDegrees);
        double targetYawRadians = Math.toRadians(targetYawDegrees);
        double groundOffset = 0; //targetGroundOffset.getOffset(resultType);

        double targetDepth =
                (groundOffset - cameraPose.z) / Math.tan(camPitchRadians + targetPitchRadians);
        targetPose = new TrcPose2D(
                targetDepth * Math.sin(targetYawRadians), targetDepth * Math.cos(targetYawRadians), targetYawDegrees);
        telemetry.addData(
                "LimelightObject." + "Python",
                "groundOffset=%.1f, cameraZ=%.1f, camPitch=%.1f, targetPitch=%.1f, targetDepth=%.1f, targetYaw=%.1f, " +
                        "targetPose=%s",
                groundOffset, cameraPose.z, cameraPose.pitch, targetPitchDegrees, targetDepth, targetYawDegrees,
                targetPose);
        telemetry.update();
        return targetPose;
    }   //getTargetPose

    private void getLimelightResults () {
        TrcPose3D cameraPose = new TrcPose3D(0.0,0.0,10.0, 0.0, 90.0, 0.0);

        LLStatus status = robot.limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = robot.limelight.getLatestResult();
        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
            telemetry.addData("tx", result.getTx());

            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // Access classifier results
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
            if (result.getPythonOutput()[0] == 0.0) {
                // No Target so slow strafe right to look
                slowstrafeRight(0.3);
            }
            else {
                // Target Found, perform fine alignment
                strafeAlign(result);
            }

        } else {
            telemetry.addData("Limelight", "Nodata available");
        }

        //getTargetPose(cameraPose, result);

        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Initialize();

        waitForStart();

        if (isStopRequested()) return;

        // Main Drive Loop runs until signaled to stop by Driver HUB
        while (opModeIsActive()) {

            //controlDrivetrain();
            //getLimelightResults();
            llTracker.Track();


            //TELEMETRY:

            //TELE.addData("Linear Slide Position", linearSlidePosition);
            //TELE.addData("Linear Slide Zero Position", linearSlideZeroPosition);


            //TELE.addData("Extendo Position", robot.extendo.getCurrentPosition());
            //TELE.addData("Extendo Zero Position", extendoZeroPosition);

            //TELE.addData("x", robot.drive.pose.position.x);
            //TELE.addData("y", robot.drive.pose.position.y);

            //TELE.addData("heading", Math.toDegrees(robot.drive.pose.heading.toDouble()));

            //TELE.addData("pin0", robot.pin0.getState());
            //TELE.addData("pin1", robot.pin1.getState());



            //robot.drive.updatePoseEstimate();



            TELE.update();




        }
        llTracker.Stop();
        //robot.limelight.stop();

    }
}
