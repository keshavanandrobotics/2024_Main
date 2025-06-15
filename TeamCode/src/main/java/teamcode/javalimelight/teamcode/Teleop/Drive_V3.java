package teamcode.javalimelight.teamcode.Teleop;

//import static teamcode.Autonomous.Poses.AUTON_END_POSE;
import static java.lang.Math.abs;
import static teamcode.Teleop.Singletons.VARS.AUTON_RED;
import static teamcode.Teleop.Singletons.VARS.CLAW_CLOSED;
import static teamcode.Teleop.Singletons.VARS.CLAW_LESS_OPEN;
import static teamcode.Teleop.Singletons.VARS.CLAW_OPEN;
import static teamcode.Teleop.Singletons.VARS.EXTENDO_MAX_TELE;
import static teamcode.Teleop.Singletons.VARS.HIGH_SAMPLE_POS;
import static teamcode.Teleop.Singletons.VARS.HIGH_SPECIMEN_POS_TELE;
import static teamcode.Teleop.Singletons.VARS.MOVE_ALL_OUT;
import static teamcode.Teleop.Singletons.VARS.MOVE_HOVER_SAMPLE;
import static teamcode.Teleop.Singletons.VARS.MOVE_OUTTAKE;
import static teamcode.Teleop.Singletons.VARS.MOVE_PICKUP_SAMPLE;
import static teamcode.Teleop.Singletons.VARS.MOVE_SPECIMEN_SCORE;
import static teamcode.Teleop.Singletons.VARS.MOVE_WALL_INTAKE;
import static teamcode.Teleop.Singletons.VARS.PIVOT_ALL_OUT;
import static teamcode.Teleop.Singletons.VARS.PIVOT_OUTTAKE;
import static teamcode.Teleop.Singletons.VARS.PIVOT_SAMPLE_PICKUP;
import static teamcode.Teleop.Singletons.VARS.PIVOT_SPECIMEN_SCORE;
import static teamcode.Teleop.Singletons.VARS.PIVOT_WALL_INTAKE;
import static teamcode.Teleop.Singletons.VARS.ROTATE_FLIP;
import static teamcode.Teleop.Singletons.VARS.ROTATE_NEUTRAL;
import static teamcode.Teleop.Singletons.VARS.SPEC_MODE;
import static teamcode.Teleop.Singletons.VARS.USING_LIMELIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import java.util.Objects;

//import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.javalimelight.Vision.LL_Tracker;
import teamcode.Robot;
import teamcode.Teleop.Singletons.MotorWeights;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import teamcode.javalimelight.trclib.pathdrive.TrcPose2D;
import teamcode.javalimelight.trclib.pathdrive.TrcPose3D;

//@Config
@TeleOp

public class Drive_V3 extends LinearOpMode{
    private static MultipleTelemetry TELE;

    public static String MODE = "TESTBOT1";




    private PIDController controller;
    public static double p = 0.0006, i = 0, d = 0.00001;

    public static int target = 30000;





    public double linearSlideZeroPosition = 0;
    public double extendoZeroPosition = 0;
    public boolean linearAutomation = false;
    public boolean startPressToggle = false;
    public boolean maximumExtension = false;
    public boolean dpadDownToggle = false;

    public boolean dpadDownToggle2 = false;
    public boolean dpadUpToggle = false;
    public boolean backPressToggle = false;
    public boolean dpadDownServoLock = false;



    public double slidesZeroPower = 0.08;
    public boolean slideResetToggle = false;
    public double slideResetTimestamp = 0.0;

    public double bPressTimestamp = 0.0;
    public double yTimestamp = 0.0;
    public boolean yToggle = false;
    public double startPressTimestamp = 0.0;
    public double dpadDownTimestamp = 0.0;
    public double dpadUpTimestamp = 0.0;
    public double backPressTimestamp = 0.0;


    public double offset = 0.0;
    public double botHeading = 0.0;

    public double rightBumperPressTimestamp = 0.0;


    public boolean extendoIn = false;

    public boolean useColorSensor = true;

    public boolean pickupSample = false;

    public double colorSensorTimer = 0.0;
    public boolean extendoOut = false;
    public boolean extendoHoldIn = false;

    public boolean extendoHoldOut = false;
    public boolean PID_MODE = false;


    public double angle =0;




    Robot robot;
    LL_Tracker llTracker;

    // Initialize Classes used for Robot Control
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

        controller = new PIDController(p, i, d);

        // Gamepad Controls
        GamepadEx g2 = new GamepadEx(gamepad2);
        GamepadEx g1 = new GamepadEx(gamepad1);

        ButtonReader G1_START_PRESS = new ButtonReader(
                g1, GamepadKeys.Button.START
        );

        ButtonReader G1A = new ButtonReader(
                g1, GamepadKeys.Button.A
        );

        ButtonReader G1Y = new ButtonReader(
                g1, GamepadKeys.Button.Y
        );

        ButtonReader G1_DPAD_UP = new ButtonReader(
                g1, GamepadKeys.Button.DPAD_UP
        );

        ButtonReader G1_DPAD_DOWN = new ButtonReader(
                g1, GamepadKeys.Button.DPAD_DOWN
        );




        ButtonReader B_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.B
        );

        ButtonReader Y_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.Y
        );

        ButtonReader X_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.X
        );


        ButtonReader RIGHT_BUMPER_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.RIGHT_BUMPER
        );

        ButtonReader START_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.START
        );

        ButtonReader BACK_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.BACK
        );

        ButtonReader DPAD_DOWN_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.DPAD_DOWN
        );

        ButtonReader DPAD_UP_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.DPAD_UP
        );

        // Limelight Init
//        robot.limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        robot.limelight.pipelineSwitch(9);
//        robot.limelight.start();
        llTracker = new LL_Tracker();
        llTracker.Init(robot, hardwareMap, TELE);
    }

    private void controlDrivetrain () {
        //DRIVETRAIN:

        double rx = 0;
        double x = 0;
        double y = 0;

        double rotX = 0;
        double rotY = 0;

        //botHeading = robot.drive.pose.heading.toDouble() - offset;

        //TELE.addData("botHeading", Math.toDegrees(botHeading));

        //TELE.addData("offset", Math.toDegrees(offset));

        if (Objects.equals(MODE, "TESTBOT1")){

            rx = gamepad1.left_stick_x;
            x = gamepad1.right_stick_x;
            y = gamepad1.right_stick_y;

        } else if (Objects.equals(MODE, "!TESTBOT1")){
            rx = gamepad1.left_stick_x;
            x = gamepad1.right_stick_x;
            y = -gamepad1.right_stick_y;
        }else if (Objects.equals(MODE, "STEVE")){
            rx = gamepad1.right_stick_x;
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
        } else if (Objects.equals(MODE, "FC")) {
            rx = gamepad1.left_stick_x;
            x = gamepad1.right_stick_x;
            y = -gamepad1.right_stick_y;


            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            x = rotX;
            y = rotY;

        }
        else if (Objects.equals(MODE, "FC2")) {
            rx = gamepad1.right_stick_x;
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;

            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            x = rotX;
            y = rotY;
        }


        double slowFactor = 1 - gamepad1.left_trigger*0.8;

        rx*=slowFactor;
        x*=slowFactor;
        y*=slowFactor;


        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = -(y + x + rx) / denominator;
        double backLeftPower = -(y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower *= MotorWeights.FRONT_LEFT;
        frontRightPower *= MotorWeights.FRONT_RIGHT;
        backLeftPower *= MotorWeights.BACK_LEFT;
        backRightPower *= MotorWeights.BACK_RIGHT;


        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backRightMotor.setPower(backRightPower);
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
        if ((llResult.getPythonOutput()[0]==1) && (abs(targetTx) > TARGET_POSITION_TOLERANCE))
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

        if (!AUTON_RED){
            useColorSensor= false;
        }

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
