package teamcode.Teleop;

import static teamcode.Autonomous.Disabled.Poses.AUTON_END_POSE;
import static teamcode.Teleop.Singletons.VARS.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Objects;

import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;
import teamcode.Teleop.Singletons.MotorWeights;


@Config
@TeleOp

public class Drive_V3 extends LinearOpMode{
    private static MultipleTelemetry TELE;

    public static String MODE = "SAM";





    private PIDController controller;

    public static double p = 0.0003, i = 0, d = 0.00001;

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
    public boolean backReleased = false;
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
    public double limitSwitchTimestamp = 0.0;
    public boolean limitSwitchOff = false;

    public boolean xToggle = false;

    public double xTimeStamp = 0.0;


    public double offset = 0.0;
    public double botHeading = 0.0;

    public double rightBumperPressTimestamp = 0.0;


    public boolean extendoIn = false;

    public boolean useColorSensor = true;

    public boolean pickupSample = false;
    public boolean specIn = false;
    public boolean holdSpec = false;
    public boolean magneticSwitch = false;

    public double colorSensorTimer = 0.0;
    public boolean extendoOut = false;
    public boolean extendoHoldIn = false;

    public boolean extendoHoldOut = false;
    public boolean PID_MODE = false;

    public boolean G1B = false;

    public double G1BTime = 0.0;


    public double angle =0;
    public boolean magneticSwitchHang = false;
    public boolean HANG_1_TARGET = true;
    public boolean HANG_2_TARGET = true;
    public boolean HANG_3_TARGET = true;
    public boolean HANG_4_TARGET = true;
    public boolean EXTENDO_HANG_TARGET = true;




    Robot robot;




    @Override
    public void runOpMode() throws InterruptedException {

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);

        robot.drive = new PinpointDrive(hardwareMap, AUTON_END_POSE);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        robot.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(USING_LIMELIGHT){
            robot.limelight.setPollRateHz(100);
            robot.limelight.start();
            robot.limelight.pipelineSwitch(0);

            MOVE_HOVER_SAMPLE = 0.53;

        }







        controller = new PIDController(p, i, d);

        GamepadEx g2 = new GamepadEx(gamepad2);
        GamepadEx g1 = new GamepadEx(gamepad1);

        ButtonReader G1_START_PRESS = new ButtonReader(
                g1, GamepadKeys.Button.START
        );


        ButtonReader G1_B = new ButtonReader(
                g1, GamepadKeys.Button.B
        );


        ButtonReader G1_DPAD_UP = new ButtonReader(
                g1, GamepadKeys.Button.DPAD_UP
        );

        ButtonReader G1_DPAD_DOWN = new ButtonReader(
                g1, GamepadKeys.Button.DPAD_DOWN
        );

        ButtonReader G1_LEFT_BUMPER = new ButtonReader(
                g1, GamepadKeys.Button.LEFT_BUMPER
        );

        ButtonReader G1_RIGHT_BUMPER  = new ButtonReader(
                g1, GamepadKeys.Button.RIGHT_BUMPER
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

        if (!AUTON_RED){
            useColorSensor= false;
        }



        robot.rightStabilizer.setPosition(RIGHT_HOLD_OFF);
        robot.leftStabilizer.setPosition(LEFT_HOLD_OFF);


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            //DRIVETRAIN:

            double rx = 0;
            double x = 0;
            double y = 0;

            double rotX = 0;
            double rotY = 0;


            botHeading = robot.drive.pose.heading.toDouble() - offset;


            TELE.addData("botHeading", Math.toDegrees(botHeading));

            TELE.addData("offset", Math.toDegrees(offset));


            if (Objects.equals(MODE, "SAM")) {

                rx = gamepad1.left_stick_x;
                x = -gamepad1.right_stick_x;
                y = gamepad1.right_stick_y;

            } else if (Objects.equals(MODE, "!SAM")) {
                rx = gamepad1.left_stick_x;
                x = gamepad1.right_stick_x;
                y = -gamepad1.right_stick_y;
            } else if (Objects.equals(MODE, "STEVE")) {
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


            } else if (Objects.equals(MODE, "FC2")) {
                rx = gamepad1.right_stick_x;
                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;


                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                x = rotX;
                y = rotY;


            }


            double slowFactor = 1 - gamepad1.left_trigger * SLOWDOWN_SPEED;

            rx *= slowFactor;
            x *= slowFactor;
            y *= slowFactor;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
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

            //G1 CONTROLS


            if (gamepad1.back) {

                useColorSensor = false;
            }

            if (G1_B.wasJustPressed()) {
                G1B = true;

                G1BTime = getRuntime();
            }

            G1_B.readValue();


            //LINEAR SLIDES:

            double linearSlidePosition = -robot.linearSlideEncoder.getCurrentPosition();


            double linearSlidePower = 0;

            if (gamepad2.right_trigger > 0.5) {
                linearSlidePower += 1;
            } else {
                linearSlidePower += gamepad2.right_trigger * 2;
            }

            if ((linearSlidePosition > linearSlideZeroPosition + 50)) {
                if (gamepad2.left_trigger > 0.5) {
                    linearSlidePower -= 1;
                } else {
                    linearSlidePower -= gamepad2.left_trigger * 2;
                }
            }

            if (!linearAutomation && !PID_MODE) {
                if ((linearSlidePosition > linearSlideZeroPosition + 50) && gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
                    robot.leftSlide.setPower(slidesZeroPower);
                    robot.rightSlide.setPower(slidesZeroPower);
                } else {

                    robot.leftSlide.setPower(linearSlidePower);
                    robot.rightSlide.setPower(linearSlidePower);
                    if (linearSlidePower < 0.12) {
                        robot.leftSlide.setPower(0);
                    } else {
                        robot.leftSlide.setPower(linearSlidePower);

                    }


                }
            }

            if (G1_DPAD_UP.wasJustPressed()) {
                HIGH_SPECIMEN_POS_TELE += 100;
                target = (int) (HIGH_SPECIMEN_POS_TELE + linearSlideZeroPosition);
                PID_MODE = true;
            }


            if (G1_DPAD_DOWN.wasJustPressed()) {
                HIGH_SPECIMEN_POS_TELE -= 100;
                target = (int) (HIGH_SPECIMEN_POS_TELE + linearSlideZeroPosition);
                PID_MODE = true;
            }

            G1_DPAD_UP.readValue();

            G1_DPAD_DOWN.readValue();

            //LIMELIGHT


            if (USING_LIMELIGHT) {
                LLResult result = robot.limelight.getLatestResult();


                double[] pythonOutputs = result.getPythonOutput();

                if (pythonOutputs != null && pythonOutputs.length > 0) {
                    angle = 90 - pythonOutputs[3];

                    if (angle > 90) {
                        angle -= 180;
                    }


                    TELE.addData("ANGLE", angle);


                }

            }


            //LINEAR SLIDE AUTOMATION


            if (!linearAutomation && PID_MODE) {
                controller.setPID(p, i, d);
                double PID = controller.calculate(linearSlidePosition, target);
                PID += 0.08;

                robot.rightSlide.setPower(PID);
                robot.leftSlide.setPower(PID);

            }

            if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
                PID_MODE = false;
            }

            if (-gamepad2.left_stick_y < -0.5) {
                target = (int) linearSlideZeroPosition;
                PID_MODE = true;
            } else if (-gamepad2.left_stick_y > 0.5) {
                target = (int) linearSlideZeroPosition + HIGH_SAMPLE_POS;
                robot.clawPivot.setPosition(PIVOT_OUTTAKE);
                robot.clawLeftMove.setPosition(MOVE_OUTTAKE);
                robot.clawRightMove.setPosition(1-MOVE_OUTTAKE);

                PID_MODE = true;

            }


            //ROTATE CLAW

            if (gamepad2.right_stick_x > 0.10 || gamepad2.right_stick_x < -0.10) {
                robot.clawRotate.setPosition(robot.clawRotate.getPosition() + (10 * gamepad2.right_stick_x / 180));
            }

            //EXTENDO

            if ((robot.extendo.getCurrentPosition() - extendoZeroPosition) <= EXTENDO_MAX_TELE) {


                if (gamepad2.right_bumper && (robot.extendo.getCurrentPosition() - extendoZeroPosition <= EXTENDO_SOFTMAX_TELE)) {
                    robot.extendo.setPower(1);
                } else if (extendoOut) {
                    robot.extendo.setPower(1);
                } else if (extendoHoldOut) {
                    robot.extendo.setPower(0.2);
                } else if ((gamepad2.left_bumper || extendoIn)) {
                    robot.extendo.setPower(-1);
                } else if (extendoHoldIn) {
                    robot.extendo.setPower(-0.2);
                } else {
                    robot.extendo.setPower(0);
                }
            } else {

                if (maximumExtension && gamepad2.right_bumper && (robot.extendo.getCurrentPosition() - extendoZeroPosition <= EXTENDO_SOFTMAX_TELE)) {
                    robot.extendo.setPower(1);
                } else if ((gamepad2.left_bumper || extendoIn)) {
                    robot.extendo.setPower(-1);
                } else if (extendoHoldIn) {
                    robot.extendo.setPower(-0.2);
                } else {
                    robot.extendo.setPower(0);
                }

            }

            if (gamepad2.left_bumper || gamepad2.back || gamepad2.dpad_up || gamepad2.y || gamepad2.x) {
                extendoOut = false;
                extendoHoldOut = false;
            }

            if (gamepad2.right_bumper || gamepad2.start || gamepad2.dpad_down || gamepad2.x || gamepad2.y) {
                extendoIn = false;
                extendoHoldIn = false;
            }


            RIGHT_BUMPER_PRESS.readValue();

            if (RIGHT_BUMPER_PRESS.wasJustPressed()) {

                if ((getRuntime() - rightBumperPressTimestamp) < 0.5) {
                    maximumExtension = true;

                }
                rightBumperPressTimestamp = getRuntime();
            }

            if (maximumExtension && !gamepad2.right_bumper) {
                maximumExtension = false;
            }


            //AUTOMATION FOR SLIDE RESET

            if (G1_START_PRESS.wasJustPressed()) {

                slideResetToggle = true;
                slideResetTimestamp = getRuntime();

            }

            G1_START_PRESS.readValue();

            if (slideResetToggle) {

                double automationTime = getRuntime() - slideResetTimestamp;
                linearAutomation = true;

                if (automationTime < 0.25) {

                    robot.leftSlide.setPower(1);
                    robot.rightSlide.setPower(1);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_OUTTAKE);
                    robot.clawRightMove.setPosition(1-MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);

                } else if (automationTime < 0.85) {

                    robot.leftSlide.setPower(0.1);
                    robot.rightSlide.setPower(0.1);
                    extendoIn = true;

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_OUTTAKE - 0.1);
                    robot.clawRightMove.setPosition(1-(MOVE_OUTTAKE-0.1));
                    robot.clawPivot.setPosition(PIVOT_SPECIMEN_SCORE);
                } else if (automationTime < 2.15) {
                    extendoIn = false;
                    robot.leftSlide.setPower(-1);
                    robot.rightSlide.setPower(-1);

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_OUTTAKE);
                    robot.clawRightMove.setPosition(1-MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);

                } else if (automationTime < 2.65) {
                    extendoIn = false;


                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_OUTTAKE);
                    robot.clawRightMove.setPosition(1-MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);
                    linearSlideZeroPosition = robot.linearSlideEncoder.getCurrentPosition();
                    extendoZeroPosition = robot.extendoEncoder.getCurrentPosition();
                } else {

                    extendoIn = false;


                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_OUTTAKE);
                    robot.clawRightMove.setPosition(1-MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);

                    linearSlideZeroPosition = robot.linearSlideEncoder.getCurrentPosition();


                    extendoZeroPosition = robot.extendoEncoder.getCurrentPosition();

                    linearAutomation = false;
                    slideResetToggle = false;
                }
            }


            //AUTOMATION FOR CLAW OPEN AND CLOSE

            B_PRESS.readValue();

            if (B_PRESS.wasJustPressed()) {

                if ((getRuntime() - bPressTimestamp) < 0.5) {
                    robot.claw.setPosition(CLAW_LESS_OPEN);

                } else {
                    robot.claw.setPosition(CLAW_OPEN);

                }
                bPressTimestamp = getRuntime();
            }

            if (gamepad2.a) robot.claw.setPosition(CLAW_CLOSED);


            //AUTOMATION FOR START --> EXTEND TO SCORE FROM WALL

            if (specIn && holdSpec) {
                startPressTimestamp = getRuntime();
                specIn = false;
                startPressToggle = true;
            } else if (G1_LEFT_BUMPER.wasJustPressed() || START_PRESS.wasJustPressed()) {
                startPressTimestamp = getRuntime();
                startPressToggle = true;
                holdSpec = true;
                limitSwitchOff = true;
            }

            if (G1_LEFT_BUMPER.wasJustReleased() || START_PRESS.wasJustReleased()) {

                target = (int) (HIGH_SPECIMEN_POS_TELE + linearSlideZeroPosition);
                PID_MODE = true;

            }

            G1_LEFT_BUMPER.readValue();
            START_PRESS.readValue();
            if (startPressToggle) {
                double automationTime = getRuntime() - startPressTimestamp;
                if (automationTime < 0.25) {
                    robot.claw.setPosition(CLAW_LOOSE_GRAB);
                } else if (holdSpec) {
                    robot.clawLeftMove.setPosition(MOVE_SPECIMEN_SCORE);
                    robot.clawRightMove.setPosition(1-MOVE_SPECIMEN_SCORE);
                    robot.clawPivot.setPosition(PIVOT_SPECIMEN_SCORE);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    if (automationTime > 0.8){
                        robot.claw.setPosition(CLAW_CLOSED);
                    }
                    if (backPressToggle){
                        startPressToggle = false;
                        limitSwitchOff = false;
                    }
                } else {
                    robot.claw.setPosition(CLAW_OPEN);

                    target = (int) linearSlideZeroPosition;

                    startPressToggle = false;
                    limitSwitchOff = false;


                }
            }
            // BACK UP AUTOMATION FOR MISS

            if (X_PRESS.wasJustPressed()){
                xToggle = true;
                xTimeStamp = getRuntime();
            }

            if (xToggle){
                double time = getRuntime() - xTimeStamp;
                PID_MODE = true;
                target = (int) (LOWER_SLIDES_MISS);
                if (time > 0.4){
                    xToggle = false;
                }
            }

            X_PRESS.readValue();

            //AUTOMATION FOR BACK --> RETRACT TO WALL

            if (BACK_PRESS.wasJustPressed() || G1_RIGHT_BUMPER.wasJustPressed()) {
                backPressToggle = true;
                backPressTimestamp = getRuntime();
            }


            if (BACK_PRESS.wasJustReleased() || G1_RIGHT_BUMPER.wasJustReleased()) {
                backReleased = true;
            }
            if (backReleased && getRuntime() - backPressTimestamp > WAIT_BACK_BUTTON){
                target = (int) linearSlideZeroPosition;
                backReleased = false;
                PID_MODE = true;
            }

            G1_RIGHT_BUMPER.readValue();

            BACK_PRESS.readValue();

            if (backPressToggle) {
                double automationTime = getRuntime() - backPressTimestamp;

                if (automationTime < 0.2) {
                    robot.claw.setPosition(CLAW_OPEN);
                } else if (automationTime < 0.65) {
                    extendoIn = true;
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawPivot.setPosition(PIVOT_ALL_OUT);

                } else if (automationTime < WAIT_BACK_BUTTON) {
                    extendoIn = true;
                } else if (automationTime < WAIT_BACK_BUTTON + 0.5) {

                    robot.claw.setPosition(CLAW_OPEN);

                    robot.clawRotate.setPosition(ROTATE_FLIP);
                    robot.clawLeftMove.setPosition(MOVE_WALL_INTAKE);
                    robot.clawRightMove.setPosition(1-MOVE_WALL_INTAKE);
                    robot.clawPivot.setPosition(PIVOT_WALL_INTAKE);
                    extendoIn = true;
                } else {
                    extendoIn = false;
                    extendoHoldIn = true;


                    backPressToggle = false;

                }
            }

            //AUTOMATION FOR DPAD DOWN --> PICKUP

            if (DPAD_DOWN_PRESS.wasJustPressed()) {

                target = (int) (0 + linearSlideZeroPosition);
                PID_MODE = true;

                dpadDownToggle = true;


                dpadDownTimestamp = getRuntime();
                dpadDownServoLock = false;
            }

            if (DPAD_DOWN_PRESS.wasJustReleased()) {

                dpadDownServoLock = true;

                robot.claw.setPosition(CLAW_OPEN);
                robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
                robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE);
                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                robot.clawRotate.setPosition(ROTATE_90);

                if (!extendoOut) {
                    extendoOut = true;
                }

            }

            DPAD_DOWN_PRESS.readValue();

            if (dpadDownToggle) {

                double automationTime = getRuntime() - dpadDownTimestamp;

                extendoIn = false;
                extendoHoldIn = false;

                if (automationTime < 0.6) {


                    robot.claw.setPosition(CLAW_CLOSED);

                    if (!dpadDownServoLock) {
                        robot.clawPivot.setPosition(PIVOT_ALL_OUT);
                        robot.clawLeftMove.setPosition(MOVE_ALL_OUT);
                        robot.clawRightMove.setPosition(1-MOVE_ALL_OUT);
                        robot.clawRotate.setPosition(ROTATE_90);
                        robot.claw.setPosition(CLAW_CLOSED);
                    } else {
                        robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
                        robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE);
                        robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                        robot.clawRotate.setPosition(ROTATE_90);
                        robot.claw.setPosition(CLAW_CLOSED);
                    }

                } else {


                    if (!dpadDownServoLock) {
                        robot.clawPivot.setPosition(PIVOT_ALL_OUT);
                        robot.clawLeftMove.setPosition(MOVE_ALL_OUT);
                        robot.clawRightMove.setPosition(1-MOVE_ALL_OUT);
                        robot.clawRotate.setPosition(ROTATE_90);
                        robot.claw.setPosition(CLAW_CLOSED);
                    } else {
                        robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
                        robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE);
                        robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                        robot.clawRotate.setPosition(ROTATE_90);
                        robot.claw.setPosition(CLAW_OPEN);
                    }


                    dpadDownToggle = false;
                }


            }


            //AUTOMATION FOR Y --> GRAB AND GO TO RELEASE POSITION


            if (Y_PRESS.wasJustPressed()) {

                colorSensorTimer = getRuntime();


                if (robot.claw.getPosition() < CLAW_CLOSED + 0.04) {

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_WALL_INTAKE);
                    robot.clawRightMove.setPosition(1-MOVE_WALL_INTAKE);
                    robot.clawPivot.setPosition(PIVOT_WALL_INTAKE);

                    extendoIn = false;

                    dpadUpToggle = false;
                    extendoHoldIn = true;


                } else if (robot.clawPivot.getPosition() <= PIVOT_SAMPLE_PICKUP + 0.03 && robot.clawPivot.getPosition() >= PIVOT_SAMPLE_PICKUP - 0.03) {

                    robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE);
                    robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE);
                    robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                }


            }


            if (Y_PRESS.wasJustReleased()) {

                if (robot.claw.getPosition() > CLAW_CLOSED + 0.04 && (pickupSample || (getRuntime() - colorSensorTimer > 0.25))) {


                    yToggle = true;


                    yTimestamp = getRuntime();
                } else if (robot.clawPivot.getPosition() > (PIVOT_SAMPLE_PICKUP - 0.15)) {
                    robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
                    robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE);
                    robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);

                }

            }


            Y_PRESS.readValue();

            if (yToggle) {

                double automationTime = getRuntime() - yTimestamp;

                if (automationTime < 0.3) {
                    robot.claw.setPosition(CLAW_CLOSED);
                } else if (automationTime < 0.95) {

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_ALL_OUT);
                    robot.clawRightMove.setPosition(1-MOVE_ALL_OUT);
                    robot.clawPivot.setPosition(PIVOT_ALL_OUT);
                    extendoIn = true;
                } else if (automationTime < 1.05 && pickupSample) {

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_WALL_INTAKE);
                    robot.clawRightMove.setPosition(1-MOVE_WALL_INTAKE);
                    robot.clawPivot.setPosition(PIVOT_WALL_INTAKE);
                    extendoIn = true;
                } else {

                    extendoIn = false;
                    extendoHoldIn = true;


                    yToggle = false;
                }
            }

            //AUTOMATION FOR DPAD UP --> GRAB SPECIMEN

            if (DPAD_UP_PRESS.wasJustPressed()) {

                colorSensorTimer = getRuntime();

                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE);
                robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE);
                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);

                if (USING_LIMELIGHT) {
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL - angle * (0.29 / 90));
                }


            }

            if (DPAD_UP_PRESS.wasJustReleased()) {

                if (pickupSample || getRuntime() - colorSensorTimer > 0.25) {

                    dpadUpToggle = true;

                    dpadUpTimestamp = getRuntime();
                } else {
                    robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
                    robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE);
                }

            }


            DPAD_UP_PRESS.readValue();

            if (dpadUpToggle) {

                double automationTime = getRuntime() - dpadUpTimestamp;


                if (automationTime < 0.3) {

                    robot.claw.setPosition(CLAW_CLOSED);
                } else if (automationTime < 1) {

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawLeftMove.setPosition(MOVE_ALL_OUT);
                    robot.clawRightMove.setPosition(1-MOVE_ALL_OUT);
                    robot.clawPivot.setPosition(PIVOT_ALL_OUT);
                    extendoIn = true;
                } else {
                    extendoIn = false;
                    extendoHoldIn = true;


                    dpadUpToggle = false;
                }
            }


            //COLOR SENSOR
            pickupSample = true;
            //if (!useColorSensor){
            //    pickupSample = true;
            //} else if (robot.pin00.getState() &&  robot.pin01.getState() && !SPEC_MODE){
            //    pickupSample = true;
            //} else if (!robot.pin00.getState()&& robot.pin01.getState() && AUTON_RED ){
            //    pickupSample = true;
            //} else {
            //    pickupSample = robot.pin00.getState() && !robot.pin01.getState() && !AUTON_RED;
            //}

            if (!robot.pin00.getState() || !robot.pin01.getState()) {
                holdSpec = true;
                if (getRuntime() - limitSwitchTimestamp < 0.1) {
                    specIn = true;
                }
            } else if (robot.pin00.getState() && robot.pin01.getState() && !limitSwitchOff){
                holdSpec = false;
                specIn = false;
                limitSwitchTimestamp = getRuntime();
            }

            magneticSwitch = !robot.pin10.getState() || !robot.pin11.getState();

            //L3 HANG

            if (G1B){
                double time = getRuntime() - G1BTime;


                if (linearSlidePosition + linearSlideZeroPosition < HANG_1 + linearSlideZeroPosition && HANG_1_TARGET){
                    robot.leftStabilizer.setPosition(LEFT_HOLD_ON);

                    robot.rightStabilizer.setPosition(RIGHT_HOLD_ON);

                    robot.clawLeftMove.setPosition(MOVE_WALL_INTAKE);
                    robot.clawRightMove.setPosition(1-MOVE_WALL_INTAKE);
                    robot.clawPivot.setPosition(PIVOT_WALL_INTAKE);

                    target = (int) (HANG_1 + linearSlideZeroPosition);
                    PID_MODE = true;
                    if (linearSlidePosition + linearSlideZeroPosition > HANG_1 + linearSlideZeroPosition - 1000){
                        HANG_1_TARGET = false;
                    }
                } else if (linearSlidePosition + linearSlideZeroPosition > HANG_2 + linearSlideZeroPosition && HANG_2_TARGET){
                    HANG_1_TARGET = false;
                    target = (int) (HANG_2 + linearSlideZeroPosition);
                    PID_MODE = true;
                    G1BTime = getRuntime();
                    if (linearSlidePosition + linearSlideZeroPosition < HANG_2 + linearSlideZeroPosition + 1000){
                        HANG_2_TARGET = false;
                        G1BTime = getRuntime();
                    }
                } else if (time < 0.5){
                    HANG_2_TARGET = false;
                    PID_MODE = false;

                }
                else if (linearSlidePosition + linearSlideZeroPosition < HANG_3 + linearSlideZeroPosition && HANG_3_TARGET){
                    target = (int) (HANG_3 + linearSlideZeroPosition);
                    PID_MODE = true;
                    if (linearSlidePosition + linearSlideZeroPosition > HANG_3 + linearSlideZeroPosition - 1000){
                        HANG_3_TARGET = false;
                    }
                }
                else if (robot.extendo.getCurrentPosition() < EXTENDO_HANG && EXTENDO_HANG_TARGET){
                    HANG_3_TARGET = false;
                    robot.clawLeftMove.setPosition(MOVE_RAISED);
                    robot.clawRightMove.setPosition(1-MOVE_RAISED);
                    extendoOut = true;
                    if (robot.extendo.getCurrentPosition() > EXTENDO_HANG - 1000){
                        extendoOut = false;
                        robot.extendo.setPower(0);
                        EXTENDO_HANG_TARGET = false;
                    }
                }
                else if (linearSlidePosition + linearSlideZeroPosition > HANG_4 + linearSlideZeroPosition && HANG_4_TARGET){
                    extendoOut = false;
                    target = (int) (HANG_4 + linearSlideZeroPosition);
                    PID_MODE = true;
                    G1BTime = getRuntime();
                    if (linearSlidePosition + linearSlideZeroPosition < HANG_4 + linearSlideZeroPosition + 1000){
                        HANG_4_TARGET = false;
                    }
                }
                else if (!magneticSwitch && !magneticSwitchHang){
                    HANG_4_TARGET = false;
                    extendoHoldIn = false;
                    extendoHoldOut = false;
                    extendoOut = false;
                    PID_MODE = false;
                    linearAutomation = true;
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    if (time > 0.5){
                        robot.extendo.setPower(-1);
                        extendoIn = true;
                    }
                }

                else {
                    magneticSwitchHang = true;
                    extendoIn = false;
                    robot.extendo.setPower(0);
                    robot.leftStabilizer.setPosition(LEFT_HOLD_OFF);

                    robot.rightStabilizer.setPosition(RIGHT_HOLD_OFF);


                    PID_MODE = false;

                    linearAutomation = true;

                    robot.leftSlide.setPower(-1);
                    robot.rightSlide.setPower(-1);



                    G1B = false;
                }


            }





            //TELEMETRY:

            TELE.addData("Linear Slide Position", linearSlidePosition);
            TELE.addData("Linear Slide Zero Position", linearSlideZeroPosition);


            TELE.addData("Extendo Position", robot.extendo.getCurrentPosition());
            TELE.addData("Extendo Zero Position", extendoZeroPosition);

            TELE.addData("x", robot.drive.pose.position.x);
            TELE.addData("y", robot.drive.pose.position.y);

            TELE.addData("heading", Math.toDegrees(robot.drive.pose.heading.toDouble()));

            TELE.addData("pin00:", robot.pin00.getState());
            TELE.addData("pin01:", robot.pin01.getState());
            TELE.addData("pin10:", robot.pin10.getState());
            TELE.addData("pin11:", robot.pin11.getState());
            TELE.addData("Extendo Power:", robot.extendo.getPower());



            robot.drive.updatePoseEstimate();



            TELE.update();




        }

    }
}