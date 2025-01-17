package teamcode.Teleop;
import static teamcode.Autonomous.Poses.*;
import static teamcode.Teleop.Singletons.Positions.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Objects;


import teamcode.Autonomous.Poses;
import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;
import teamcode.Teleop.Singletons.MotorWeights;



@Config
@TeleOp

public class Drive_V2 extends LinearOpMode{
    private static MultipleTelemetry TELE;

    public static String MODE = "SAM";





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



    public double slidesZeroPower = 0.1;
    public boolean slideResetToggle = false;
    public double slideResetTimestamp = 0.0;

    public double bPressTimestamp = 0.0;
    public double yTimestamp = 0.0;
    public boolean yToggle = false;
    public double startPressTimestamp = 0.0;
    public double dpadDownTimestamp = 0.0;
    public double dpadUpTimestamp = 0.0;
    public double backPressTimestamp = 0.0;

    public double rightBumperPressTimestamp = 0.0;


    public boolean extendoIn = false;
    public boolean extendoOut = false;
    public boolean extendoHoldIn = false;

    public boolean extendoHoldOut = false;
    public boolean PID_MODE = false;




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
        robot.centerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        controller = new PIDController(p, i, d);

        GamepadEx g2 = new GamepadEx(gamepad2);
        GamepadEx g1 = new GamepadEx(gamepad1);

        ButtonReader G1_START_PRESS = new ButtonReader(
                g1, GamepadKeys.Button.START
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

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            //DRIVETRAIN:

            double rx = 0;
            double x = 0;
            double y = 0;


            if (Objects.equals(MODE, "SAM")){

                rx = gamepad1.left_stick_x;
                x = -gamepad1.right_stick_x;
                y = gamepad1.right_stick_y;

            } else if (Objects.equals(MODE, "!SAM")){
                rx = gamepad1.left_stick_x;
                x = gamepad1.right_stick_x;
                y = -gamepad1.right_stick_y;
            }

            double slowFactor = 1 - gamepad1.left_trigger*0.8;

            rx*=slowFactor;
            x*=slowFactor;
            y*=slowFactor;


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



            //LINEAR SLIDES:

            double linearSlidePosition = -robot.linearSlideEncoder.getCurrentPosition();


            double linearSlidePower = 0;

            if (gamepad2.right_trigger>0.5){
                linearSlidePower += 1;
            } else {
                linearSlidePower += gamepad2.right_trigger*2;
            }

            if((linearSlidePosition>linearSlideZeroPosition+50)){
                if (gamepad2.left_trigger>0.5){
                    linearSlidePower -= 1;
                } else {
                    linearSlidePower -= gamepad2.left_trigger*2;
                }
            }

            if (!linearAutomation && !PID_MODE) {
                if ((linearSlidePosition > linearSlideZeroPosition + 50) && gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
                    robot.leftSlide.setPower(slidesZeroPower);
                    robot.rightSlide.setPower(slidesZeroPower);
                } else {

                    robot.leftSlide.setPower(linearSlidePower);
                    robot.rightSlide.setPower(linearSlidePower);
                    if (linearSlidePower<0.12) {
                        robot.centerSlide.setPower(0);
                    } else {
                        robot.centerSlide.setPower(linearSlidePower);

                    }


                }
            }

            //LINEAR SLIDE AUTOMATION


            if (!linearAutomation && PID_MODE){
                controller.setPID(p,i,d);
                double PID = controller.calculate(linearSlidePosition, target);
                PID += 0.08;

                robot.rightSlide.setPower(PID);
                robot.leftSlide.setPower(PID);
                robot.centerSlide.setPower(PID);

            }

            if (gamepad2.left_trigger!= 0|| gamepad2.right_trigger!= 0) {PID_MODE = false;}

            if (-gamepad2.left_stick_y<0){
                target = (int) linearSlideZeroPosition;
                PID_MODE = true;
            }

            if (-gamepad2.left_stick_y>0){
                target = (int) linearSlideZeroPosition + HIGH_SAMPLE_POS;
                robot.clawPivot.setPosition(PIVOT_OUTTAKE);
                robot.clawMove.setPosition(MOVE_OUTTAKE);

                PID_MODE = true;

            }


            //ROTATE CLAW

            if (gamepad2.right_stick_x!=0){
                robot.clawRotate.setPosition(robot.clawRotate.getPosition()+(4*gamepad2.right_stick_x/180));
            }
            
            //EXTENDO

            if ((robot.extendo.getCurrentPosition()-extendoZeroPosition)<=EXTENDO_MAX_TELE){



                if ((gamepad2.right_bumper||extendoOut)){robot.extendo.setPower(1);}

                else if (extendoHoldOut){robot.extendo.setPower(0.3);}
                else if ((gamepad2.left_bumper||extendoIn)){robot.extendo.setPower(-1);}

                else if (extendoHoldIn){robot.extendo.setPower(-0.3);}

                else {robot.extendo.setPower(0);}
            } else {

                if (maximumExtension&&gamepad2.right_bumper){
                    robot.extendo.setPower(1);
                }
                else if ((gamepad2.left_bumper || extendoIn)) {
                    robot.extendo.setPower(-1);
                } else if (extendoHoldIn) {
                    robot.extendo.setPower(-0.3);
                } else {
                    robot.extendo.setPower(0);
                }

            }

            if (gamepad2.left_bumper||gamepad2.back||gamepad2.dpad_up||gamepad2.y||gamepad2.x){
                extendoOut = false;
                extendoHoldOut = false;
            }

            if (gamepad2.right_bumper||gamepad2.start||gamepad2.dpad_down||gamepad2.x||gamepad2.y){
                extendoIn = false;
                extendoHoldIn = false;
            }

            if (gamepad1.back){
                extendoZeroPosition = robot.extendo.getCurrentPosition();
            }

            RIGHT_BUMPER_PRESS.readValue();

            if (RIGHT_BUMPER_PRESS.wasJustPressed()){

                if ((getRuntime()-rightBumperPressTimestamp)<0.5){
                    maximumExtension = true;

                }
                rightBumperPressTimestamp = getRuntime();
            }

            if (maximumExtension && !gamepad2.right_bumper){
                maximumExtension = false;
            }





            //AUTOMATION FOR SLIDE RESET

            if (G1_START_PRESS.wasJustPressed()){

                slideResetToggle = true;
                slideResetTimestamp = getRuntime();

            }

            G1_START_PRESS.readValue();

            if (slideResetToggle){

                double automationTime = getRuntime() - slideResetTimestamp;
                linearAutomation = true;

                if (automationTime < 0.25){

                    robot.leftSlide.setPower(1);
                    robot.rightSlide.setPower(1);
                    robot.centerSlide.setPower(1);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);

                } else if (automationTime<0.85){

                    robot.leftSlide.setPower(0.1);
                    robot.rightSlide.setPower(0.1);
                    robot.centerSlide.setPower(0.1);
                    extendoIn = true;

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_OUTTAKE - 0.1);
                    robot.clawPivot.setPosition(PIVOT_SPECIMEN_SCORE);
                } else if (automationTime<2.15) {
                    extendoIn = false;
                    robot.leftSlide.setPower(-1);
                    robot.rightSlide.setPower(-1);
                    robot.centerSlide.setPower(-1);

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);

                }  else if (automationTime < 2.65){
                    extendoIn = false;


                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);
                    linearSlideZeroPosition = robot.linearSlideEncoder.getCurrentPosition();
                    extendoZeroPosition = robot.extendoEncoder.getCurrentPosition();
                }

                else {

                    extendoIn = false;


                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);

                    linearSlideZeroPosition = robot.linearSlideEncoder.getCurrentPosition();


                    extendoZeroPosition = robot.extendoEncoder.getCurrentPosition();

                    linearAutomation = false;
                    slideResetToggle = false;
                }
            }


            //AUTOMATION FOR CLAW OPEN AND CLOSE

            B_PRESS.readValue();

            if (B_PRESS.wasJustPressed()){

                if ((getRuntime()-bPressTimestamp)<0.5){
                    robot.claw.setPosition(CLAW_LESS_OPEN);

                } else {
                    robot.claw.setPosition(CLAW_OPEN);

                }
                bPressTimestamp = getRuntime();
            }

            if(gamepad2.a) robot.claw.setPosition(CLAW_CLOSED);





            //AUTOMATION FOR START --> EXTEND TO SCORE FROM WALL

            if (START_PRESS.wasJustPressed()){
                startPressToggle = true;
                startPressTimestamp = getRuntime();
            }

            if (START_PRESS.wasJustReleased()){
                target = HIGH_SPECIMEN_POS;
                PID_MODE = true;
            }

            START_PRESS.readValue();

            if (startPressToggle){
                double automationTime = getRuntime() - startPressTimestamp;

                if (automationTime < 0.4){
                    robot.claw.setPosition(CLAW_CLOSED);
                } else {
                    robot.clawMove.setPosition(MOVE_SPECIMEN_SCORE);
                    robot.clawPivot.setPosition(PIVOT_SPECIMEN_SCORE);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    startPressToggle = false;
                }
            }

            //AUTOMATION FOR BACK --> RETRACT TO WALL

            if (BACK_PRESS.wasJustPressed()){
                backPressToggle = true;
                backPressTimestamp = getRuntime();
            }

            if (BACK_PRESS.wasJustReleased()){
                target = (int) linearSlideZeroPosition;
                PID_MODE = true;
            }

            BACK_PRESS.readValue();

            if (backPressToggle) {
                double automationTime = getRuntime() - backPressTimestamp;

                if (automationTime < 0.2){
                    robot.claw.setPosition(CLAW_OPEN);
                } else if (automationTime < 0.65){
                    extendoIn = true;
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);

                } else if (automationTime < 0.8){
                    extendoIn = true;
                    robot.claw.setPosition(CLAW_CLOSED);
                } else if (automationTime < 1.5){

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_FLIP);
                    robot.clawMove.setPosition(MOVE_WALL_INTAKE);
                    robot.clawPivot.setPosition(PIVOT_WALL_INTAKE);
                    extendoIn = true;
                } else {
                    extendoIn = false;
                    extendoHoldIn = true;

                    robot.clawRotate.setPosition(ROTATE_FLIP);
                    robot.claw.setPosition(CLAW_OPEN);

                    backPressToggle = false;

                }
            }

            //AUTOMATION FOR DPAD DOWN --> PICKUP

            if (DPAD_DOWN_PRESS.wasJustPressed() || X_PRESS.wasJustPressed()){

                target = (int) ( 0 + linearSlideZeroPosition);
                PID_MODE = true;

                dpadDownToggle = true;


                dpadDownTimestamp = getRuntime();
                dpadDownServoLock = false;
            }

            if (DPAD_DOWN_PRESS.wasJustReleased()||X_PRESS.wasJustReleased()){

                dpadDownServoLock = true;

                robot.claw.setPosition(CLAW_OPEN);
                robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);
                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);

                if(!extendoOut){
                    extendoOut = true;
                }

            }

            DPAD_DOWN_PRESS.readValue();

            X_PRESS.readValue();

            if (dpadDownToggle){

                double automationTime = getRuntime() - dpadDownTimestamp;

                extendoIn = false;
                extendoHoldIn = false;

                if (automationTime<0.6){


                    robot.claw.setPosition(CLAW_CLOSED);

                    if (!dpadDownServoLock) {
                        robot.clawPivot.setPosition(PIVOT_ALL_OUT);
                        robot.clawMove.setPosition(MOVE_ALL_OUT);
                        robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                        robot.claw.setPosition(CLAW_CLOSED);
                    } else {
                        robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);
                        robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                        robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                        robot.claw.setPosition(CLAW_CLOSED);
                    }

                } else {





                    if (!dpadDownServoLock) {
                        robot.clawPivot.setPosition(PIVOT_ALL_OUT);
                        robot.clawMove.setPosition(MOVE_ALL_OUT);
                        robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                        robot.claw.setPosition(CLAW_CLOSED);
                    } else {
                        robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);
                        robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                        robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                        robot.claw.setPosition(CLAW_OPEN);
                    }


                    dpadDownToggle=false;
                }



            }




            //AUTOMATION FOR Y --> GRAB AND GO TO RELEASE POSITION



            if (Y_PRESS.wasJustReleased()){

                yToggle = true;

                yTimestamp = getRuntime();

            }



            Y_PRESS.readValue();

            if (yToggle){

                double automationTime = getRuntime() - yTimestamp;



                if (automationTime<0.3){

                    robot.claw.setPosition(CLAW_CLOSED);
                } else if (automationTime < 1){

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_WALL_INTAKE);
                    robot.clawPivot.setPosition(PIVOT_WALL_INTAKE);
                    extendoIn = true;
                } else {
                    extendoIn = false;
                    extendoHoldIn = true;


                    yToggle = false;
                }
            }

            //AUTOMATION FOR DPAD UP --> GRAB SPECIMEN

            if (DPAD_UP_PRESS.wasJustPressed()){

                robot.clawMove.setPosition(MOVE_PICKUP_SAMPLE);
                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);

            }

            if (DPAD_UP_PRESS.wasJustReleased()){

                dpadUpToggle = true;

                dpadUpTimestamp = getRuntime();

            }



            DPAD_UP_PRESS.readValue();

            if (dpadUpToggle){

                double automationTime = getRuntime() - dpadUpTimestamp;



                if (automationTime<0.3){

                    robot.claw.setPosition(CLAW_CLOSED);
                } else if (automationTime < 1){

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_ALL_OUT);
                    robot.clawPivot.setPosition(PIVOT_ALL_OUT);
                    extendoIn = true;
                } else {
                    extendoIn = false;
                    extendoHoldIn = true;


                    dpadUpToggle = false;
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

            robot.drive.updatePoseEstimate();



            TELE.update();




        }

    }
}
