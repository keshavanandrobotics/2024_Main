package teamcode.Teleop;
import static teamcode.Teleop.Singletons.ServoPositions.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Objects;

import teamcode.Autonomous.Poses;
import teamcode.Robot;
import teamcode.Teleop.Singletons.DeadZones;
import teamcode.Teleop.Singletons.GamepadJoystickCurve;
import teamcode.Teleop.Singletons.MotorWeights;
import teamcode.Teleop.Singletons.ServoPositions;
import teamcode.pedroPathing.constants.FConstants;
import teamcode.pedroPathing.constants.LConstants;


@Config
@TeleOp

public class Drive_V2 extends LinearOpMode{
    private static MultipleTelemetry TELE;

    public static String MODE = "SAM";

    public static boolean PEDRO = false;

    private Follower follower;
    private final Pose startPose = Poses.AUTON_END;




    public double linearSlideZeroPosition = 0;
    public boolean linearAutomation = false;
    public double slidesZeroPower = 0;

    public boolean slideResetToggle = false;
    public double slideResetTimestamp = 0.0;
    public double bPressTimestamp = 0.0;

    public boolean extendoIn = false;



    Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        if (PEDRO) {

            Constants.setConstants(FConstants.class, LConstants.class);
            follower = new Follower(hardwareMap);
            follower.setStartingPose(startPose);
        }

        GamepadEx g2 = new GamepadEx(gamepad2);

        ButtonReader LEFT_STICK_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.LEFT_STICK_BUTTON
        );

        ButtonReader B_PRESS = new ButtonReader(
                g2, GamepadKeys.Button.B
        );

        waitForStart();

        if (isStopRequested()) return;

        if (PEDRO) follower.startTeleopDrive();

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

            double slowFactor = gamepad1.left_trigger*0.8;

            rx*=slowFactor;
            x*=slowFactor;
            y*=slowFactor;

            if (PEDRO){
                follower.setTeleOpMovementVectors(y, x, rx, true);
                follower.update();

                TELE.addData("X", follower.getPose().getX());
                TELE.addData("Y", follower.getPose().getY());
                TELE.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            } else {
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

            }

            //LINEAR SLIDES:

            double linearSlidePosition = robot.leftSlide.getCurrentPosition() + robot.rightSlide.getCurrentPosition() + robot.centerSlide.getCurrentPosition();
            linearSlidePosition *= (1.0 /3);

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

            if (!linearAutomation) {
                if ((linearSlidePosition > linearSlideZeroPosition + 50) && gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
                    robot.leftSlide.setPower(slidesZeroPower);
                    robot.rightSlide.setPower(slidesZeroPower);
                } else {

                    robot.leftSlide.setPower(linearSlidePower);
                    robot.centerSlide.setPower(linearSlidePower);
                    robot.rightSlide.setPower(linearSlidePower);


                }
            }

            //ROTATE CLAW

            if (gamepad2.right_stick_x!=0){
                robot.clawRotate.setPosition(robot.clawRotate.getPosition()+(4*gamepad2.right_stick_x/180));
            }


            //AUTOMATION FOR SLIDE RESET

            if (LEFT_STICK_PRESS.wasJustPressed()){

                slideResetToggle = true;
                slideResetTimestamp = getRuntime();

            }

            LEFT_STICK_PRESS.readValue();

            if (slideResetToggle){

                double automationTime = getRuntime() - slideResetTimestamp;

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
                    robot.clawMove.setPosition(MOVE_OUTTAKE);
                    robot.clawPivot.setPosition(PIVOT_OUTTAKE);
                } else if (automationTime<2.15) {
                    extendoIn = false;
                    robot.leftSlide.setPower(-1);
                    robot.rightSlide.setPower(-1);
                    robot.centerSlide.setPower(-1);

                    robot.claw.setPosition(CLAW_CLOSED);

                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_NEUTRAL);
                    robot.clawPivot.setPosition(PIVOT_NEUTRAL);

                } else if (automationTime<2.95){
                    extendoIn = false;
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                    robot.clawMove.setPosition(MOVE_NEUTRAL);
                    robot.clawPivot.setPosition(PIVOT_NEUTRAL);
                    linearSlideZeroPosition = linearSlidePosition;
                } else {

                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);

                    robot.claw.setPosition(CLAW_CLOSED);
                    robot.clawRotate.setPosition(ROTATE_FLIP);
                    robot.clawMove.setPosition(MOVE_NEUTRAL);
                    robot.clawPivot.setPosition(PIVOT_NEUTRAL);
                    linearSlideZeroPosition = linearSlidePosition;
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

            if(gamepad2.x) robot.claw.setPosition(CLAW_CLOSED);










            TELE.update();




        }

    }
}
