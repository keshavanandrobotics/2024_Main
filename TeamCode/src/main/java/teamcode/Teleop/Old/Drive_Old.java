package teamcode.Teleop.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import teamcode.Robot;
import teamcode.Teleop.Singletons.DeadZones;
import teamcode.Teleop.Singletons.GamepadJoystickCurve;
import teamcode.Teleop.Singletons.MotorWeights;

@Config
@Disabled
@TeleOp
public class Drive_Old extends LinearOpMode {
    private static MultipleTelemetry TELE;

    public static String CURVE = "LINEAR";
    public static double DEGREE = 1;
    public static double SLIDE_ZERO_POWER = 0.1;


    public static double LINEAR_ZERO = 0;










    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);


        boolean bPressed = true;
        boolean intake = false;
        boolean grab = false;

        double POS_ZERO = -800000;

        double autoTime = 0.0;


        boolean extendoOut = false;
        boolean extendoIn = false;
        boolean automatic = false;


        double extendoPower = 0.0;


        double intakeStamp = 0.0;
        double grabStamp = 0.0;
        double bTimestamp = 0.0;

        boolean backMode = false;



        GamepadEx g1 = new GamepadEx(gamepad1);
        GamepadEx g2 = new GamepadEx(gamepad2);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ButtonReader g2X = new ButtonReader(
                g2, GamepadKeys.Button.A
        );

        ButtonReader g2press = new ButtonReader(
                g2, GamepadKeys.Button.LEFT_STICK_BUTTON
        );

        ButtonReader g2Back = new ButtonReader(
                g2, GamepadKeys.Button.BACK
        );

        ButtonReader g2B = new ButtonReader(
                g2, GamepadKeys.Button.B
        );

        ButtonReader dpadDown = new ButtonReader(
                g2, GamepadKeys.Button.DPAD_DOWN
        );

        ButtonReader dpadUp = new ButtonReader(
                g2, GamepadKeys.Button.DPAD_UP
        );

        ButtonReader start = new ButtonReader(
                g2, GamepadKeys.Button.START
        );

        boolean starty = false;
        double st = 0.0;









        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {





            double turbo = 0.8 + 0.2 * GamepadJoystickCurve.JoystickCurve(gamepad1.right_trigger, "POWER", 2.0)
                        - 0.6 * GamepadJoystickCurve.JoystickCurve(gamepad1.left_trigger, "POWER", 2.0);

            double rx = GamepadJoystickCurve.JoystickCurve(DeadZones.Linear(gamepad1.left_stick_x, 0.2), CURVE, DEGREE);
            double x = GamepadJoystickCurve.JoystickCurve(DeadZones.Linear(-gamepad1.right_stick_x, 0.2), CURVE, DEGREE);
            double y = GamepadJoystickCurve.JoystickCurve(DeadZones.Linear(gamepad1.right_stick_y, 0.2), CURVE, DEGREE);



            rx *= turbo;
            x *= turbo;
            y *= turbo;

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


            double linearSlidePosition = robot.leftSlide.getCurrentPosition() + robot.rightSlide.getCurrentPosition();
            linearSlidePosition *= (1.0 /3);

            double linearSlidePower = gamepad2.right_trigger;

            if((linearSlidePosition>LINEAR_ZERO+50)){
                linearSlidePower-=gamepad2.left_trigger;

            }








                if (!intake&&!grab&&!automatic) {
                    if ((linearSlidePosition > LINEAR_ZERO + 50) && gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
                        robot.leftSlide.setPower(SLIDE_ZERO_POWER);
                        robot.rightSlide.setPower(SLIDE_ZERO_POWER);
                    } else {

                        robot.leftSlide.setPower(linearSlidePower);
                        robot.rightSlide.setPower(linearSlidePower);


                    }
                }

                if (g2press.wasJustPressed()){
                    automatic = true;
                    autoTime = getRuntime();

            }

                if (automatic){
                    double ttTime = getRuntime() - autoTime;

                    if (ttTime<0.25){
                        robot.leftSlide.setPower(1);
                        robot.rightSlide.setPower(1);


                        robot.clawRotate.setPosition(0.3906);
                        robot.clawLeftMove.setPosition(0.8);
                        robot.clawPivot.setPosition(0.12);
                    }
                    else if (ttTime<0.85){

                        robot.leftSlide.setPower(0.1);
                        robot.rightSlide.setPower(0.1);
                        extendoIn = true;

                            robot.claw.setPosition(0.65);

                        robot.clawRotate.setPosition(0.3906);
                        robot.clawLeftMove.setPosition(0.8);
                        robot.clawPivot.setPosition(0.12);

                    } else if (ttTime<2.15){
                        extendoIn = false;
                        robot.leftSlide.setPower(-1);
                        robot.rightSlide.setPower(-1);

                            robot.claw.setPosition(0.65);

                        robot.clawRotate.setPosition(0.3906);

                        robot.claw.setPosition(0.65);
                        robot.clawRotate.setPosition(.3906);
                        robot.clawLeftMove.setPosition(0.6);
                        robot.clawPivot.setPosition(0.62);

                    }
                    else if (ttTime<2.95){
                        extendoIn = false;
                        robot.leftSlide.setPower(0);
                        robot.rightSlide.setPower(0);

                        robot.claw.setPosition(0.65);
                        robot.clawRotate.setPosition(0.3906);
                        robot.clawLeftMove.setPosition(0.6);
                        robot.clawPivot.setPosition(0.62);
                        LINEAR_ZERO = linearSlidePosition;



                    }




                    else {
                        robot.leftSlide.setPower(0);
                        robot.rightSlide.setPower(0);

                        robot.claw.setPosition(0.65);
                        robot.clawRotate.setPosition(0.95);
                        robot.clawLeftMove.setPosition(0.6);
                        robot.clawPivot.setPosition(0.62);
                        LINEAR_ZERO = linearSlidePosition;
                        automatic = false;
                    }
                }

                g2press.readValue();



         if (gamepad2.right_stick_x!=0){
                robot.clawRotate.setPosition(robot.clawRotate.getPosition()+(4*gamepad2.right_stick_x/180));
            }

            g2B.readValue();

            if (g2B.wasJustPressed()){

                if ((getRuntime()-bTimestamp)<0.5){
                    robot.claw.setPosition(0.74);

                } else {
                    robot.claw.setPosition(0.84);

                }
                bPressed = true;
                bTimestamp = getRuntime();
            }

            g2X.readValue();

            if (g2X.wasJustPressed()){
                if (bPressed) {
                    robot.claw.setPosition(0.65);
                    bPressed = false;
                } else {
                    robot.claw.setPosition(0.65);
                    bPressed = true;
                }
            }

            if (g2Back.wasJustPressed()){
                backMode = true;
                grab = true;
                grabStamp = getRuntime();
            }

            g2Back.readValue();


            if (gamepad2.left_bumper||gamepad2.back){
                extendoOut = false;
            }

            if (gamepad2.right_bumper||gamepad2.start){
                extendoIn = false;
            }



            if ((gamepad2.right_bumper||((intake||starty)&&extendoOut))){
                robot.extendo.setPower(1);

            } else if ((gamepad2.left_bumper||((grab||automatic)&&extendoIn))){
                robot.extendo.setPower(-1);

                if (robot.extendo.getCurrent(CurrentUnit.AMPS)>6){

                    POS_ZERO = robot.extendoEncoder.getCurrentPosition();
                }

            } else {
                robot.extendo.setPower(0);










            }



            if (gamepad2.x&&(robot.extendoEncoder.getCurrentPosition()>(POS_ZERO+500))){

                    robot.claw.setPosition(0.65);

                robot.clawLeftMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
            }

            if (gamepad2.y){

                    robot.claw.setPosition(0.65);

                robot.clawRotate.setPosition(0.3906);
                robot.clawLeftMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
            }

            if (start.wasJustPressed()){
                starty = true;
                st = getRuntime();
            }

            if (starty){
                double n = getRuntime() - st;
                if (n <0.3){
                    robot.claw.setPosition(0.65);
                    robot.clawRotate.setPosition(0.95);
                    robot.clawLeftMove.setPosition(0.6);
                    robot.clawPivot.setPosition(0.62);
                } else if (n<0.7){
                    extendoOut = true;
                } else if (n <1){
                    robot.claw.setPosition(0.84);
                } else {
                    extendoOut = false;
                    starty = false;
                }
            }

            start.readValue();

            if(dpadDown.wasJustPressed()){
                intake = true;
                 intakeStamp = getRuntime();

            }


            if(dpadUp.wasJustPressed()){
                grab = true;
                grabStamp = getRuntime();

            }

            if (gamepad2.right_stick_button){
                LINEAR_ZERO = linearSlidePosition;
            }

            if (grab){
                extendoOut = false;
                double grabTimer = getRuntime() - grabStamp;
                if ((grabTimer<0.2)){

                    if (backMode){
                        robot.claw.setPosition(0.65);

                    } else {
                        robot.leftSlide.setPower(-0.5);
                        robot.rightSlide.setPower(-0.5);
                    }

                } else if (grabTimer<0.5){
                    if (!backMode){
                        robot.leftSlide.setPower(0);
                        robot.rightSlide.setPower(0);

                    } else {
                        extendoIn = true;
                        robot.clawRotate.setPosition(0.3906);
                    }

                } else if (grabTimer<0.8){
                    if (!backMode){
                        robot.leftSlide.setPower(0);
                        robot.rightSlide.setPower(0);
                        robot.claw.setPosition(0.65);
                    } else {
                        extendoIn = true;
                        robot.clawRotate.setPosition(0.3906);
                    }

                }

                else if (grabTimer<1.5){

                        robot.claw.setPosition(0.65);

                    robot.clawRotate.setPosition(0.3906);
                    robot.clawLeftMove.setPosition(0.8);
                    robot.clawPivot.setPosition(0.12);
                    extendoIn = true;

                } else {
                    extendoIn = true;
                    grab = false;

                    if (backMode){
                        robot.clawRotate.setPosition(0.95);

                    }
                    backMode= false;

                }
            }


            TELE.addData("zz", LINEAR_ZERO);
            TELE.addData("d", linearSlidePosition);

            if (intake){

                extendoIn = false;
                double intakeTimer = getRuntime()-intakeStamp;
                robot.claw.setPosition(0.65);


                robot.clawPivot.setPosition(0.62);
                robot.clawLeftMove.setPosition(0.6);

                if (intakeTimer<0.5){
                    extendoOut = true;

                } else if (intakeTimer<0.7){
                    extendoOut = true;
                    robot.leftSlide.setPower(1);
                    robot.rightSlide.setPower(1);


                } else {
                    extendoOut = true;
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.clawLeftMove.setPosition(0.48);
                    robot.clawPivot.setPosition(0.93);
                    robot.claw.setPosition(0.84);
                    robot.clawRotate.setPosition(0.3906);

                    intake=false;
                }
            }

            dpadDown.readValue();
            dpadUp.readValue();

            TELE.addData("va", robot.rightSlide.getCurrent(CurrentUnit.MILLIAMPS));









            TELE.update();









        }


    }
}

