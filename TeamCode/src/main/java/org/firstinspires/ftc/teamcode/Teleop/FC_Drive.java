package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Teleop.Singletons.GamepadJoystickCurve;

@Config
@TeleOp
public class FC_Drive extends LinearOpMode {
    private static MultipleTelemetry TELE;
    public static String CURVE = "LINEAR";
    public static double DEGREE = 1;




    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);


        GamepadEx g1 = new GamepadEx(gamepad1);
        GamepadEx g2 = new GamepadEx(gamepad2);

        robot.imu.initialize(robot.parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.start) {
                robot.imu.resetYaw();
            }


            double turbo = 0.8 + 0.2 * GamepadJoystickCurve.JoystickCurve(gamepad1.right_trigger, "POWER", 2.0)
                    - 0.6 * GamepadJoystickCurve.JoystickCurve(gamepad1.left_trigger, "POWER", 2.0);

            double rx = GamepadJoystickCurve.JoystickCurve(gamepad1.left_stick_x,CURVE, DEGREE);
            double x = GamepadJoystickCurve.JoystickCurve(gamepad1.right_stick_x,CURVE, DEGREE);
            double y = GamepadJoystickCurve.JoystickCurve(-gamepad1.right_stick_y,CURVE, DEGREE);

            rx*=turbo;
            x*=turbo;
            y*=turbo;

            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.frontLeftMotor.setPower(frontLeftPower);
            robot.backLeftMotor.setPower(backLeftPower);
            robot.frontRightMotor.setPower(frontRightPower);
            robot.backRightMotor.setPower(backRightPower);


        }


    }
}

