package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Teleop.Singletons.DeadZones;
import org.firstinspires.ftc.teamcode.Teleop.Singletons.GamepadJoystickCurve;
import org.firstinspires.ftc.teamcode.Teleop.Singletons.MotorWeights;

@Config
@TeleOp
public class PositionalServoProgrammer extends LinearOpMode {

    public static double TEST_POSITION = 1;
    public static double MOVE_POSITION = 0.6;
    public static double PIVOT_POSITION = 0.6;





    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {


        robot = new Robot(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            robot.testServo.setPosition(TEST_POSITION);
            robot.clawPivot.setPosition(PIVOT_POSITION);
            robot.clawMove.setPosition(MOVE_POSITION);




        }


    }
}

