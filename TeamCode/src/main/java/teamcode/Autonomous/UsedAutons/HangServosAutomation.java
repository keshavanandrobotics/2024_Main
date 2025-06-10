package teamcode.Autonomous.UsedAutons;

import static teamcode.Teleop.Singletons.VARS.*;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import teamcode.Robot;


@Config
@Autonomous(preselectTeleOp = "Drive_V3")

public class HangServosAutomation extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.leftHook.setPosition(LEFT_SPRING_ON);
        robot.rightHook.setPosition(RIGHT_SPRING_ON);
        waitForStart();
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            robot.leftHook.setPosition(LEFT_SPRING_OFF);
            robot.rightHook.setPosition(RIGHT_SPRING_OFF);
            sleep(10000);
        }
    }
}