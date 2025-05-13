package teamcode.Autonomous.UsedAutons;

import static teamcode.Teleop.Singletons.VARS.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import teamcode.Robot;

public class HangServosAutomation extends LinearOpMode {
    Robot robot;

    public void runOpMode() throws InterruptedException {
        robot.leftSpringHook.setPosition(LEFT_SPRING_ON);
        robot.rightSpringHook.setPosition(RIGHT_SPRING_ON);
        if (isStopRequested()) return;
        if (opModeIsActive()) {
            robot.leftSpringHook.setPosition(LEFT_SPRING_OFF);
            robot.rightSpringHook.setPosition(RIGHT_SPRING_OFF);
        }
    }
}