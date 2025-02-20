package teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Robot;

@Config
@TeleOp
public class PositionalServoProgrammer extends LinearOpMode {

    public static double TEST_POSITION = 0.501;
    public static double MOVE_POSITION = 0.501;
    public static double PIVOT_POSITION = 0.501;
    public static double CLAW_POSITION= 0.501;
    public static double ROTATE_POSITION = 0.501;







    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {


        robot = new Robot(hardwareMap);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (TEST_POSITION != 0.501){robot.testServo.setPosition(TEST_POSITION);}
            if (PIVOT_POSITION != 0.501){robot.clawPivot.setPosition(PIVOT_POSITION);}
            if (MOVE_POSITION != 0.501){robot.clawMove.setPosition(MOVE_POSITION);}
            if (ROTATE_POSITION != 0.501){robot.clawRotate.setPosition(ROTATE_POSITION);}
            if (CLAW_POSITION != 0.501){robot.claw.setPosition(CLAW_POSITION);}






        }


    }
}

