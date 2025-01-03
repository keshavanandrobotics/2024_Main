package teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import teamcode.Robot;


@Autonomous
@Config
public class SlidesPIDTest extends LinearOpMode {

    Robot robot;

    public static PIDFController PIDF = new PIDFController(0, 0, 0, 0);

    public static int TARGET_POSITION = 10000;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot (hardwareMap);


        waitForStart();

        if (opModeIsActive()){
            while (Math.abs(robot.linearSlideEncoder.getCurrentPosition() - TARGET_POSITION) <50){
                double power = PIDF.calculate(robot.linearSlideEncoder.getCurrentPosition(), TARGET_POSITION);

                robot.leftSlide.setPower(power);
                robot.centerSlide.setPower(power);
                robot.rightSlide.setPower(power);


            }
        }

    }

}
