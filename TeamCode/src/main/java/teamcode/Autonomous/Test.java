package teamcode.Autonomous;

import static teamcode.Teleop.Singletons.VARS.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ejml.equation.Variable;

import teamcode.Robot;
import teamcode.Teleop.Singletons.VARS;


@Config
@Autonomous

public class Test extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        robot.claw.setPosition(CLAW_OPEN);
        robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);
        robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);


        robot.limelight = hardwareMap.get(Limelight3A.class, "limelight");


        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()){
            robot.extendo.setPower(0.7);

            while(robot.extendo.getPower()>0.1) {

                if (!robot.pin0.getState() && robot.pin1.getState()) {
                    robot.extendo.setPower(0);
                }

            }

            LLResult result = robot.limelight.getLatestResult();




            double[] pythonOutputs = result.getPythonOutput();

            if (pythonOutputs != null && pythonOutputs.length > 0) {
               double angle = 90 - pythonOutputs[3];

                if (angle > 90){
                    angle -= 180;
                }

                robot.clawRotate.setPosition(ROTATE_90 - angle* (0.29/90));

                sleep (1000);




            }

            robot.extendo.setPower(0.2);
            sleep(100);
            robot.extendo.setPower(0);


            robot.clawMove.setPosition(MOVE_PICKUP_SAMPLE);

            sleep(500);
            robot.claw.setPosition(CLAW_CLOSED);
            sleep(500);

            robot.clawMove.setPosition(MOVE_ALL_OUT);
            robot.clawPivot.setPosition(PIVOT_ALL_OUT);

            sleep(150000);


        }

    }
}
