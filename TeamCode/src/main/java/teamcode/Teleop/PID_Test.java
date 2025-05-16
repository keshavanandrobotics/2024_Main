package teamcode.Teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Robot;

@TeleOp
@Config
public class PID_Test extends LinearOpMode {

    private PIDController controller;
    public static double p = 0.0006, i = 0, d = 0.00001;
    public static double f = 0.08;

    public static boolean extendo = false;

    public static int target = 15000;

    private final double ticks = 1425.1 /360;

    Robot robot;

    int pos;


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);

        controller.setTolerance(50);
        robot = new Robot (hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            controller.setPID(p,i,d);

            if (extendo){
                pos = robot.extendo.getCurrentPosition();
            } else {
                pos = -robot.linearSlideEncoder.getCurrentPosition();
            }

            double pid = controller.calculate(pos, target);

            double ff =  f;

            double power = pid + ff;


            if (extendo){
                robot.extendo.setPower(power);
            } else {
                robot.rightSlide.setPower(power);
                robot.leftSlide.setPower(power);
                robot.centerSlide.setPower(power);
            }

            telemetry.addData("pos", pos);
            telemetry.addData("target", target);
            telemetry.update();





        }

    }
}
