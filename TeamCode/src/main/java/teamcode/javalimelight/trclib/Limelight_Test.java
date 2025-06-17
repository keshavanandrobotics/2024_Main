package teamcode.javalimelight.trclib;

import static java.lang.Math.*;

import static teamcode.Teleop.Singletons.VARS.*;
import static teamcode.Autonomous.UsedAutons.SampleAuton_NoLimelight.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
@TeleOp
public class Limelight_Test extends LinearOpMode {
    public static int COLOR = 9;

    public static boolean CHECK_ANGLE = false;
    private MultipleTelemetry TELE;
    LL_Tracker llTracker;

    Robot robot;

    private boolean Initialize () {




        // Limelight Init
//        robot.limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        robot.limelight.pipelineSwitch(9);
//        robot.limelight.start();
        llTracker = new LL_Tracker();
        return llTracker.Init(robot, hardwareMap, TELE, COLOR);
    }


    private void stopstrafe () {
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }




    public Action LinearSlidePID(int position, double holdPower){

        return new Action() {

            private final PIDController controller = new PIDController(0.0006,0,0.00001);





            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                double linearSlidePosition = -robot.linearSlideEncoder.getCurrentPosition();

                controller.setPID(0.0006,0,0.00001);

                double power = controller.calculate(linearSlidePosition, position) + 0.08;

                robot.leftSlide.setPower(power);
                robot.rightSlide.setPower(power);

                telemetry.addData("power", power);
                telemetry.addData("Target", position);
                telemetry.addData("pos", linearSlidePosition);

                if (Math.abs(position - linearSlidePosition)<475){
                    telemetry.addLine("Success");
                    telemetry.update();


                    robot.leftSlide.setPower(holdPower);
                    robot.rightSlide.setPower(holdPower);


                    return false;

                } else {


                    telemetry.update();

                    return  true;

                }

            }

        };
    }

    public Action extendoPID (int position){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (position == EXTENDO_SAMPLE_PICKUP && robot.extendoEncoder.getCurrentPosition() < EXTENDO_SAMPLE_PICKUP){
                    robot.extendo.setPower(1);
                    return true;
                } else if (position == EXTENDO_PICKUP_SAMPLE && robot.extendoEncoder.getCurrentPosition() > EXTENDO_PICKUP_SAMPLE) {
                    robot.extendo.setPower(-0.5);
                    return true;
                } else{
                    robot.extendo.setPower(0);
                    return false;
                }
            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Base Robot must be initialized first.
        robot = new Robot(hardwareMap);

        //robot.drive = new PinpointDrive(hardwareMap, AUTON_END_POSE);

        // Motors
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
        robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE_LIMELIGHT);
        robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE_LIMELIGHT);
        robot.claw.setPosition(CLAW_OPEN);
        robot.linearSlideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlideEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Actions.runBlocking(
                new SequentialAction(
                        LinearSlidePID(LINEAR_SLIDES_HOVER_LIMELIGHT,0),
                        extendoPID(EXTENDO_SAMPLE_PICKUP)
                )
        );
        waitForStart();

        if (isStopRequested()) return;

        // Main Drive Loop runs until signaled to stop by Driver HUB
        while (opModeIsActive()) {

            //controlDrivetrain();
            //getLimelightResults();
            if (Initialize() && !CHECK_ANGLE){
                if (!llTracker.Track()){
                    CHECK_ANGLE = false;
                    robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE_LIMELIGHT);
                    robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE_LIMELIGHT);
                    robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                    robot.claw.setPosition(CLAW_OPEN);
                    sleep(1);
                } else {
                    CHECK_ANGLE = true;
                    stopstrafe();
                    llTracker.Stop();
                }

                if (CHECK_ANGLE){

                    telemetry.addData("Angle", llTracker.getSampleAngle());
                    robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE_LIMELIGHT);
                    robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE_LIMELIGHT);
                    robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP_LIMELIGHT);
                    sleep(200);
                    Actions.runBlocking(
                            new SequentialAction(
                                    LinearSlidePID(LINEAR_SLIDES_PICKUP_LIMELIGHT,0)
                            )
                    );
                    sleep(300);
                    robot.claw.setPosition(CLAW_CLOSED);
                    sleep(200);
                    robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE_LIMELIGHT);
                    robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE_LIMELIGHT);
                    sleep(5000);
                    robot.claw.setPosition(CLAW_OPEN);
                    CHECK_ANGLE = false;
                } else {
                    Actions.runBlocking(
                            new SequentialAction(
                                    LinearSlidePID(LINEAR_SLIDES_HOVER_LIMELIGHT,0)
                            )
                    );
                }

            } else if (!Initialize()){
                TELE.addLine("No limelight");
            }





            //TELEMETRY:

            //TELE.addData("Linear Slide Position", linearSlidePosition);
            //TELE.addData("Linear Slide Zero Position", linearSlideZeroPosition);


            //TELE.addData("Extendo Position", robot.extendo.getCurrentPosition());
            //TELE.addData("Extendo Zero Position", extendoZeroPosition);

            //TELE.addData("x", robot.drive.pose.position.x);
            //TELE.addData("y", robot.drive.pose.position.y);

            //TELE.addData("heading", Math.toDegrees(robot.drive.pose.heading.toDouble()));

            //TELE.addData("pin0", robot.pin0.getState());
            //TELE.addData("pin1", robot.pin1.getState());



            //robot.drive.updatePoseEstimate();



            TELE.update();




        }
        //robot.limelight.stop();

    }
}
