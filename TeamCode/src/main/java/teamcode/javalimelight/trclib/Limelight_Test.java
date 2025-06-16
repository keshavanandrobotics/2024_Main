package teamcode.javalimelight.trclib;

import static java.lang.Math.*;

import static teamcode.Autonomous.UsedAutons.SampleAuton.*;
import static teamcode.Teleop.Singletons.VARS.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.Robot;
import teamcode.javalimelight.trclib.LL_Tracker;
import teamcode.javalimelight.trclib.pathdrive.TrcPose2D;
import teamcode.javalimelight.trclib.pathdrive.TrcPose3D;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@TeleOp
public class Limelight_Test extends LinearOpMode {
    public static int COLOR = 9;

    public static boolean CHECK_ANGLE = false;
    private MultipleTelemetry TELE;

    Robot robot;

    LL_Tracker llTracker;

    LLResult result;

    private boolean Initialize () {




        // Limelight Init
//        robot.limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        robot.limelight.pipelineSwitch(9);
//        robot.limelight.start();
        llTracker = new LL_Tracker();
        final boolean init = llTracker.Init(robot, hardwareMap, TELE, COLOR);
        return init;
    }
    private void slowstrafeRight (double power) {
        robot.frontLeftMotor.setPower(power);
        robot.backLeftMotor.setPower(-power);
        robot.frontRightMotor.setPower(-power);
        robot.backRightMotor.setPower(power);
    }

    private void slowstrafeLeft (double power) {
        robot.frontLeftMotor.setPower(-power);
        robot.backLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(-power);
    }

    private void stopstrafe () {
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    private void strafeAlign (LLResult llResult) {
        double targetTx = llResult.getTx();
        double drivePower = 0.0;
        final double MIN_DRIVE_POWER = 0.15;
        final double TARGET_POSITION_TOLERANCE = 3.0;
        final double HORIZONTAL_FOV_RANGE = 26.0;
        final double DRIVE_POWER_REDUCTION = 2.0;

        // Only with valid data and if too far off target
        if ((llResult.getPythonOutput()[0]==1) && (Math.abs(targetTx) > TARGET_POSITION_TOLERANCE))
        {

            // Adjust Robot Speed based on how far the target is located
            // Only drive at half speed max
            drivePower = targetTx/HORIZONTAL_FOV_RANGE / DRIVE_POWER_REDUCTION;

            // Make sure we have enough power to actually drive the wheels
            if (abs(drivePower) < MIN_DRIVE_POWER) {
                if (drivePower > 0.0) {
                    drivePower = MIN_DRIVE_POWER;
                } else {
                    drivePower = -MIN_DRIVE_POWER;
                }

            }
            telemetry.addData("drivePower: ", drivePower);

            if (drivePower > 0.0) {
                slowstrafeLeft(abs(drivePower));
            } else
            {
                slowstrafeRight(abs(drivePower));
            }

        }
        else {
            telemetry.addData("drivePower", "No Target or Movement Needed");
            stopstrafe();
        }
        telemetry.update();
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
        robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
        robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE);
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
            if (Initialize()&&!CHECK_ANGLE){
                while (!llTracker.Track()){
                    CHECK_ANGLE = false;
                    Actions.runBlocking(
                            new SequentialAction(
                                    LinearSlidePID(LINEAR_SLIDES_HOVER_LIMELIGHT,0)
                            )
                    );
                    robot.clawRightMove.setPosition(1-MOVE_HOVER_SAMPLE);
                    robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
                    robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                    robot.claw.setPosition(CLAW_OPEN);
                }
                stopstrafe();
                llTracker.Stop();
                CHECK_ANGLE = true;
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
