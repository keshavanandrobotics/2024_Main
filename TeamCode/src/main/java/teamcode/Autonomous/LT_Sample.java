package teamcode.Autonomous;



import static teamcode.Teleop.Singletons.Positions.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;

import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;

@Config
@Autonomous (preselectTeleOp = "Drive_V2")
public class LT_Sample extends LinearOpMode{

    Robot robot;

    public int TARGET = 0;

    public class SpecimenScoreServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_SPECIMEN_SCORE);
            robot.clawRotate.setPosition(ROTATE_NEUTRAL);
            robot.clawMove.setPosition(MOVE_SPECIMEN_SCORE);


            robot.claw.setPosition(CLAW_CLOSED);
            return false;
        }
    }

    public class ParkServos implements Action {

        double stamp = getRuntime();

        double ticker = 1;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (ticker ==1){
                stamp = getRuntime();
            }

            ticker ++;

            if (getRuntime()- stamp< 0.2){
                robot.clawPivot.setPosition(PIVOT_ALL_OUT);
                robot.clawRotate.setPosition(ROTATE_NEUTRAL);
                robot.clawMove.setPosition(MOVE_ALL_OUT);


                robot.claw.setPosition(CLAW_OPEN);
                return  true;
            } else {


                return false;
            }
        }
    }



    public class LinearSlidesPID implements Action {

        private final PIDController controller = new PIDController(0.0006,0,0.00001);

        double timeStamp = getRuntime();




        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            double linearSlidePosition = -robot.linearSlideEncoder.getCurrentPosition();

            controller.setPID(0.0006,0,0.00001);

            double power = controller.calculate(linearSlidePosition, TARGET) + 0.08;

            robot.leftSlide.setPower(power);
            robot.rightSlide.setPower(power);
            robot.centerSlide.setPower(power);

            telemetry.addData("power", power);
            telemetry.addData("Target", TARGET);
            telemetry.addData("pos", linearSlidePosition);

            if ((Math.abs(TARGET - linearSlidePosition)<300) || linearSlidePosition > 59000 && TARGET ==HIGH_SAMPLE_POS){
                telemetry.addLine("Success");
                telemetry.update();

                if (TARGET<=0){
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);
                } else {

                    robot.leftSlide.setPower(0.12);
                    robot.rightSlide.setPower(0.12);
                    robot.centerSlide.setPower(0.12);
                }

                return false;

            } else {


                telemetry.update();

                return  true;

            }

        }
    }

    public class OuttakeServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_OUTTAKE);
            robot.clawRotate.setPosition(ROTATE_NEUTRAL);
            robot.clawMove.setPosition(MOVE_OUTTAKE);


            robot.claw.setPosition(CLAW_CLOSED);
            return false;
        }
    }

    public class SampleHoverServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
            robot.clawRotate.setPosition(ROTATE_NEUTRAL);
            robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);


            robot.claw.setPosition(CLAW_OPEN);
            return false;
        }
    }

    public class Sample3HoverServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
            robot.clawRotate.setPosition(ROTATE_90);
            robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);


            robot.claw.setPosition(CLAW_OPEN);
            return false;
        }
    }

    public class ServosPickupSample implements Action {

        double ticker = 1;

        double stamp;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (ticker ==1){
                stamp = getRuntime();
            }

            ticker ++;

            if (getRuntime() - stamp < 0.4){
                robot.clawMove.setPosition(MOVE_PICKUP_SAMPLE);

                robot.clawMove.setPosition(MOVE_PICKUP_SAMPLE);

                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);

                return true;

            } else if ( getRuntime() - stamp < 0.6){

                robot.clawMove.setPosition(MOVE_PICKUP_SAMPLE);
                robot.claw.setPosition(CLAW_CLOSED);
                return true;
            } else {

                robot.clawMove.setPosition(MOVE_PICKUP_SAMPLE);
                robot.claw.setPosition(CLAW_CLOSED);
                return false;
            }



        }
    }


    public class ExtendoOut implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (robot.extendo.getCurrentPosition()<EXTENDO_MAX_TELE){
                robot.extendo.setPower(1);
                return true;

            } else {
                robot.extendo.setPower(0);
                return false;
            }



        }
    }

    public class ExtendoSpecimenOut implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            robot.extendo.setPower(1);
            return false;


        }
    }


    public class ExtendoIn implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (robot.extendoEncoder.getCurrentPosition()>200){
                robot.extendo.setPower(-1);
                return true;
            } else {
                return false;
            }





        }
    }



    public class LinearSlidesSpecimenScore implements Action {

        private final PIDController controller = new PIDController(0.0006,0,0.00001);

        double timeStamp = getRuntime();




        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            double linearSlidePosition = -robot.linearSlideEncoder.getCurrentPosition();

            controller.setPID(0.0006,0,0.00001);

            double power = controller.calculate(linearSlidePosition, TARGET) + 0.08;

            robot.leftSlide.setPower(power);
            robot.rightSlide.setPower(power);
            robot.centerSlide.setPower(power);

            telemetry.addData("power", power);
            telemetry.addData("Target", TARGET);
            telemetry.addData("pos", linearSlidePosition);

            if (robot.drive.pose.position.x>15){
                telemetry.addLine("Success");
                telemetry.update();

                robot.leftSlide.setPower(0.12);
                robot.rightSlide.setPower(0.12);
                robot.centerSlide.setPower(0.12);

                return false;

            } else {

                if (linearSlidePosition> 14000){
                    robot.extendo.setPower(1);
                }


                telemetry.update();

                return  true;

            }

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {



        robot = new Robot(hardwareMap);
        robot.drive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.linearSlideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlideEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.claw.setPosition(CLAW_CLOSED);
        robot.clawMove.setPosition(MOVE_AUTONOMOUS_INIT);
        robot.clawPivot.setPosition(PIVOT_AUTONOMOUS_INIT);
        robot.clawRotate.setPosition(ROTATE_NEUTRAL);

        TrajectoryActionBuilder specimenScore = robot.drive.actionBuilder(new Pose2d(0,0,0))
                .splineTo(new Vector2d(20,0),0);

        TrajectoryActionBuilder firstSamplePickup = robot.drive.actionBuilder(new Pose2d(20,0,0))
                .strafeToLinearHeading(new Vector2d(12.7, 41), Math.toRadians(0),
                        new TranslationalVelConstraint(20));

        TrajectoryActionBuilder firstSampleNet = robot.drive.actionBuilder(new Pose2d(12.7, 41, 0))

                .strafeToLinearHeading(new Vector2d(9,44), Math.toRadians(-45));



        TrajectoryActionBuilder scoreSample = robot.drive.actionBuilder(new Pose2d(9, 44, Math.toRadians(-45)))

                .strafeToLinearHeading(new Vector2d(5,50), Math.toRadians(-45),
                        new TranslationalVelConstraint(20));

        TrajectoryActionBuilder secondSamplePickup = robot.drive.actionBuilder(new Pose2d(5, 50, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(12.7,51), Math.toRadians(0),
                        new TranslationalVelConstraint(10));

        TrajectoryActionBuilder secondSampleNet = robot.drive.actionBuilder(new Pose2d(12.7 , 50, 0))

                .strafeToLinearHeading(new Vector2d(9,44), Math.toRadians(-45));


        TrajectoryActionBuilder thirdSamplePickup = robot.drive.actionBuilder(new Pose2d(5, 50, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(37.5,33), Math.toRadians(90),
                        new TranslationalVelConstraint(20));

        TrajectoryActionBuilder thirdSampleNet = robot.drive.actionBuilder(new Pose2d(37.5 , 33, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(9,44), Math.toRadians(-45));

        TrajectoryActionBuilder park = robot.drive.actionBuilder(new Pose2d(5, 50, Math.toRadians(-45)))
                .splineToSplineHeading(new Pose2d(56,27,Math.toRadians(-90)), Math.toRadians(0),
                        new TranslationalVelConstraint(30))
                .splineToSplineHeading(new Pose2d(56,14,Math.toRadians(-90)), Math.toRadians(0),
                        new TranslationalVelConstraint(30));

        waitForStart();

        if(isStopRequested()) return;

        if (opModeIsActive()){
            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            new SpecimenScoreServos(),
                            new LinearSlidesSpecimenScore(),
                            specimenScore.build()
                    )
            );


            robot.claw.setPosition(CLAW_OPEN);

            TARGET = -200;

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new ExtendoIn(),
                                    new ParallelAction(
                                            new SampleHoverServos()),
                                            new LinearSlidesPID(),
                                            new ExtendoOut()

                            ),

                            firstSamplePickup.build()
                    )
            );

            Actions.runBlocking(new ServosPickupSample());

            TARGET = HIGH_SAMPLE_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            firstSampleNet.build(),
                            new LinearSlidesPID(),
                            new ExtendoIn(),
                            new OuttakeServos()
                    )
            );

            robot.extendo.setPower(-0.2);

            Actions.runBlocking(scoreSample.build());





          TARGET = -200;
          robot.claw.setPosition(CLAW_OPEN);
          sleep( 350);

          Actions.runBlocking(secondSamplePickup.build());



            Actions.runBlocking(
                    new ParallelAction(

                            new LinearSlidesPID(),
                            new ExtendoOut(),
                            new SampleHoverServos()

                    )
            );

            Actions.runBlocking(new ServosPickupSample());

            TARGET = HIGH_SAMPLE_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            secondSampleNet.build(),
                            new LinearSlidesPID(),
                            new ExtendoIn(),
                            new OuttakeServos()
                    )
            );
            robot.extendo.setPower(-0.2);

            Actions.runBlocking(scoreSample.build());

            TARGET = -200;
            robot.claw.setPosition(CLAW_OPEN);
            sleep( 350);

            Actions.runBlocking(thirdSamplePickup.build());

            Actions.runBlocking(
                    new ParallelAction(

                            new LinearSlidesPID(),
                            new ExtendoOut(),
                            new Sample3HoverServos()

                    )
            );

            Actions.runBlocking(new ServosPickupSample());

            TARGET = HIGH_SAMPLE_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            thirdSampleNet.build(),
                            new LinearSlidesPID(),
                            new ExtendoIn(),
                            new OuttakeServos()
                    )
            );

            robot.extendo.setPower(-0.2);

            Actions.runBlocking(scoreSample.build());

            robot.claw.setPosition(CLAW_OPEN);

            sleep( 350);

            TARGET = AUTO_PARK_SLIDE_POS;

            Actions.runBlocking(

                    new ParallelAction(
                            park.build(),
                            new SequentialAction(

                                    new ParkServos(),

                                    new LinearSlidesPID()

                                    )
                    )

            );

        }


    }
}