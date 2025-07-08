package teamcode.Autonomous.UsedAutons;


import static teamcode.Autonomous.Disabled.Poses.*;
import static teamcode.Teleop.Singletons.VARS.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;
import teamcode.javalimelight.trclib.LL_Tracker;


@Config
@Autonomous (preselectTeleOp = "Drive_V4")
public class SampleAuton_Limelight extends LinearOpMode{

    Robot robot;
    public static int COLOR = 8;

    public static boolean CHECK_ANGLE = false;
    private MultipleTelemetry TELE;
    LL_Tracker llTracker;


    public static int SPEED = 30;
    public static int SUBSPEED = 170;

    public int TARGET = 0;
    public static double MOVE_3 = 0.54;
    public static int LINEAR_SLIDES_HOVER_LIMELIGHT = 10000;

    public static int LINEAR_SLIDES_PICKUP_LIMELIGHT = 7000;

    public static int LINEAR_SLIDES_LOWER = 1500;
    public static int LINEAR_SLIDES_LOWER_CHECK = 2500;

    public static double SAMPLE_DOWN_TIME = 0.5;
    public static double SAMPLE_SCORE_TIME = 0.25;
    public static double LINEAR_SLIDES_UP = 2.0;
    public static double PARK_TIME = 1.0;
    public static double EXTENDO_OUT = 0.5;
    public static double LINEAR_SLIDES_TIME = 1.7;

    public static double SAMPLE_NET_X1 = 9.5, SAMPLE_NET_Y1 = 20;
    public static double SAMPLE_NET_X2 = 3.5, SAMPLE_NET_Y2 = 20.5;
    public static double SAMPLE_NET_HEADING = -45;
    public static double SAMPLE_NET_HEADING_SUB = -30;

    public static double SAMPLE_X1 = 16, SAMPLE_Y1 = 9.5;
    public static double SAMPLE_X2 = 16, SAMPLE_Y2 = 19.5;
    public static double SAMPLE_X3 = 36.5, SAMPLE_Y3 = 7, SAMPLE_HEADING3 = 90;
    public static double SUB_X1 = 50, SUB_Y1 = 0;
    public static double SUB_X2 = 60, SUB_Y2 = -15;
    public static double SUB_X3 = 56, SUB_Y3 = -25;
    public static double SUB_HEADING = -90;

    public double startStamp = 0.0;

    public static double TIME_ALLOWED = 29.0;
    public static double LINEAR_SLIDES_SUB = 1.5;

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
                robot.clawLeftMove.setPosition(MOVE_ALL_OUT);
                robot.clawRightMove.setPosition(1-MOVE_ALL_OUT);


                robot.claw.setPosition(CLAW_CLOSED);
                return  true;
            } else {


                return false;
            }
        }
    }



    public Action LinearSlidesPID(int position, double holdPower){

        return new Action() {

            private final PIDController controller = new PIDController(0.0003,0,0.00001);
            double stamp = getRuntime();





            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                double linearSlidePosition = -robot.linearSlideEncoder.getCurrentPosition();

                controller.setPID(0.0003,0,0.00001);

                double power = controller.calculate(linearSlidePosition, position) + 0.08;

                robot.leftSlide.setPower(power);
                robot.rightSlide.setPower(power);

                TELE.addData("power", power);
                TELE.addData("Target", position);
                TELE.addData("pos", linearSlidePosition);

                if (Math.abs(position - linearSlidePosition)<475 || (linearSlidePosition > HIGH_SAMPLE_POS+1000 && position == HIGH_SAMPLE_POS_TELE) || (linearSlidePosition < LINEAR_SLIDES_LOWER_CHECK && position ==  LINEAR_SLIDES_LOWER) || (getRuntime()-stamp > LINEAR_SLIDES_TIME && linearSlidePosition > HIGH_SAMPLE_POS) || getRuntime()-stamp > LINEAR_SLIDES_UP){
                    TELE.addLine("Success");
                    TELE.update();


                    robot.leftSlide.setPower(holdPower);
                    robot.rightSlide.setPower(holdPower);


                    return false;

                } else {


                    TELE.update();

                    return  true;

                }

            }

        };
    }

    public Action Wait (double time) {

        return new Action() {

            double stamp = getRuntime();

            int ticker = 1;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (ticker == 1) {
                    stamp = getRuntime();
                }
                ticker++;

                if (getRuntime() - stamp < time) {
                    return true;
                } else {


                    return false;
                }
            }
        };
    }


    public class upOuttakeServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_OUTTAKE);
            robot.clawRotate.setPosition(ROTATE_90);
            robot.clawLeftMove.setPosition(MOVE_RAISED);
            robot.clawRightMove.setPosition(1-MOVE_RAISED);


            robot.claw.setPosition(CLAW_CLOSED);
            return false;
        }
    }

    public class downOuttakeServos implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            robot.clawLeftMove.setPosition(MOVE_OUTTAKE);
            robot.clawRightMove.setPosition(1-MOVE_OUTTAKE);
            return false;
        }
    }



    public class SampleHoverServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
            robot.clawRotate.setPosition(ROTATE_NEUTRAL);
            robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
            robot.clawRightMove.setPosition(1-(MOVE_HOVER_SAMPLE));


            robot.claw.setPosition(CLAW_OPEN);
            return false;
        }
    }

    public class SampleHoverServos_Limelight implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP_LIMELIGHT);
            robot.clawRotate.setPosition(ROTATE_NEUTRAL);
            robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE_LIMELIGHT);
            robot.clawRightMove.setPosition(1-(MOVE_HOVER_SAMPLE_LIMELIGHT));


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
            robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
            robot.clawRightMove.setPosition(1-(MOVE_HOVER_SAMPLE));


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
                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE);
                robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE);



                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);

                return true;

            } else if ( getRuntime() - stamp < 0.6){

                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE);
                robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE);
                robot.claw.setPosition(CLAW_CLOSED);
                return true;
            } else {

                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE);
                robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE);
                robot.claw.setPosition(CLAW_CLOSED);
                return false;
            }



        }
    }

    public class ServosPickupSample3 implements Action {

        double ticker = 1;

        double stamp;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (ticker ==1){
                stamp = getRuntime();
            }

            ticker ++;

            if (getRuntime() - stamp < 0.4){
                robot.clawLeftMove.setPosition(MOVE_3);
                robot.clawRightMove.setPosition(1-MOVE_3);



                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);

                return true;

            } else if ( getRuntime() - stamp < 0.6){

                robot.clawLeftMove.setPosition(MOVE_3);
                robot.clawRightMove.setPosition(1-MOVE_3);
                robot.claw.setPosition(CLAW_CLOSED);
                return true;
            } else {

                robot.clawLeftMove.setPosition(MOVE_3);
                robot.clawRightMove.setPosition(1-MOVE_3);
                robot.claw.setPosition(CLAW_CLOSED);
                return false;
            }



        }
    }

    public class ServosPickupSample_Limelight implements Action {

        double ticker = 1;

        double stamp;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (ticker ==1){
                stamp = getRuntime();
            }

            ticker ++;

            if (getRuntime() - stamp < 0.5){
                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE_LIMELIGHT);
                robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE_LIMELIGHT);



                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP_LIMELIGHT);

                return true;

            } else if ( getRuntime() - stamp < 0.8){

                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE_LIMELIGHT);
                robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE_LIMELIGHT);
                robot.claw.setPosition(CLAW_CLOSED);
                return true;
            } else {

                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE_LIMELIGHT);
                robot.clawRightMove.setPosition(1-MOVE_PICKUP_SAMPLE_LIMELIGHT);
                robot.claw.setPosition(CLAW_CLOSED);
                return false;
            }



        }
    }


    public class ExtendoOut implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (robot.extendo.getCurrentPosition()<EXTENDO_SAMPLE_PICKUP){
                robot.extendo.setPower(1);
                return true;

            } else {
                robot.extendo.setPower(0);
                return false;
            }



        }
    }




    public class ExtendoIn implements Action {

        double timer = 0.0;
        int ticker = 1;



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (ticker ==1){
                timer = getRuntime();
            }

            ticker ++;

            robot.extendo.setPower(-1);


            if (robot.extendoEncoder.getCurrentPosition()>200){


                robot.extendo.setPower(-1);


                telemetry.update();
                return true;
            } else if (getRuntime()-timer < 0.4){
                return  true;
            }

            else {
                robot.extendo.setPower(-1);
                return false;
            }





        }
    }





    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.drive = new PinpointDrive(hardwareMap, AUTON_START_POSE);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        robot.linearSlideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlideEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.claw.setPosition(CLAW_CLOSED);
        robot.clawLeftMove.setPosition(MOVE_AUTONOMOUS_INIT);
        robot.clawRightMove.setPosition(1-MOVE_AUTONOMOUS_INIT);
        robot.clawPivot.setPosition(PIVOT_AUTONOMOUS_INIT);
        robot.clawRotate.setPosition(ROTATE_NEUTRAL);

        robot.rightStabilizer.setPosition(RIGHT_HOLD_OFF);
        robot.leftStabilizer.setPosition(LEFT_HOLD_OFF);

        robot.leftPTO.setPosition(LEFT_PTO_OFF);
        robot.rightPTO.setPosition(RIGHT_PTO_OFF);

        robot.limelight.setPollRateHz(100);
        robot.limelight.start();
        robot.limelight.pipelineSwitch(COLOR);




        TrajectoryActionBuilder sample0Net = robot.drive.actionBuilder(AUTON_START_POSE)
                .strafeToLinearHeading(new Vector2d(SAMPLE_NET_X2, SAMPLE_NET_Y2), Math.toRadians(SAMPLE_NET_HEADING));



        TrajectoryActionBuilder firstSamplePickup = robot.drive.actionBuilder(new Pose2d(SAMPLE_NET_X2, SAMPLE_NET_Y2, Math.toRadians(SAMPLE_NET_HEADING)))
                .strafeToLinearHeading(new Vector2d(SAMPLE_X1, SAMPLE_Y1), Math.toRadians(0),
                        new TranslationalVelConstraint(SPEED));

        TrajectoryActionBuilder firstSampleNet = robot.drive.actionBuilder(new Pose2d(SAMPLE_X1, SAMPLE_Y1, 0))
                .strafeToLinearHeading(new Vector2d(SAMPLE_NET_X2, SAMPLE_NET_Y2), Math.toRadians(SAMPLE_NET_HEADING),
                        new TranslationalVelConstraint(SPEED));


        TrajectoryActionBuilder secondSamplePickup = robot.drive.actionBuilder(new Pose2d(SAMPLE_NET_X2, SAMPLE_NET_Y2, Math.toRadians(SAMPLE_NET_HEADING)))
                .strafeToLinearHeading(new Vector2d(SAMPLE_X2,SAMPLE_Y2), Math.toRadians(0),
                        new TranslationalVelConstraint(SPEED));

        TrajectoryActionBuilder secondSampleNet = robot.drive.actionBuilder(new Pose2d(SAMPLE_X2 , SAMPLE_Y2, 0))
                .strafeToLinearHeading(new Vector2d(SAMPLE_NET_X2, SAMPLE_NET_Y2), Math.toRadians(SAMPLE_NET_HEADING));


        TrajectoryActionBuilder thirdSamplePickup = robot.drive.actionBuilder(new Pose2d(SAMPLE_NET_X2, SAMPLE_NET_Y2, Math.toRadians(SAMPLE_NET_HEADING)))
                .strafeToLinearHeading(new Vector2d(SAMPLE_X3,SAMPLE_Y3), Math.toRadians(SAMPLE_HEADING3),
                        new TranslationalVelConstraint(SPEED));

        TrajectoryActionBuilder thirdSampleNet = robot.drive.actionBuilder(new Pose2d(SAMPLE_X3, SAMPLE_Y3, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(SAMPLE_NET_X2, SAMPLE_NET_Y2), Math.toRadians(SAMPLE_NET_HEADING),
                        new TranslationalVelConstraint(SPEED));

        TrajectoryActionBuilder sub = robot.drive.actionBuilder(new Pose2d(SAMPLE_NET_X2, SAMPLE_NET_Y2, Math.toRadians(SAMPLE_NET_HEADING)))
                .splineToSplineHeading(new Pose2d(SUB_X1, SUB_Y1,Math.toRadians(SUB_HEADING)), Math.toRadians(0),
                        new TranslationalVelConstraint(SUBSPEED))
                .splineToSplineHeading(new Pose2d(SUB_X2,SUB_Y2,Math.toRadians(SUB_HEADING)), Math.toRadians(0),
                        new TranslationalVelConstraint(SPEED));

        TrajectoryActionBuilder park = robot.drive.actionBuilder(new Pose2d(SAMPLE_NET_X2, SAMPLE_NET_Y2, Math.toRadians(SAMPLE_NET_HEADING_SUB)))
                .splineToSplineHeading(new Pose2d(SUB_X1, SUB_Y1,Math.toRadians(SUB_HEADING)), Math.toRadians(0),
                        new TranslationalVelConstraint(SUBSPEED))
                .splineToSplineHeading(new Pose2d(SUB_X3,SUB_Y3,Math.toRadians(SUB_HEADING)), Math.toRadians(0),
                        new TranslationalVelConstraint(SUBSPEED));

        TrajectoryActionBuilder subScore = robot.drive.actionBuilder(new Pose2d(SUB_X2, SUB_Y2, Math.toRadians(SUB_HEADING)))
                .strafeToLinearHeading(new Vector2d(SUB_X1, SUB_Y1), Math.toRadians(SUB_HEADING),
                        new TranslationalVelConstraint(SUBSPEED))
                .strafeToLinearHeading(new Vector2d(SAMPLE_NET_X2, SAMPLE_NET_Y2), Math.toRadians(SAMPLE_NET_HEADING_SUB),
                        new TranslationalVelConstraint(SPEED));
        while (!Initialize()){
            sleep(1);
        }

        waitForStart();

        if(isStopRequested()) return;

        if (opModeIsActive()){
            startStamp = getRuntime();
            robot.leftPTO.setPosition(LEFT_PTO_OFF);
            robot.rightPTO.setPosition(RIGHT_PTO_OFF);
            TARGET = HIGH_SAMPLE_POS_TELE;

            Actions.runBlocking(
                    new ParallelAction(
                            sample0Net.build(),
                            LinearSlidesPID(TARGET,0.12),
                            new ExtendoIn(),
                            new upOuttakeServos()
                    )
            );

            robot.extendo.setPower(-0.2);

            Actions.runBlocking(
                    new SequentialAction(
                            new downOuttakeServos(),
                            Wait(SAMPLE_SCORE_TIME)
                    )
            );





            TARGET = LINEAR_SLIDES_LOWER;
            robot.claw.setPosition(CLAW_OPEN);
            sleep( 350);

            Actions.runBlocking(
                    new ParallelAction(
                            firstSamplePickup.build(),
                            new SampleHoverServos(),
                            new SequentialAction(
                                    Wait(SAMPLE_DOWN_TIME),
                                    LinearSlidesPID(TARGET,-0.12),
                                    new ExtendoOut()
                            )
                    )

            );

            sleep(100);


            Actions.runBlocking(new ServosPickupSample());
            TARGET = HIGH_SAMPLE_POS_TELE;

            Actions.runBlocking(
                    new ParallelAction(
                            firstSampleNet.build(),
                            LinearSlidesPID(TARGET,0.12),
                            new ExtendoIn(),
                            new upOuttakeServos()
                    )
            );

            robot.extendo.setPower(-0.2);

            Actions.runBlocking(
                    new SequentialAction(
                            new downOuttakeServos(),
                            Wait(SAMPLE_SCORE_TIME)
                    )
            );




            TARGET = LINEAR_SLIDES_LOWER;
            robot.claw.setPosition(CLAW_OPEN);
            sleep( 350);

            Actions.runBlocking(
                    new ParallelAction(
                            secondSamplePickup.build(),
                            new SampleHoverServos(),
                            new SequentialAction(
                                    Wait(SAMPLE_DOWN_TIME),
                                    LinearSlidesPID(TARGET,-0.12),
                                    new ExtendoOut()
                            )
                    )

            );

            sleep(100);


            Actions.runBlocking(new ServosPickupSample());

            TARGET = HIGH_SAMPLE_POS_TELE;

            Actions.runBlocking(
                    new ParallelAction(
                            secondSampleNet.build(),
                            LinearSlidesPID(TARGET,0.12),
                            new ExtendoIn(),
                            new upOuttakeServos()
                    )
            );
            robot.extendo.setPower(-0.2);

            Actions.runBlocking(
                    new SequentialAction(
                            new downOuttakeServos(),
                            Wait(SAMPLE_SCORE_TIME)
                    )
            );
            TARGET = LINEAR_SLIDES_LOWER;
            robot.claw.setPosition(CLAW_OPEN);
            sleep( 350);

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    Wait(SAMPLE_DOWN_TIME),
                                    LinearSlidesPID(TARGET,-0.12)
                            ),
                            thirdSamplePickup.build(),
                            new upOuttakeServos()
                    )
            );


            sleep( 100);
            robot.extendo.setPower(0.3);

            Actions.runBlocking(
                    new SequentialAction(
                            new Sample3HoverServos(),
                            new ExtendoOut(),
                            Wait(SAMPLE_DOWN_TIME),
                            new ServosPickupSample3()
                    )
            );

            TARGET = HIGH_SAMPLE_POS_TELE;

            Actions.runBlocking(
                    new ParallelAction(
                            thirdSampleNet.build(),
                            LinearSlidesPID(TARGET,0.12),
                            new ExtendoIn(),
                            new upOuttakeServos()
                    )
            );

            robot.extendo.setPower(-0.2);

            Actions.runBlocking(
                    new SequentialAction(
                            new downOuttakeServos(),
                            Wait(SAMPLE_SCORE_TIME)
                    )
            );
            robot.claw.setPosition(CLAW_OPEN);

            sleep( 350);

            TARGET = LINEAR_SLIDES_HOVER_LIMELIGHT;


            Actions.runBlocking(
                    new SequentialAction(
                        new ParallelAction(
                                sub.build(),
                                new SequentialAction(
                                        new SampleHoverServos_Limelight(),
                                        LinearSlidesPID(TARGET,0.08)
                                )
                        ),
                        new ExtendoOut()
                    )
            );
            while (!llTracker.Track() && getRuntime() - startStamp < TIME_ALLOWED){
                TARGET = LINEAR_SLIDES_PICKUP_LIMELIGHT;
            }
            stopstrafe();
            llTracker.Stop();
            llTracker.getSampleAngle();
            TARGET = LINEAR_SLIDES_PICKUP_LIMELIGHT;


            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new ServosPickupSample_Limelight(),
                                    new SequentialAction(
                                            Wait(SAMPLE_SCORE_TIME),
                                            LinearSlidesPID(TARGET,-0.08)
                                    )
                            ),
                            Wait(SAMPLE_SCORE_TIME),
                            new ParkServos(),
                            new ExtendoIn(),
                            new upOuttakeServos()
                    )
            );
            TARGET = HIGH_SAMPLE_POS_TELE;
            LINEAR_SLIDES_UP = 5.0;
            Actions.runBlocking(
                new ParallelAction(
                        subScore.build(),
                        new SequentialAction(
                                Wait(LINEAR_SLIDES_SUB),
                                LinearSlidesPID(TARGET,0.12)
                        )
                )
            );


            Actions.runBlocking(
                    new SequentialAction(
                            new downOuttakeServos(),
                            Wait(SAMPLE_SCORE_TIME)
                    )
            );
            robot.claw.setPosition(CLAW_OPEN);
            sleep(350);
            TARGET = AUTO_PARK_SLIDE_POS;
            Actions.runBlocking(
                    new ParallelAction(
                            park.build(),
                            new ParkServos(),
                            new SequentialAction(
                                    Wait(SAMPLE_DOWN_TIME),
                                    LinearSlidesPID(TARGET,0),
                                    Wait(PARK_TIME),
                                    LinearSlidesPID(TARGET,-0.5)
                            )

                    )
            );




        }


    }
}
