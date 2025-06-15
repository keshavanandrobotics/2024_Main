package teamcode.Autonomous.UsedAutons;


import static teamcode.Autonomous.Disabled.Poses.AUTON_START_POSE;
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

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;

@Config
@Autonomous (preselectTeleOp = "Drive_V3")
public class SampleAuton extends LinearOpMode{

    Robot robot;

    public static int SPEED = 30;
    public static int PARKSPEED = 170;

    public int TARGET = 0;
    public static int EXTENDO_SAMPLE_PICKUP = 20000;

    public static int LINEAR_SLIDES_LOWER = 1500;

    public static double SAMPLE_DOWN_TIME = 0.5;
    public static double SAMPLE_SCORE_TIME = 0.25;
    public static double EXTENDO_OUT = 1.5;

    public static double SAMPLE_NET_X1 = 9, SAMPLE_NET_Y1 = 20;
    public static double SAMPLE_NET_X2 = 5, SAMPLE_NET_Y2 = 20;
    public static double SAMPLE_NET_HEADING = -45;
    public static double SAMPLE_X1 = 14, SAMPLE_Y1 = 9;
    public static double SAMPLE_X2 = 14, SAMPLE_Y2 = 20;
    public static double SAMPLE_X3 = 37, SAMPLE_Y3 = 6, SAMPLE_HEADING3 = 90;
    public static double PARK_X1 = 56, PARK_Y1 = 0;
    public static double PARK_X2 = 56, PARK_Y2 = -30;
    public static double PARK_HEADING = -90;

    public Action limelightTracker (double x_coordinate, double y_coordinate, double heading) {
        return new Action() {
            double tx = 0.0;
            double ty = 0.0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                LLResult result = robot.limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    tx = result.getTx(); // How far left or right the target is (degrees)
                    ty = result.getTy(); // How far up or down the target is (degrees)

                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                } else {
                    telemetry.addData("Limelight", "No Targets");
                    return true;
                }

                TrajectoryActionBuilder limelightTrack = robot.drive.actionBuilder(new Pose2d(x_coordinate, y_coordinate, heading))
                        .strafeToLinearHeading(new Vector2d(x_coordinate - ty, y_coordinate - tx), heading,
                                new TranslationalVelConstraint(SPEED/2));

                Actions.runBlocking(limelightTrack.build());
                return false;

            }
        };

    }

    public Action limelightRotate (){
        return new Action() {
            double angle = 0.0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                LLResult result = robot.limelight.getLatestResult();


                double[] pythonOutputs = result.getPythonOutput();

                if (pythonOutputs != null && pythonOutputs.length > 0) {
                    angle = pythonOutputs[3];

                    robot.clawRotate.setPosition(angle / 180);


                    telemetry.addData("ANGLE", angle);
                    return false;

                } else {
                    return true;
                }
            }
        };
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

            telemetry.addData("power", power);
            telemetry.addData("Target", TARGET);
            telemetry.addData("pos", linearSlidePosition);

            if ((Math.abs(TARGET - linearSlidePosition)<LINEAR_SLIDES_LOWER) || linearSlidePosition > HIGH_SAMPLE_POS && TARGET == HIGH_SAMPLE_POS){
                telemetry.addLine("Success");
                telemetry.update();

                if (TARGET<=0){
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                } else {

                    robot.leftSlide.setPower(0.08);
                    robot.rightSlide.setPower(0.08);
                }

                return false;

            } else {


                telemetry.update();

                return  true;

            }

        }
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
            robot.clawRotate.setPosition(ROTATE_NEUTRAL);
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        robot.limelight.pipelineSwitch(0);

        TrajectoryActionBuilder sample0Net = robot.drive.actionBuilder(AUTON_START_POSE)
                .strafeToLinearHeading(new Vector2d(SAMPLE_NET_X2, SAMPLE_NET_Y2), Math.toRadians(SAMPLE_NET_HEADING));



        TrajectoryActionBuilder firstSamplePickup = robot.drive.actionBuilder(new Pose2d(SAMPLE_NET_X2, SAMPLE_NET_Y2, Math.toRadians(SAMPLE_NET_HEADING)))
                .strafeToLinearHeading(new Vector2d(SAMPLE_X1, SAMPLE_Y1), Math.toRadians(0),
                        new TranslationalVelConstraint(SPEED));

        TrajectoryActionBuilder firstSampleNet = robot.drive.actionBuilder(new Pose2d(SAMPLE_X1, SAMPLE_Y1, 0))
                .strafeToLinearHeading(new Vector2d(SAMPLE_NET_X2, SAMPLE_NET_Y2), Math.toRadians(SAMPLE_NET_HEADING));


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

        TrajectoryActionBuilder park = robot.drive.actionBuilder(new Pose2d(SAMPLE_NET_X2, SAMPLE_NET_Y2, Math.toRadians(SAMPLE_NET_HEADING)))
                .splineToSplineHeading(new Pose2d(PARK_X1, PARK_Y1,Math.toRadians(PARK_HEADING)), Math.toRadians(0),
                        new TranslationalVelConstraint(PARKSPEED))
                .splineToSplineHeading(new Pose2d(PARK_X2,PARK_Y2,Math.toRadians(PARK_HEADING)), Math.toRadians(0),
                        new TranslationalVelConstraint(PARKSPEED));

        waitForStart();

        if(isStopRequested()) return;

        if (opModeIsActive()){

            TARGET = HIGH_SAMPLE_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            sample0Net.build(),
                            new LinearSlidesPID(),
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





            TARGET = 0;
            robot.claw.setPosition(CLAW_OPEN);
            sleep( 350);

            Actions.runBlocking(
                    new ParallelAction(
                            firstSamplePickup.build(),
                            new SampleHoverServos(),
                            new SequentialAction(
                                    Wait(SAMPLE_DOWN_TIME),
                                    new LinearSlidesPID(),
                                    new ExtendoOut()
                            )
                    )

            );

            sleep(100);

            Actions.runBlocking(new ServosPickupSample());

            TARGET = HIGH_SAMPLE_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            firstSampleNet.build(),
                            new LinearSlidesPID(),
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




            TARGET = 0;
            robot.claw.setPosition(CLAW_OPEN);
            sleep( 350);

            Actions.runBlocking(
                    new ParallelAction(
                            secondSamplePickup.build(),
                            new SampleHoverServos(),
                            new SequentialAction(
                                    Wait(SAMPLE_DOWN_TIME),
                                    new LinearSlidesPID(),
                                    new ExtendoOut()
                            )
                    )

            );
            sleep( 100);


            Actions.runBlocking(new ServosPickupSample());

            TARGET = HIGH_SAMPLE_POS;

            Actions.runBlocking(
                        new ParallelAction(
                                secondSampleNet.build(),
                                new LinearSlidesPID(),
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
            TARGET = 0;
            robot.claw.setPosition(CLAW_OPEN);
            sleep( 350);

            Actions.runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                        Wait(SAMPLE_DOWN_TIME),
                                        new LinearSlidesPID()
                                ),
                                thirdSamplePickup.build(),
                                new Sample3HoverServos(),
                                new ExtendoOut()
                        )
            );


            sleep( 100);


            Actions.runBlocking(new ServosPickupSample());

            TARGET = HIGH_SAMPLE_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            thirdSampleNet.build(),
                            new LinearSlidesPID(),
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

            TARGET = AUTO_PARK_SLIDE_POS;

            Actions.runBlocking(

                    new ParallelAction(
                            park.build(),
                            new SequentialAction(

                                    new ParkServos(),

                                    new LinearSlidesPID(),
                                    Wait(EXTENDO_OUT),
                                    new ExtendoOut()

                                    )
                    )

            );



        }


    }
}
