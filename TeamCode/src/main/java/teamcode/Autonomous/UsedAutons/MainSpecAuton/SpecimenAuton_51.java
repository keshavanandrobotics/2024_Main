package teamcode.Autonomous.UsedAutons.MainSpecAuton;

import static teamcode.Autonomous.Disabled.Poses.*;
import static teamcode.Autonomous.UsedAutons.MainSpecAuton.SPEC_AUTO_VARS.*;
import static teamcode.Teleop.Singletons.VARS.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import teamcode.Autonomous.Disabled.MTI;
import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;

@Config
@Autonomous (preselectTeleOp = "Drive_V4")
public class SpecimenAuton_51 extends LinearOpMode {

    Robot robot;





    public TranslationalVelConstraint VEL_CONSTRAINT = new TranslationalVelConstraint(MAX_PUSHING_VEL);
    public ProfileAccelConstraint ACCEL_CONSTRAINT = new ProfileAccelConstraint(-Math.abs(MAX_PUSHING_DECCEL), MAX_PUSHING_ACCEL);

    public TranslationalVelConstraint VEL_CONSTRAINT2 = new TranslationalVelConstraint(MAX_CYCLING_VEL);
    public ProfileAccelConstraint ACCEL_CONSTRAINT2 = new ProfileAccelConstraint(-Math.abs(MAX_CYCLING_DECCEL), MAX_CYCLING_ACCEL);

    public TranslationalVelConstraint VEL_CONSTRAINT3 = new TranslationalVelConstraint(MAX_SAMPLE_VEL);
    public ProfileAccelConstraint ACCEL_CONSTRAINT3 = new ProfileAccelConstraint(-Math.abs(MAX_SAMPLE_DECCEL), MAX_SAMPLE_ACCEL);



    public Action ExtendoPID(double power){

        return new Action() {



            final double finalPower = power;




            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                robot.extendo.setPower(finalPower);
                return false;
            }

        };
    }





    public Action Servos(double clawPos, double rotatePos, double movePos, double pivotPos){

        return new Action() {

            boolean claw = true;
            boolean rotate = true;
            boolean move = true;
            boolean pivot = true;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (clawPos == 0.501){
                    claw = false;
                } else {


                    robot.claw.setPosition(clawPos);


                }
                if (rotatePos == 0.501){
                    rotate = false;
                } else {




                    robot.clawRotate.setPosition(rotatePos);
                }
                if (movePos== 0.501){
                    move = false;
                } else {




                    robot.clawLeftMove.setPosition(movePos);
                    robot.clawRightMove.setPosition(1-movePos);
                }
                if (pivotPos== 0.501){
                    pivot = false;
                } else {

                    robot.clawPivot.setPosition(pivotPos);
                }




                return false;




            }
        };


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

    public Action Wait (double time){
        return new Action() {

            int ticker = 1;

            double stamp = 0.0;

            final double desiredTime = time;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (ticker ==1){
                    stamp = getRuntime();
                }

                ticker ++;

                double runTime = getRuntime() - stamp;

                return (runTime < desiredTime);


            }

        };
    }







    @Override
    public void runOpMode() throws InterruptedException {

        //Initialization:

        robot = new Robot(hardwareMap);


        robot.drive = new PinpointDrive(hardwareMap, AUTON_START_POSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Motor Inits:

        robot.linearSlideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlideEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.claw.setPosition(CLAW_CLOSED);
        robot.clawLeftMove.setPosition(MOVE_AUTONOMOUS_INIT);
        robot.clawRightMove.setPosition(1-MOVE_AUTONOMOUS_INIT);
        robot.clawPivot.setPosition(PIVOT_AUTONOMOUS_INIT);
        robot.clawRotate.setPosition(ROTATE_FLIP);

        robot.rightStabilizer.setPosition(RIGHT_HOLD_OFF);
        robot.leftStabilizer.setPosition(LEFT_HOLD_OFF);
        robot.leftPTO.setPosition(LEFT_PTO_OFF);
        robot.rightPTO.setPosition(RIGHT_PTO_OFF);

        //Trajectories


        TrajectoryActionBuilder score0 = robot.drive.actionBuilder(AUTON_START_POSE)
                .strafeToLinearHeading(new Vector2d(FIRST_SPEC_SCORE_X, FIRST_SPEC_SCORE_Y), 0, VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder pickup1 = robot.drive.actionBuilder(new Pose2d(FIRST_SPEC_SCORE_X, FIRST_SPEC_SCORE_Y, 0))
                .strafeToLinearHeading(new Vector2d(PICKUP_X1, PICKUP_Y1), Math.toRadians(PICKUP_HEADING), VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder drop1 = robot.drive.actionBuilder(new Pose2d(PICKUP_X1, PICKUP_Y1, Math.toRadians(PICKUP_HEADING)))
                .strafeToLinearHeading(new Vector2d(DROP_X1, DROP_Y1), Math.toRadians(DROP_HEADING), VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder pickup2 = robot.drive.actionBuilder(new Pose2d(DROP_X1, DROP_Y1, Math.toRadians(DROP_HEADING)))
                .strafeToLinearHeading(new Vector2d(PICKUP_X2, PICKUP_Y2), Math.toRadians(PICKUP_HEADING), VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder drop2 = robot.drive.actionBuilder(new Pose2d(PICKUP_X2, PICKUP_Y2, Math.toRadians(PICKUP_HEADING)))
                .strafeToLinearHeading(new Vector2d(DROP_X2, DROP_Y2), Math.toRadians(DROP_HEADING), VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder pickup3 = robot.drive.actionBuilder(new Pose2d(DROP_X2, DROP_Y2, Math.toRadians(DROP_HEADING)))
                .strafeToLinearHeading(new Vector2d(PICKUP_X3, PICKUP_Y3), Math.toRadians(PICKUP_HEADING), VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder drop3 = robot.drive.actionBuilder(new Pose2d(PICKUP_X3, PICKUP_Y3, Math.toRadians(PICKUP_HEADING)))
                .strafeToLinearHeading(new Vector2d(DROP_X3, DROP_Y3), Math.toRadians(DROP_HEADING), VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder firstWallGrab = robot.drive.actionBuilder(new Pose2d(DROP_X3, DROP_Y3, DROP_HEADING))
                .strafeToLinearHeading(new Vector2d(FIRST_WALL_GRAB_X, FIRST_WALL_GRAB_Y), 0, VEL_CONSTRAINT2,ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder firstScore = robot.drive.actionBuilder(new Pose2d(FIRST_WALL_GRAB_X, FIRST_WALL_GRAB_Y, 0))
                .strafeToLinearHeading(new Vector2d(SPEC_SCORE_X2,SPEC_SCORE_Y2), Math.toRadians(SPEC_SCORE_HEADING2),VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder subsequentWallGrabs = robot.drive.actionBuilder(new Pose2d(SPEC_SCORE_X2, SPEC_SCORE_Y2, Math.toRadians(SPEC_SCORE_HEADING2)))
                .strafeToLinearHeading(new Vector2d(WALL_GRAB_X,WALL_GRAB_Y), Math.toRadians(0),VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder subsequentScores = robot.drive.actionBuilder(new Pose2d(WALL_GRAB_X,WALL_GRAB_Y,0))
                .strafeToLinearHeading(new Vector2d(SPEC_SCORE_X2,SPEC_SCORE_Y2), Math.toRadians(SPEC_SCORE_HEADING2),VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder lastWallGrab = robot.drive.actionBuilder(new Pose2d(SPEC_SCORE_X2, SPEC_SCORE_Y2, Math.toRadians(SPEC_SCORE_HEADING2)))
                .strafeToLinearHeading(new Vector2d(FIRST_WALL_GRAB_X, FIRST_WALL_GRAB_Y),0, VEL_CONSTRAINT2,ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder sampleScore = robot.drive.actionBuilder(new Pose2d(FIRST_WALL_GRAB_X, FIRST_WALL_GRAB_Y, 0))
                .strafeToLinearHeading(new Vector2d(SAMPLE_SCORE_X1, SAMPLE_SCORE_Y1), Math.toRadians(SAMPLE_SCORE_TRAVELHEADING), VEL_CONSTRAINT3,ACCEL_CONSTRAINT3)
                .strafeToLinearHeading(new Vector2d(SAMPLE_SCORE_X2, SAMPLE_SCORE_Y2), Math.toRadians(SAMPLE_SCORE_TRAVELHEADING), VEL_CONSTRAINT3,ACCEL_CONSTRAINT3)
                .strafeToLinearHeading(new Vector2d(SAMPLE_SCORE_X3, SAMPLE_SCORE_Y3), Math.toRadians(SAMPLE_SCORE_HEADING), VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);


        robot.drive.rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.drive.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.drive.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.drive.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        while(opModeInInit()){


            telemetry.addData("AUTON IS RED", AUTON_RED);

            telemetry.addData("TAKE YELLOWS", !SPEC_MODE);

            if (gamepad1.b){
                AUTON_RED = true;
            }

            if (gamepad1.x){
                AUTON_RED = false;
            }

            if (gamepad1.a){
                SPEC_MODE = true;
            }

            if (gamepad1.y){
                SPEC_MODE = false;
            }

            telemetry.update();
        }



        if (isStopRequested()) return;

        if (opModeIsActive()){

            Actions.runBlocking(
                    new ParallelAction(
                            score0.build(),
                            Servos(CLAW_CLOSED, ROTATE_NEUTRAL, MOVE_SPECIMEN_SCORE, PIVOT_SPECIMEN_SCORE),
                            new SequentialAction(
                                    LinearSlidePID(HIGH_SPECIMEN_POS, 0.12),
                                    ExtendoPID(1),
                                    Wait(EXTENDO_ALL_OUT),
                                    Servos(CLAW_OPEN, 0.501, 0.501, 0.501),
                                    ExtendoPID(-1),
                                    Wait(EXTENDO_FIRST_WAIT)
                            )
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            pickup1.build(),
                            LinearSlidePID(LINEAR_SLIDE_LOWER_THRESHOLD, -0.12),
                            Servos(0.501, ROTATE_AUTON_SPEC_SCORE, MOVE_HOVER_SAMPLE, PIVOT_SAMPLE_PICKUP),
                            new SequentialAction(
                                    Wait(EXTENDO_PICKUP_WAIT),
                                    ExtendoPID(1),
                                    Wait(EXTENDO_ALL_OUT),
                                    ExtendoPID(-0.2)
                            )
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            Servos(0.501,ROTATE_AUTON_SPEC_SCORE, MOVE_PICKUP_SAMPLE, 0.501),
                            Wait(CLAW_DOWN_TIME),
                            Servos(CLAW_CLOSED, ROTATE_AUTON_SPEC_SCORE,0.501,0.501),
                            Wait(CLAW_CLOSE_TIME),
                            new ParallelAction(
                                    drop1.build(),
                                    Servos(0.501,ROTATE_AUTON_SPEC_SCORE,MOVE_HOVER_SAMPLE,0.501)
                            ),
                            Servos(CLAW_OPEN,ROTATE_AUTON_SPEC_SCORE,0.501,0.501),
                            Wait(CLAW_OPEN_TIME),
                            pickup2.build(),
                            Servos(0.501,ROTATE_AUTON_SPEC_SCORE, MOVE_PICKUP_SAMPLE, 0.501),
                            Wait(CLAW_DOWN_TIME),
                            Servos(CLAW_CLOSED, ROTATE_AUTON_SPEC_SCORE,0.501,0.501),
                            Wait(CLAW_CLOSE_TIME),
                            new ParallelAction(
                                    drop2.build(),
                                    Servos(0.501,ROTATE_AUTON_SPEC_SCORE,MOVE_HOVER_SAMPLE,0.501)
                            ),
                            Servos(CLAW_OPEN,ROTATE_AUTON_SPEC_SCORE,0.501,0.501),
                            Wait(CLAW_OPEN_TIME),
                            pickup3.build(),
                            Servos(0.501,ROTATE_AUTON_SPEC_SCORE, MOVE_PICKUP_SAMPLE, 0.501),
                            Wait(CLAW_DOWN_TIME),
                            Servos(CLAW_CLOSED, ROTATE_AUTON_SPEC_SCORE,0.501,0.501),
                            Wait(CLAW_CLOSE_TIME),
                            new ParallelAction(
                                    drop3.build(),
                                    new SequentialAction(
                                            ExtendoPID(-1),
                                            Wait(EXTENDO_IN_WAIT),
                                            ExtendoPID(0)
                                    ),
                                    Servos(0.501,ROTATE_AUTON_SPEC_SCORE,MOVE_HOVER_SAMPLE,0.501)
                            ),
                            Servos(CLAW_OPEN,ROTATE_AUTON_SPEC_SCORE,0.501,0.501),
                            Wait(CLAW_OPEN_TIME)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ExtendoPID(-1),
                                    firstWallGrab.build(),
                                    new SequentialAction(
                                            Servos(0.501, ROTATE_FLIP, MOVE_RAISED, PIVOT_WALL_INTAKE),
                                            Wait(MOVE_WAIT),
                                            Servos(0.501,0.501,MOVE_WALL_INTAKE,0.501)
                                    )
                            ),
                            Wait(HUMAN_PLAYER_WAIT),
                            Servos(CLAW_CLOSED, 0.501, 0.501, .501),
                            Wait(CLAW_CLOSE_TIME)
                    )
            );



            Actions.runBlocking(
                    new ParallelAction(
                            firstScore.build(),
                            Servos(0.501, ROTATE_AUTON_SPEC_SCORE, MOVE_SPECIMEN_SCORE, PIVOT_SPECIMEN_SCORE),
                            LinearSlidePID(HIGH_SPECIMEN_POS, 0.12),
                            new SequentialAction(
                                    Wait(EXTENDO_OUT_WAIT),
                                    ExtendoPID(1)
                            )
                    )
            );

            for (int i = 0; i<3; i++) {

                Actions.runBlocking(
                        new SequentialAction(
                                Servos(CLAW_OPEN, 0.501, 0.501, 0.501),
                                Wait(CLAW_OPEN_TIME),
                                new ParallelAction(
                                        subsequentWallGrabs.build(),
                                        new SequentialAction(
                                                ExtendoPID(-1),
                                                Wait(EXTENDO_RETRACT_WAIT),
                                                new ParallelAction(
                                                        LinearSlidePID(LINEAR_SLIDE_LOWER_THRESHOLD, -0.12),
                                                        Servos(CLAW_OPEN, ROTATE_FLIP, MOVE_WALL_INTAKE, PIVOT_WALL_INTAKE),
                                                        ExtendoPID(-0.5)

                                                )
                                        )
                                )

                        )


                );

                //Actions.runBlocking(
                //        new SequentialAction(
                //                Wait(HUMAN_PLAYER_WAIT),
                //                ExtendoPID(EXTENDO_GRAB_THRESHOLD, 1, 1),
                //                Wait(EXTENDO_IN_WAIT),
                //                Servos(CLAW_CLOSED, 0.501, 0.501, .501),
                //                Wait(CLAW_CLOSE_TIME)
                //        )
                //);

                //while (!limitClawChecker()){
                //    Actions.runBlocking(
                //            new ParallelAction(
                //                    ExtendoPID(EXTENDO_GRAB_THRESHOLD, 0, 0),
                //                    Servos(CLAW_OPEN, 0.501,0.501,0.501),
                //                    new SequentialAction(
                //                            Wait(CLAW_OPEN_TIME),
                //                            retrySpec.build(),
                //                            Wait(HUMAN_PLAYER_WAIT),
                //                            ExtendoPID(EXTENDO_GRAB_THRESHOLD, 1, 1),
                //                            Wait(EXTENDO_IN_WAIT),
                //                            Servos(CLAW_CLOSED, 0.501, 0.501, .501),
                //                            Wait(CLAW_CLOSE_TIME)
                //                    )
                //            )
                //    );
                //}

                Actions.runBlocking(
                        new SequentialAction(
                                Wait(HUMAN_PLAYER_WAIT),
                                ExtendoPID(-1),
                                Wait(EXTENDO_IN_WAIT),
                                Servos(CLAW_CLOSED, 0.501, 0.501, .501),
                                Wait(CLAW_CLOSE_TIME),
                                new ParallelAction(
                                        subsequentScores.build(),
                                        Servos(0.501, ROTATE_AUTON_SPEC_SCORE, MOVE_SPECIMEN_SCORE, PIVOT_SPECIMEN_SCORE),
                                        LinearSlidePID(HIGH_SPECIMEN_POS, 0.12),
                                        new SequentialAction(
                                                Wait(EXTENDO_OUT_WAIT),
                                                ExtendoPID(1)
                                        )

                                )
                        )
                );
            }


            Actions.runBlocking(
                    new SequentialAction(
                            Servos(CLAW_OPEN, 0.501, 0.501, 0.501),
                            Wait(CLAW_OPEN_TIME),
                            new ParallelAction(
                                    lastWallGrab.build(),
                                    new SequentialAction(
                                            ExtendoPID(-1),
                                            Wait(EXTENDO_RETRACT_WAIT),
                                            new ParallelAction(
                                                    LinearSlidePID(LINEAR_SLIDE_LOWER_THRESHOLD, -0.12),
                                                    Servos(CLAW_OPEN, ROTATE_FLIP, MOVE_WALL_INTAKE, PIVOT_WALL_INTAKE),
                                                    ExtendoPID(-0.2)
                                            )
                                    )
                            )

                    )


            );

            Actions.runBlocking(
                    new SequentialAction(
                            Servos(CLAW_CLOSED, 0.501,0.501,0.501),
                            Wait(CLAW_CLOSE_TIME),
                            new ParallelAction(
                                    sampleScore.build(),
                                    new SequentialAction(
                                            LinearSlidePID(3000,0),
                                            Wait(LEAVE_OBS),
                                            new ParallelAction(
                                                    Servos(0.501, ROTATE_90, MOVE_RAISED, PIVOT_OUTTAKE),
                                                    LinearSlidePID(HIGH_SAMPLE_POS, 0.2)
                                            )
                                    )
                            )
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            Servos(0.501,0.501,MOVE_OUTTAKE, 0.501),
                            Wait(SAMPLE_DOWN_TIME),
                            Servos(CLAW_OPEN, 0.501,0.501,0.501)
                    )
            );

            sleep(10000);




        }


    }


}