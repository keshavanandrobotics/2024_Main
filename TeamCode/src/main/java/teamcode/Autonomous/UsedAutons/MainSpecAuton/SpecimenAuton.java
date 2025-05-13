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


import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;

@Config
@Autonomous (preselectTeleOp = "Drive_V3")
public class SpecimenAuton extends LinearOpMode {

    Robot robot;





    public TranslationalVelConstraint VEL_CONSTRAINT = new TranslationalVelConstraint(MAX_PUSHING_VEL);
    public ProfileAccelConstraint ACCEL_CONSTRAINT = new ProfileAccelConstraint(-Math.abs(MAX_PUSHING_DECCEL), MAX_PUSHING_ACCEL);

    public TranslationalVelConstraint VEL_CONSTRAINT2 = new TranslationalVelConstraint(MAX_CYCLING_VEL);
    public ProfileAccelConstraint ACCEL_CONSTRAINT2 = new ProfileAccelConstraint(-Math.abs(MAX_CYCLING_DECCEL), MAX_CYCLING_ACCEL);



    public Action ExtendoPID(int position, double power, double holdPower){

        return new Action() {

            int pos = position;

            int ticker = 1;

            double finalPower = 0.0;

            double finalHoldPower = 0.0;


            boolean reversed = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (ticker ==1){

                    if (robot.extendoEncoder.getCurrentPosition() < pos){
                        finalPower = power;

                        finalHoldPower = holdPower;

                        reversed = false;
                    } else {
                        finalPower = -power;
                        finalHoldPower = -holdPower;

                        reversed = true;

                    }



                }

                ticker++;

                if ((robot.extendoEncoder.getCurrentPosition() < pos) && !reversed){

                    robot.extendo.setPower(finalPower);



                    return true;

                }

                else if ((robot.extendoEncoder.getCurrentPosition() > pos) && reversed){

                    robot.extendo.setPower(finalPower);




                    return true;

                }


                else {

                    robot.extendo.setPower(finalHoldPower);



                    return false;


                }

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




                    robot.clawMove.setPosition(movePos);
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
                robot.centerSlide.setPower(power);

                telemetry.addData("power", power);
                telemetry.addData("Target", position);
                telemetry.addData("pos", linearSlidePosition);

                if (Math.abs(position - linearSlidePosition)<475){
                    telemetry.addLine("Success");
                    telemetry.update();


                    robot.leftSlide.setPower(holdPower);
                    robot.rightSlide.setPower(holdPower);
                    robot.centerSlide.setPower(holdPower);


                    return false;

                } else {


                    telemetry.update();

                    return  true;

                }

            }

        };
    }

    public boolean limitClawChecker (){
        return (!robot.pin00.getState() || !robot.pin01.getState());
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
        robot.clawMove.setPosition(MOVE_AUTONOMOUS_INIT);
        robot.clawPivot.setPosition(PIVOT_AUTONOMOUS_INIT);
        robot.clawRotate.setPosition(ROTATE_NEUTRAL);

        robot.rightSpringHook.setPosition(RIGHT_SPRING_OFF);
        robot.leftSpringHook.setPosition(LEFT_SPRING_OFF);

        robot.rightStabilizer.setPosition(RIGHT_HOLD_OFF);
        robot.leftStabilizer.setPosition(LEFT_HOLD_OFF);

        //Trajectories


        TrajectoryActionBuilder pushing = robot.drive.actionBuilder(AUTON_START_POSE)
                .splineToConstantHeading(new Vector2d(X1, Y1), 0, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(X2, Y2), 0, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(X3, Y3), -Math.PI / 2, VEL_CONSTRAINT, ACCEL_CONSTRAINT)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(X4, Y4), Math.PI, VEL_CONSTRAINT, ACCEL_CONSTRAINT)

                .setReversed(false)
                .splineToConstantHeading(new Vector2d(X3, Y3), 0, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(X5, Y5), -Math.PI / 2, VEL_CONSTRAINT, ACCEL_CONSTRAINT)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(X6, Y6), Math.PI, VEL_CONSTRAINT, ACCEL_CONSTRAINT)

                .setReversed(false)
                .splineToConstantHeading(new Vector2d(X5, Y5), 0, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
                .splineToConstantHeading(new Vector2d(X7, Y7), -Math.PI / 2, VEL_CONSTRAINT, ACCEL_CONSTRAINT)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(X8, Y8), Math.PI, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
                //.strafeToLinearHeading(new Vector2d(X9, Y9), 0, VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder firstWallGrab = robot.drive.actionBuilder(new Pose2d(X8, Y8, 0))
                .strafeToLinearHeading(new Vector2d(FIRST_WALL_GRAB_X, FIRST_WALL_GRAB_Y), 0, VEL_CONSTRAINT2,ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder firstScore = robot.drive.actionBuilder(new Pose2d(FIRST_WALL_GRAB_X, FIRST_WALL_GRAB_Y, 0))
                .strafeToLinearHeading(new Vector2d(SPEC_SCORE_X,SPEC_SCORE_Y), Math.toRadians(SPEC_SCORE_HEADING),VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder subsequentWallGrabs = robot.drive.actionBuilder(new Pose2d(SPEC_SCORE_X, SPEC_SCORE_Y, Math.toRadians(SPEC_SCORE_HEADING)))
                .strafeToLinearHeading(new Vector2d(WALL_GRAB_X,WALL_GRAB_Y), Math.toRadians(0),VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder subsequentScores = robot.drive.actionBuilder(new Pose2d(WALL_GRAB_X,WALL_GRAB_Y,0))
                .strafeToLinearHeading(new Vector2d(SPEC_SCORE_X,SPEC_SCORE_Y), Math.toRadians(SPEC_SCORE_HEADING),VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

        TrajectoryActionBuilder retrySpec = robot.drive.actionBuilder(new Pose2d(WALL_GRAB_X,WALL_GRAB_Y,0))
                .strafeToLinearHeading(new Vector2d(RETRY_SPEC_X,RETRY_SPEC_Y), 0, VEL_CONSTRAINT2, ACCEL_CONSTRAINT2)
                .strafeToLinearHeading(new Vector2d(WALL_GRAB_X,WALL_GRAB_Y), 0, VEL_CONSTRAINT2, ACCEL_CONSTRAINT2);

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
                            pushing.build(),
                            new SequentialAction(
                                    Wait(PUSHING_SERVO_ROTATE),
                                    Servos(0.501, ROTATE_FLIP, 0.501, 0.501),
                                    Wait(PUSHING_SERVO_OPEN),
                                    Servos(CLAW_OPEN, 0.501, 0.501, 0.501),
                                    Wait(PUSHING_SERVO_PIVOT),
                                    Servos(0.501, 0.501, 0.501, PIVOT_WALL_INTAKE),
                                    Wait(PUSHING_SERVO_MOVE),
                                    Servos(0.501, 0.501, MOVE_WALL_INTAKE, 0.501),
                                    ExtendoPID(EXTENDO_INITIAL_HUMAN_PLAYER, 1, 0)
                            )

                    )
            );

            //Actions.runBlocking(firstWallGrab.build());



            Actions.runBlocking(
                    new SequentialAction(
                            firstWallGrab.build(),
                            Wait(HUMAN_PLAYER_WAIT),
                            ExtendoPID(EXTENDO_GRAB_THRESHOLD, 1, 1),
                            Wait(EXTENDO_IN_WAIT),
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
                                            ExtendoPID(EXTENDO_SCORE_THRESHOLD, 1, 1)
                                    )
                            )
            );

            for (int i = 0; i<4; i++) {

                Actions.runBlocking(
                        new SequentialAction(
                                Servos(CLAW_OPEN, 0.501, 0.501, 0.501),
                                Wait(CLAW_OPEN_TIME),
                                new ParallelAction(
                                        subsequentWallGrabs.build(),
                                        new SequentialAction(
                                                ExtendoPID(EXTENDO_CYCLE_HUMAN_PLAYER, 1, 0),
                                                new ParallelAction(
                                                        LinearSlidePID(LINEAR_SLIDE_LOWER_THRESHOLD, -0.12),
                                                        Servos(CLAW_OPEN, ROTATE_FLIP, MOVE_WALL_INTAKE, PIVOT_WALL_INTAKE)

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
                                ExtendoPID(EXTENDO_GRAB_THRESHOLD, 1, 1),
                                Wait(EXTENDO_IN_WAIT),
                                Servos(CLAW_CLOSED, 0.501, 0.501, .501),
                                Wait(CLAW_CLOSE_TIME),
                                new ParallelAction(
                                        subsequentScores.build(),
                                        Servos(0.501, ROTATE_AUTON_SPEC_SCORE, MOVE_SPECIMEN_SCORE, PIVOT_SPECIMEN_SCORE),
                                        LinearSlidePID(HIGH_SPECIMEN_POS, 0.12),
                                        new SequentialAction(
                                                Wait(EXTENDO_OUT_WAIT),
                                                ExtendoPID(EXTENDO_SCORE_THRESHOLD, 1, 1)
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
                                    subsequentWallGrabs.build(),
                                    new SequentialAction(
                                            ExtendoPID(EXTENDO_CYCLE_HUMAN_PLAYER, 1, 0),
                                            new ParallelAction(
                                                    LinearSlidePID(LINEAR_SLIDE_LOWER_THRESHOLD, -0.12),
                                                    Servos(CLAW_OPEN, ROTATE_FLIP, MOVE_WALL_INTAKE, PIVOT_WALL_INTAKE)

                                            )
                                    )
                            )

                    )


            );

            sleep(10000);




        }


    }


}
