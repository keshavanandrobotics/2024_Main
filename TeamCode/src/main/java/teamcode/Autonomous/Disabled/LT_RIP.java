package teamcode.Autonomous.Disabled;


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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;

@Config
@Disabled
@Autonomous (preselectTeleOp = "Drive_V3")
public class LT_RIP extends LinearOpMode {

    Robot robot;

    public int TARGET = 0;






    public class SampleScoreServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_SPECIMEN_SCORE);
            robot.clawRotate.setPosition(ROTATE_NEUTRAL);
            robot.clawLeftMove.setPosition(MOVE_SPECIMEN_SCORE);


            robot.claw.setPosition(CLAW_CLOSED);
            return false;
        }
    }

    public class SpecimenHoverServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_WALL_INTAKE);
            robot.clawRotate.setPosition(ROTATE_FLIP);
            robot.clawLeftMove.setPosition(MOVE_WALL_INTAKE);


            robot.claw.setPosition(CLAW_OPEN);
            return false;
        }
    }

    public class SpecimenPickupServos implements Action {

        double stamp = getRuntime();

        double ticker = 1;


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (ticker ==1){
                stamp = getRuntime();
            }

            ticker++;

            if (getRuntime() - stamp <0.21){

                robot.claw.setPosition(CLAW_CLOSED);

                robot.drive.leftFront.setPower(-0.2);
                robot.drive.rightFront.setPower(-0.2);
                robot.drive.leftBack.setPower(-0.2);
                robot.drive.rightBack.setPower(-0.2);

                return true;

            } else {

                robot.drive.leftFront.setPower(0);
                robot.drive.rightFront.setPower(0);
                robot.drive.leftBack.setPower(0);
                robot.drive.rightBack.setPower(0);


                robot.claw.setPosition(CLAW_CLOSED);
                return false;
            }



        }
    }

    public class SpecimenHoverServos2 implements Action {

        double stamp = getRuntime();

        int ticker = 1;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {




            robot.claw.setPosition(CLAW_OPEN);

            robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE + 0.04);

            robot.clawRotate.setPosition(ROTATE_FLIP);

            if (robot.claw.getPosition()<(CLAW_OPEN -0.04)){
                return true;
            } else {
                return  false;
            }

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

            if (getRuntime() - stamp < 0.19){
                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE);



                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                robot.clawRotate.setPosition(ROTATE_LM3_SPECIMEN_AUTON);

                return true;

            } else if ( getRuntime() - stamp < 0.26){

                robot.clawLeftMove.setPosition(MOVE_PICKUP_SAMPLE );
                robot.claw.setPosition(CLAW_CLOSED);
                return true;
            } else {

                robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE);
                robot.claw.setPosition(CLAW_CLOSED);
                return false;
            }



        }
    }

    public class ServosSampleDrop implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                robot.clawRotate.setPosition(ROTATE_LM3_SPECIMEN_AUTON);
                robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE + 0.0275);

                robot.claw.setPosition(CLAW_OPEN);
                return false;




        }
    }

    public class ServosSampleDrop2 implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
            robot.clawRotate.setPosition(ROTATE_LM3_SPECIMEN_AUTON);
            robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE +0.0275);

            robot.claw.setPosition(CLAW_LESS_OPEN);
            return false;




        }
    }



    public class SamplePickupServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
            robot.clawRotate.setPosition(ROTATE_LM3_SPECIMEN_AUTON);
            robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE + 0.0275);


            robot.claw.setPosition(CLAW_OPEN);
            return false;
        }
    }

    public class ExtendoOut implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            robot.extendo.setPower(1);
            return false;


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

            telemetry.addData("Ext", robot.extendo.getCurrentPosition());

            telemetry.addData("Et", robot.extendo.getCurrentPosition() > 200);

            telemetry.update();


            if (robot.extendoEncoder.getCurrentPosition()>2450){


                robot.extendo.setPower(-1);
                return true;
            } else {

                robot.extendo.setPower(0);
                return false;
            }





        }
    }

    public class ExtendoIn2 implements Action {

        double timer = 0.0;
        int ticker = 1;



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {





                robot.extendo.setPower(-1);
                return false;






        }
    }


    public class Serve implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.claw.setPosition(CLAW_OPEN);

            TARGET = -200;

            return false;






        }
    }

    public class Extra implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawLeftMove.setPosition(MOVE_HOVER_SAMPLE + 0.0275);

            return false;






        }
    }



    public class ExtendoInTemp implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                robot.extendo.setPower(-1);
                return false;







        }
    }




    public class LinearSlidesSampleScore implements Action {

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

            if (robot.drive.pose.position.x>15){
                telemetry.addLine("Success");
                telemetry.update();

                robot.leftSlide.setPower(0.12);
                robot.rightSlide.setPower(0.12);

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

    public class LinearSlidesSampleScore2 implements Action {

        private final PIDController controller = new PIDController(0.0006,0,0.00001);

        double ticker = 1;

        double stamp;





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

            if (ticker ==1){
                stamp = getRuntime();
            }



            if (robot.drive.pose.position.x>18 && Math.toDegrees(robot.drive.pose.heading.toDouble()) > -10){
                telemetry.addLine("Success");

                robot.extendo.setPower(1);
                telemetry.update();

                robot.leftSlide.setPower(0.12);
                robot.rightSlide.setPower(0.12);

                return false;

            } else {

                if (getRuntime() - stamp > 1.3){
                    robot.extendo.setPower(1);
                    telemetry.addLine("yay");
                }


                telemetry.update();

                ticker++;

                return  true;

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

            if (Math.abs(TARGET - linearSlidePosition)<475){
                telemetry.addLine("Success");
                telemetry.update();

                if (TARGET<=0){
                    robot.leftSlide.setPower(-0.1);
                    robot.rightSlide.setPower(-0.1);
                } else {

                    robot.leftSlide.setPower(0.12);
                    robot.rightSlide.setPower(0.12);
                }

                return false;

            } else {


                telemetry.update();

                return  true;

            }

        }
    }

    public class Drop implements Action {



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            if (Math.toDegrees(robot.drive.pose.heading.toDouble()) > -42 ) {


                robot.claw.setPosition(CLAW_OPEN);

                return false;

            } else {
                return true;
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
        robot.clawLeftMove.setPosition(MOVE_AUTONOMOUS_INIT);
        robot.clawPivot.setPosition(PIVOT_AUTONOMOUS_INIT);
        robot.clawRotate.setPosition(ROTATE_NEUTRAL);

        //TRAJECTORIES





        TrajectoryActionBuilder trajectory2 = robot.drive.actionBuilder(new Pose2d(0,0,0))
                .strafeToLinearHeading(new Vector2d(14.5, -21.7), Math.toRadians(-38));
        TrajectoryActionBuilder trajectory3 = robot.drive.actionBuilder(new Pose2d(14.5,-22.7,Math.toRadians(-38)))
                .strafeToLinearHeading(new Vector2d(14.5, -27.45), Math.toRadians(-120));

        TrajectoryActionBuilder trajectory4 = robot.drive.actionBuilder(new Pose2d(14.5,-27.45,Math.toRadians(-120)))
                .strafeToLinearHeading(new Vector2d(12.05, -32.65), Math.toRadians(-38));

        TrajectoryActionBuilder trajectory5 = robot.drive.actionBuilder(new Pose2d(12.05,-32.65,Math.toRadians(-38)))
                .strafeToLinearHeading(new Vector2d(14.5, -22.45), Math.toRadians(-120));

        TrajectoryActionBuilder trajectory6 = robot.drive.actionBuilder(new Pose2d(14.5,-22.45,Math.toRadians(-120)))
                .strafeToLinearHeading(new Vector2d(14.4, -41.5), Math.toRadians(-38));

        TrajectoryActionBuilder trajectory7 = robot.drive.actionBuilder(new Pose2d(14.4,-41.5,Math.toRadians(-38)))
                .strafeToLinearHeading(new Vector2d(14.5, -17.45), Math.toRadians(-120));
        TrajectoryActionBuilder trajectory8 = robot.drive.actionBuilder(new Pose2d(14.5,-17.5,Math.toRadians(0)))
                .strafeToConstantHeading(new Vector2d(4, -32));
        TrajectoryActionBuilder trajectory9a = robot.drive.actionBuilder(new Pose2d(4,-32,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(24, 8), Math.toRadians(0));


        TrajectoryActionBuilder trajectory9b = robot.drive.actionBuilder(new Pose2d(1.6,-32,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(24, 7), Math.toRadians(0));

        TrajectoryActionBuilder trajectory9c = robot.drive.actionBuilder(new Pose2d(1.6,-32,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(24, 6), Math.toRadians(0));
        TrajectoryActionBuilder trajectory9d = robot.drive.actionBuilder(new Pose2d(1.6,-32,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(24, 5), Math.toRadians(0));
        TrajectoryActionBuilder trajectory9e = robot.drive.actionBuilder(new Pose2d(1.6,-32,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(24, 4), Math.toRadians(0));



        TrajectoryActionBuilder trajectory10a = robot.drive.actionBuilder(new Pose2d(24,8,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(1.6, -32), Math.toRadians(0));

        TrajectoryActionBuilder trajectory10b = robot.drive.actionBuilder(new Pose2d(24,7,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(1.6, -32), Math.toRadians(0));

        TrajectoryActionBuilder trajectory10c = robot.drive.actionBuilder(new Pose2d(24,6,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(1.6, -32), Math.toRadians(0));

        TrajectoryActionBuilder trajectory10d = robot.drive.actionBuilder(new Pose2d(24,5,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(1.6, -32), Math.toRadians(0));





        TrajectoryActionBuilder trajectory11 = robot.drive.actionBuilder(new Pose2d(24,4,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(1.8, -37), Math.toRadians(0),
        new TranslationalVelConstraint(125));

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

        if(opModeIsActive()){



            robot.claw.setPosition(CLAW_OPEN);



            Actions.runBlocking(
                    new ParallelAction(
                            new SamplePickupServos(),
                            new LinearSlidesPID(),
                            new ExtendoOut(),


                            trajectory2.build()

                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new SequentialAction(
                                    new ServosPickupSample(),
                                    trajectory3.build()
                            ),
                            new ParallelAction(
                                    new ServosSampleDrop(),
                                    new Extra(),
                                    trajectory4.build()
                            )

                    )


            );

            sleep(550);

            Actions.runBlocking(

                    new SequentialAction(
                            new SequentialAction(
                                    new ServosPickupSample(),
                                    trajectory5.build()
                            ),
                            new ParallelAction(
                                    new ServosSampleDrop(),
                                    new Extra(),
                                    trajectory6.build()
                            ),
                            new SequentialAction(
                                    new ServosPickupSample(),
                                    trajectory7.build(),
                                    new SpecimenHoverServos2()
                            )
                    )

            );

            Actions.runBlocking(new SpecimenHoverServos2());

            Actions.runBlocking(

                    new SequentialAction(
                            new ParallelAction(
                                    new ExtendoIn(),
                                    new SpecimenHoverServos(),
                                    trajectory8.build()
                            )

                    )


            );

            Actions.runBlocking(new ExtendoIn2());

            sleep (40);










            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new SampleScoreServos(),
                                    new LinearSlidesSampleScore2(),
                                    trajectory9a.build()
                            ),
                            new Serve(),
                            new ParallelAction(
                                    new SequentialAction(
                                            new ExtendoIn(),
                                            new ParallelAction(
                                                    new SpecimenHoverServos(),
                                                    new LinearSlidesPID()
                                            )
                                    ),
                                    trajectory10a.build()

                            ),
                            new ExtendoIn2()



                    )

            );

            sleep (40);




            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new SampleScoreServos(),
                                    new LinearSlidesSampleScore2(),
                                    trajectory9b.build()
                            ),
                            new Serve(),
                            new ParallelAction(
                                    new SequentialAction(
                                            new ExtendoIn(),
                                            new ParallelAction(
                                                    new SpecimenHoverServos(),
                                                    new LinearSlidesPID()
                                            )
                                    ),
                                    trajectory10b.build()


                            ),
                            new ExtendoIn2()



                    )

            );



            sleep (40);


            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new SampleScoreServos(),
                                    new LinearSlidesSampleScore2(),
                                    trajectory9c.build()
                            ),
                            new Serve(),
                            new ParallelAction(
                                    new SequentialAction(
                                            new ExtendoIn(),
                                            new ParallelAction(
                                                    new SpecimenHoverServos(),
                                                    new LinearSlidesPID()
                                            )
                                    ),
                                    trajectory10c.build()

                            ),
                            new ExtendoIn2()



                    )

            );


            sleep (40);



            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new SampleScoreServos(),
                                    new LinearSlidesSampleScore2(),
                                    trajectory9d.build()
                            ),
                            new Serve(),
                            new ParallelAction(
                                    new SequentialAction(
                                            new ExtendoIn(),
                                            new ParallelAction(
                                                    new SpecimenHoverServos(),
                                                    new LinearSlidesPID()
                                            )
                                    ),
                                    trajectory10d.build()

                            ),
                            new ExtendoIn2()



                    )

            );

            sleep (40);


            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new SampleScoreServos(),
                                    new LinearSlidesSampleScore2(),
                                    trajectory9e.build()
                            ),
                            new Serve(),
                            new ParallelAction(
                                    new SequentialAction(
                                            new ExtendoIn(),
                                            new ParallelAction(
                                                    new SpecimenHoverServos(),
                                                    new LinearSlidesPID()
                                            )
                                    ),
                                    trajectory11.build()

                            ),
                            new ExtendoIn2()



                    )

            );



            sleep(150000000);

        }



    }
}
