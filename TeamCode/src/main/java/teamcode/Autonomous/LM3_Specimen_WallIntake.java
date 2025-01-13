package teamcode.Autonomous;


import static teamcode.Teleop.Singletons.Positions.*;

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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;

@Config
@Autonomous (preselectTeleOp = "Drive_V2")
public class LM3_Specimen_WallIntake extends LinearOpMode{

    Robot robot;

    public int TARGET = 0;




    public class SampleScoreServos implements Action {

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

    public class SpecimenHoverServos implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            robot.clawPivot.setPosition(PIVOT_WALL_INTAKE);
            robot.clawRotate.setPosition(ROTATE_FLIP);
            robot.clawMove.setPosition(MOVE_WALL_INTAKE);


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

            if (getRuntime() - stamp <0.3){

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


    public class ServosPickupSample implements Action {

        double ticker = 1;

        double stamp;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (ticker ==1){
                 stamp = getRuntime();
            }

            ticker ++;

            if (getRuntime() - stamp < 0.25){
                robot.clawMove.setPosition(MOVE_PICKUP_SAMPLE);

                robot.clawMove.setPosition(MOVE_PICKUP_SAMPLE);

                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                robot.clawRotate.setPosition(ROTATE_LM3_SPECIMEN_AUTON);

                return true;

            } else if ( getRuntime() - stamp < 0.35){

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

    public class ServosSampleDrop implements Action {

        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                robot.clawPivot.setPosition(PIVOT_SAMPLE_PICKUP);
                robot.clawRotate.setPosition(ROTATE_LM3_SPECIMEN_AUTON);
                robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);

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
            robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);

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
            robot.clawMove.setPosition(MOVE_HOVER_SAMPLE);


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
            robot.centerSlide.setPower(power);

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
                robot.centerSlide.setPower(0.12);

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
            robot.centerSlide.setPower(power);

            telemetry.addData("power", power);
            telemetry.addData("Target", TARGET);
            telemetry.addData("pos", linearSlidePosition);

            if (Math.abs(TARGET - linearSlidePosition)<300){
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

        //TRAJECTORIES



        TrajectoryActionBuilder trajectory1 = robot.drive.actionBuilder(new Pose2d(0,0,0))

                .splineTo(new Vector2d(20,0),0);

        TrajectoryActionBuilder trajectory2 = robot.drive.actionBuilder(new Pose2d(18,0,0))
                .strafeToLinearHeading(new Vector2d(14.1, -22.95), Math.toRadians(-38));
        TrajectoryActionBuilder trajectory3 = robot.drive.actionBuilder(new Pose2d(14.1,-22.95,Math.toRadians(-38)))
                .strafeToLinearHeading(new Vector2d(14.5, -27.45), Math.toRadians(-120));

        TrajectoryActionBuilder trajectory4 = robot.drive.actionBuilder(new Pose2d(14.5,-27.45,Math.toRadians(-120)))
                .strafeToLinearHeading(new Vector2d(14.1, -32.45), Math.toRadians(-38));

        TrajectoryActionBuilder trajectory5 = robot.drive.actionBuilder(new Pose2d(14.1,-32.45,Math.toRadians(-38)))
                .strafeToLinearHeading(new Vector2d(14.5, -22.45), Math.toRadians(-120));

        TrajectoryActionBuilder trajectory6 = robot.drive.actionBuilder(new Pose2d(14.5,-22.45,Math.toRadians(-120)))
                .strafeToLinearHeading(new Vector2d(15.2, -42.45), Math.toRadians(-38));

        TrajectoryActionBuilder trajectory7 = robot.drive.actionBuilder(new Pose2d(15.2,-42.45,Math.toRadians(-38)))
                .strafeToLinearHeading(new Vector2d(14.5, -17.45), Math.toRadians(-120));
        TrajectoryActionBuilder trajectory8 = robot.drive.actionBuilder(new Pose2d(14.5,-17.5,Math.toRadians(0)))
                .strafeToConstantHeading(new Vector2d(2, -32));



        TrajectoryActionBuilder trajectory9 = robot.drive.actionBuilder(new Pose2d(3,-32,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(25, 15), Math.toRadians(0));


        TrajectoryActionBuilder trajectory10 = robot.drive.actionBuilder(new Pose2d(25,15,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(2, -32), Math.toRadians(0));





        waitForStart();

        if (isStopRequested()) return;

        if(opModeIsActive()){

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            new SampleScoreServos(),
                            new LinearSlidesSampleScore(),
                            trajectory1.build()
                    )
            );

            robot.claw.setPosition(CLAW_OPEN);

            TARGET = -200;

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new ExtendoIn(),
                                    new ParallelAction(
                                            new SamplePickupServos(),
                                            new LinearSlidesPID(),
                                            new ExtendoOut()

                            )
                            ),
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
                                    trajectory4.build()
                            ),
                            new SequentialAction(
                                    new ServosPickupSample(),
                                    trajectory5.build()
                            ),
                            new ParallelAction(
                                    new ServosSampleDrop(),
                                    trajectory6.build()
                            ),
                            new SequentialAction(
                                    new ServosPickupSample(),
                                    trajectory7.build()
                            )
                    )
            );

            Actions.runBlocking(
                            new ParallelAction(
                                    new ExtendoIn(),
                                    new SpecimenHoverServos(),
                                    trajectory8.build()
                            )


            );

            sleep( 200);

            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            new SampleScoreServos(),
                            new LinearSlidesSampleScore2(),
                            trajectory9.build()
                    )
            );

            robot.claw.setPosition(CLAW_OPEN);

            TARGET = -200;

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new ExtendoIn(),
                                    new ParallelAction(
                                            new SpecimenHoverServos(),
                                            new LinearSlidesPID()
                                    )
                            ),
                            trajectory10.build()

                    )
            );

            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            new SampleScoreServos(),
                            new LinearSlidesSampleScore2(),
                            trajectory9.build()
                    )
            );
            robot.claw.setPosition(CLAW_OPEN);

            TARGET = -200;

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new ExtendoIn(),
                                    new ParallelAction(
                                            new SpecimenHoverServos(),
                                            new LinearSlidesPID()
                                    )
                            ),
                            trajectory10.build()

                    )
            );

            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            new SampleScoreServos(),
                            new LinearSlidesSampleScore2(),
                            trajectory9.build()
                    )
            );

            robot.claw.setPosition(CLAW_OPEN);

            TARGET = -200;

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new ExtendoIn(),
                                    new ParallelAction(
                                            new SpecimenHoverServos(),
                                            new LinearSlidesPID()
                                    )
                            ),
                            trajectory10.build()

                    )
            );

            Actions.runBlocking(new SpecimenPickupServos());

            TARGET = HIGH_SPECIMEN_POS;

            Actions.runBlocking(
                    new ParallelAction(
                            new SampleScoreServos(),
                            new LinearSlidesSampleScore2(),
                            trajectory9.build()
                    )
            );








            sleep(150000000);

        }



    }
}
