package teamcode.Autonomous.Disabled;
// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import teamcode.Autonomous.RoadRunner.MecanumDrive;
import teamcode.Autonomous.RoadRunner.PinpointDrive;
import teamcode.Robot;


@Config
@Disabled
@Autonomous
public class LM2_Specimen extends LinearOpMode {



    public int position = 1200;

    public int GLOBAL_POSITION = 0;

    public double slideDownWait = 0.4;



    Robot robot;
    private static MultipleTelemetry TELE;

    public class Slides implements Action {

        double ticker = 0;



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ticker++;
            if (ticker ==1){

                position-=GLOBAL_POSITION;


                robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                robot.leftSlide.setTargetPosition(position);


                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.centerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.leftSlide.setPower(1);

                return true;
            }
            else if (robot.leftSlide.isBusy()&&robot.leftSlide.getCurrentPosition()<position+40){
                robot.centerSlide.setPower(robot.leftSlide.getPower());
                robot.rightSlide.setPower(robot.leftSlide.getPower());
                return true;
            } else {



                robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                GLOBAL_POSITION+= robot.leftSlide.getCurrentPosition();

                robot.leftSlide.setPower(0.1);






                robot.centerSlide.setPower(0.1);
                robot.rightSlide.setPower(0.1);

                return false;

            }
        }
    }

    public Action slides() {
        return new Slides();
    }


    public class SlidesDown implements Action {

        double stamp = getRuntime();





        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double timer = getRuntime() - stamp;
            if (timer< slideDownWait){
                robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return true;
            }
            else if (robot.leftSlide.getCurrentPosition()>(-GLOBAL_POSITION+40)){
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.centerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                robot.centerSlide.setPower(-1);


                return true;
            }
            else {

                GLOBAL_POSITION = 0;


                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                robot.centerSlide.setPower(0);





                return false;

            }
        }
    }

    public class ExtendedIntake implements Action {

        double stamp = getRuntime();





        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double timer = getRuntime() - stamp;
            if ((robot.leftSlide.getCurrentPosition()>(-GLOBAL_POSITION+40))&&timer<0.1){
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.centerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                robot.centerSlide.setPower(-1);


                return true;
            }
            else if ((robot.leftSlide.getCurrentPosition()>(-GLOBAL_POSITION+40))){
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.centerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.claw.setPosition(0.65);

                robot.claw.setPosition(0.65);

                robot.clawRotate.setPosition(0.95);
                robot.clawMove.setPosition(0.55);
                robot.clawPivot.setPosition(0.61);

                robot.clawRotate.setPosition(0.95);


                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                robot.centerSlide.setPower(-1);


                return true;
            }
            else {

                robot.extendo.setPower(-0.1);



                GLOBAL_POSITION = 0;


                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                robot.centerSlide.setPower(0);

                robot.claw.setPosition(0.84);
                robot.claw.setPosition(0.84);





                return false;

            }
        }
    }


    public class Intake implements Action {

        double stamp = getRuntime();


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double timer = getRuntime()- stamp;
            if (timer<4){
                robot.claw.setPosition(0.65);
                robot.claw.setPosition(0.65);
                return true;

            }
            else if(timer<4.5){
                robot.claw.setPosition(0.65);

                robot.claw.setPosition(0.65);

                robot.clawRotate.setPosition(0.95);
                robot.clawMove.setPosition(0.55);
                robot.clawPivot.setPosition(0.61);

                robot.clawRotate.setPosition(0.95);

                return true;

            } else {

                robot.claw.setPosition(0.84);
                robot.claw.setPosition(0.84);



                robot.extendo.setPower(-0.2);
                return false;

            }

        }
    }

    public class In implements Action {

        double stamp = getRuntime();


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double timer = getRuntime()- stamp;
            if (timer<0.3){
                robot.clawRotate.setPosition(0.3906);
                robot.clawRotate.setPosition(0.3906);




                return true;

            } else if (timer <1){

                robot.clawMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.claw.setPosition(0.65);
                return true;

            }

            else if (timer <2){

                robot.clawRotate.setPosition(0.95);

                robot.clawRotate.setPosition(0.95);

                robot.clawMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.claw.setPosition(0.65);
                return true;

            }
            else{
                robot.extendo.setPower(-0.3);

                robot.clawRotate.setPosition(0.95);
                robot.clawMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.claw.setPosition(0.65);

                robot.clawRotate.setPosition(0.95);
                robot.clawMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.claw.setPosition(0.65);

                return false;

            }

        }
    }

    public Action intake() {return  new Intake();}
    public Action in() {return  new In();}

    public Action slidesDown() {
        return new SlidesDown();
    }
    public Action intakeExtended() {
        return new ExtendedIntake();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap);



        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));

        robot.drive = new PinpointDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder scoreSpecimen = robot.drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(31,0), Math.toRadians(180));

        TrajectoryActionBuilder scoreLater = robot.drive.actionBuilder(new Pose2d(3, -36.5, Math.PI))
                .strafeToLinearHeading(new Vector2d(33,15), Math.toRadians(180));

        TrajectoryActionBuilder scoreLater2 = robot.drive.actionBuilder(new Pose2d(3.1, -36.5, Math.PI))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(33,8), Math.toRadians(0));

        TrajectoryActionBuilder Park = robot.drive.actionBuilder(new Pose2d(33, 8, Math.PI))
                .splineToConstantHeading(new Vector2d(6,-36.5), 0);

        TrajectoryActionBuilder pushing = robot.drive.actionBuilder(new Pose2d(31,0, Math.PI))
                .splineToConstantHeading(new Vector2d(20,-28.5),0)
                .splineToConstantHeading(new Vector2d(53, -33),0)
                .splineToConstantHeading(new Vector2d(53, -40),0)
                .splineToConstantHeading(new Vector2d(14, -40),0)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(53, -40),0)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(53, -48),0)
                .splineToConstantHeading(new Vector2d(14, -48),0)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(53, -48),0)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(53, -54),0)
                .splineToConstantHeading(new Vector2d(14, -54),0)
                .splineToConstantHeading(new Vector2d(5, -36.5),0,
                        new TranslationalVelConstraint(25))
                .splineToConstantHeading(new Vector2d(3, -36.5),0,
                new TranslationalVelConstraint(10));
        TrajectoryActionBuilder intakeLater = robot.drive.actionBuilder(new Pose2d(33,15, Math.PI))
                .splineToConstantHeading(new Vector2d(13, -36.5),Math.PI)
                .splineToConstantHeading(new Vector2d(3.1, -36.5),0,

                        new TranslationalVelConstraint(10));









        robot.clawRotate.setPosition(0.95);
        robot.clawMove.setPosition(0.8);
        robot.clawPivot.setPosition(0.12);
        robot.claw.setPosition(0.65);

        robot.clawRotate.setPosition(0.95);
        robot.clawMove.setPosition(0.8);
        robot.clawPivot.setPosition(0.12);
        robot.claw.setPosition(0.65);


        robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();

        if (opModeIsActive()) {

            robot.extendo.setPower(-0.3);


            robot.clawRotate.setPosition(0.95);
            robot.clawMove.setPosition(0.8);
            robot.clawPivot.setPosition(0.12);
            robot.claw.setPosition(0.65);

            robot.clawRotate.setPosition(0.95);
            robot.clawMove.setPosition(0.8);
            robot.clawPivot.setPosition(0.12);
            robot.claw.setPosition(0.65);




            position = 1400;

            Actions.runBlocking(
                    new ParallelAction(scoreSpecimen.build(), slides())
            );
            robot.centerSlide.setPower(0.1);
            robot.rightSlide.setPower(0.1);



            //Scoring The Specimen
            robot.extendo.setPower(-0.1);
            robot.claw.setPosition(0.67);

            position = 1900;

            Actions.runBlocking(slides());

            //Lower Slides

            robot.claw.setPosition(0.75);



            position = 0;



            Actions.runBlocking(
                    new ParallelAction( slidesDown(), pushing.build(), intake())
            );

            robot.claw.setPosition(0.65);
            robot.extendo.setPower(-0.3);

            robot.claw.setPosition(0.65);


            position = 1300;

            sleep(200);

            robot.clawRotate.setPosition(0.95);
            robot.clawMove.setPosition(0.8);
            robot.clawPivot.setPosition(0.12);
            robot.claw.setPosition(0.65);

            robot.clawRotate.setPosition(0.95);
            robot.clawMove.setPosition(0.8);
            robot.clawPivot.setPosition(0.12);
            robot.claw.setPosition(0.65);


            Actions.runBlocking(
                    new ParallelAction(scoreLater.build(), slides())
            );
            robot.centerSlide.setPower(0.1);
            robot.rightSlide.setPower(0.1);



            //Scoring The Specimen
            robot.extendo.setPower(-0.1);
            robot.claw.setPosition(0.67);

            position = 1900;

            Actions.runBlocking(slides());
            robot.claw.setPosition(0.75);

            robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Actions.runBlocking(
                    new ParallelAction(intakeLater.build(), intakeExtended())
            );
            robot.claw.setPosition(0.65);
            sleep(200);

            position = 1300;

            Actions.runBlocking(
                    new ParallelAction(scoreLater2.build(), slides(), in())
            );
            robot.centerSlide.setPower(0.1);
            robot.rightSlide.setPower(0.1);



            //Scoring The Specimen
            robot.extendo.setPower(-0.1);
            robot.claw.setPosition(0.67);

            position = 1900;

            Actions.runBlocking(slides());
            robot.claw.setPosition(0.75);

            Actions.runBlocking(
                    new ParallelAction( slidesDown(), Park.build())
            );
            sleep(5000000);
        }






    }
}

