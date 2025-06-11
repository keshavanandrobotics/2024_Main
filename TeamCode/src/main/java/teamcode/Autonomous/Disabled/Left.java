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
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
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
public class Left extends LinearOpMode {

    public static double timeSlidesUP = 1.65;

    public class Claw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.claw.setPosition(0.6);
            robot.clawRotate.setPosition(0.3906);

            robot.clawLeftMove.setPosition(0.6);
            robot.clawPivot.setPosition(0.62);
            robot.extendo.setPower(1);
            return false;
        }
    }
    public class Slides implements Action {

        double stamp = getRuntime();



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

           if (getRuntime()-stamp<timeSlidesUP){
               robot.leftSlide.setPower(1);
               robot.rightSlide.setPower(1);
               robot.claw.setPosition(0.62);
               return true;
           } else {
               robot.leftSlide.setPower(0);
               robot.rightSlide.setPower(0);
               return false;
           }


        }
    }
    public class ides implements Action {

        double stamp = getRuntime();



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (getRuntime()-stamp<3.7){
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                robot.clawRotate.setPosition(0.3906);
                robot.clawLeftMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.clawLeftMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                return true;
            } else {
                robot.extendo.setPower(-0.1);

                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                return false;
            }


        }
    }
    public class slideDown implements Action {

        double stamp = getRuntime();



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (getRuntime()-stamp<.2){
                robot.claw.setPosition(0.84);
                robot.extendo.setPower(-1);
                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                return true;
            } else if (getRuntime() - stamp <1.2){
                robot.extendo.setPower(0);
                return true;
            }  else if (getRuntime()-stamp <1.7){
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);

                robot.extendo.setPower(1);

                robot.clawLeftMove.setPosition(0.55);

                robot.clawPivot.setPosition(0.93);
                robot.claw.setPosition(0.84);
                robot.clawRotate.setPosition(0.3906);
                return true;
            }

            else {
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                return false;
            }


        }
    }

    public class ddDown implements Action {

        double stamp = getRuntime();



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (getRuntime()-stamp<2){

                robot.claw.setPosition(0.6);
                robot.claw.setPosition(0.6);
                robot.clawLeftMove.setPosition(0.55);
                robot.clawPivot.setPosition(0.93);
                robot.clawLeftMove.setPosition(0.55);

                robot.clawPivot.setPosition(0.93);
                robot.extendo.setPower(1);
                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                robot.claw.setPosition(0.6);


                return true;
            } else if (getRuntime()-stamp<3){

                robot.claw.setPosition(0.84);
                robot.claw.setPosition(0.84);
                robot.clawLeftMove.setPosition(0.55);

                robot.clawPivot.setPosition(0.93);
                robot.clawLeftMove.setPosition(0.55);

                robot.clawPivot.setPosition(0.93);
                robot.extendo.setPower(1);
                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);

                return true;
            }

            else if (getRuntime()-stamp<3.5){
                robot.claw.setPosition(0.84);
                robot.clawRotate.setPosition(0.3906);

                robot.clawLeftMove.setPosition(0.55);
                robot.clawPivot.setPosition(0.93);
                robot.clawLeftMove.setPosition(0.55);

                robot.clawPivot.setPosition(0.93);
                robot.clawPivot.setPosition(0.93);
                robot.clawLeftMove.setPosition(0.55);
                robot.clawPivot.setPosition(0.93);

                robot.claw.setPosition(0.84);

                robot.extendo.setPower(1);
                return true;
            }
 else {

                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                robot.clawLeftMove.setPosition(0.55);
                robot.clawPivot.setPosition(0.93);
                robot.claw.setPosition(0.84);
                robot.clawLeftMove.setPosition(0.55);
                robot.clawRotate.setPosition(0.3906);

                robot.clawPivot.setPosition(0.93);
                robot.clawLeftMove.setPosition(0.55);
                robot.clawPivot.setPosition(0.93);

                return false;
            }

        }
    }

    public class s implements Action {

        double stamp = getRuntime();




        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double grabTimer = getRuntime()-stamp;
            if (grabTimer<0.1){
                robot.clawLeftMove.setPosition(0.48);
                return  true;
            }
           else if (grabTimer<0.3){
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.claw.setPosition(0.58);
               return true;



           }

            else if (grabTimer<1){

                robot.clawRotate.setPosition(0.3906);
                robot.clawLeftMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.extendo.setPower(-1);
               return true;


           } else {
               robot.extendo.setPower(0);
               return false;


           }

        }
    }




    public class sSS implements Action {

        double stamp = getRuntime();




        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double grabTimer = getRuntime()-stamp;
             if (grabTimer<0.3){
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                robot.claw.setPosition(0.58);
                return true;



            }

            else if (grabTimer<1){

                robot.clawRotate.setPosition(0.3906);
                robot.clawLeftMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.extendo.setPower(-1);
                return true;


            } else {
                robot.extendo.setPower(0);
                return false;


            }

        }
    }

    public class upUp implements Action {


        double stamp = getRuntime();



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {



            if (getRuntime()-stamp<0.4){
                robot.clawLeftMove.setPosition(0.48);
                robot.clawLeftMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);

                robot.clawPivot.setPosition(0.93);

                robot.clawLeftMove.setPosition(0.48);

                return true;


            }

            if (getRuntime()-stamp<0.6){
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.claw.setPosition(0.58);
                robot.claw.setPosition(0.58);
                    return true;


            } else if (getRuntime()-stamp<0.9){

                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);

                robot.clawRotate.setPosition(0.3906);
                robot.clawLeftMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);

                    return true;


            }

            else if (getRuntime()-stamp<1.2){
                if (robot.claw.getPosition()>0.58){
                    robot.claw.setPosition(0.58);
                }
                robot.clawRotate.setPosition(0.3906);
                robot.clawLeftMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.extendo.setPower(-1);
                return true;
            } else {
                robot.extendo.setPower(-0.1);
                return false;
            }


        }
    }

    public class downdd implements Action{
        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if ((getRuntime()-stamp<0.5)){
                return true;
            }
            if (getRuntime()-stamp<1.05) {
                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                return true;
            }   else {
                robot.leftSlide.setPower(0.1);
                robot.rightSlide.setPower(0.1);
                return false;
            }


        }
    }
    public Action close(){
        return new s();
    }

    public Action ose(){
        return new sSS();
    }
    public Action claw() {
        return new Claw();
    }

    public Action slides() {
        return new Slides();
    }

    public Action dsssss() {
        return new upUp();
    }


    public Action down() {
        return new slideDown();
    }
    public Action up() {
        return new ides();
    }

    public Action dDown() {
        return new ddDown();
    }

    public Action ddddddd() {
        return new downdd();
    }

    Robot robot;
    private static MultipleTelemetry TELE;


    @Override
    public void runOpMode() throws InterruptedException {
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap);

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        robot.drive = new PinpointDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = robot.drive.actionBuilder(initialPose)
                .waitSeconds(0.2)
                        .strafeToLinearHeading(new Vector2d(15.39,0), 0);

        TrajectoryActionBuilder trajectory1 = robot.drive.actionBuilder(new Pose2d(15.39, 0, 0))
                .strafeToLinearHeading(new Vector2d(18.5,40), 0,
                        new TranslationalVelConstraint(30));

        TrajectoryActionBuilder trajectory1e = robot.drive.actionBuilder(new Pose2d(5.4, 53, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(18.25,49), 0,
                        new TranslationalVelConstraint(20));



        TrajectoryActionBuilder trajectory2 = robot.drive.actionBuilder(new Pose2d(18.7, 42, 0))

                .strafeToLinearHeading(new Vector2d(6,47), Math.toRadians(-45));

        TrajectoryActionBuilder trajectory2e = robot.drive.actionBuilder(new Pose2d(18.45 , 51, 0))

                .strafeToLinearHeading(new Vector2d(6,47), Math.toRadians(-45));

        TrajectoryActionBuilder trajectory23 = robot.drive.actionBuilder(new Pose2d(6, 46, Math.toRadians(-45)))

                .strafeToLinearHeading(new Vector2d(5.4,53), Math.toRadians(-45),
                        new TranslationalVelConstraint(20));

        TrajectoryActionBuilder trajectorywww3 = robot.drive.actionBuilder(new Pose2d(5.4, 53, Math.toRadians(-45)))
                .splineToSplineHeading(new Pose2d(53,27,Math.toRadians(-90)), Math.toRadians(0),
                        new TranslationalVelConstraint(40))
                .splineToSplineHeading(new Pose2d(53,18,Math.toRadians(-90)), Math.toRadians(0),
                        new TranslationalVelConstraint(40));



        robot.claw.setPosition(0.6);
        robot.clawLeftMove.setPosition(0.5);
        robot.clawPivot.setPosition(0.2);
        robot.clawRotate.setPosition(0.38);

        robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);










        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        trajectory.build(),
                        new SequentialAction(
                                claw(),
                                slides()
                        )
                )
        );

        Actions.runBlocking(

                        new ParallelAction(
                                trajectory1.build(),
                                down()

                        )


        );

        Actions.runBlocking(
                new SequentialAction(
                        close(),
                        new ParallelAction(
                                up(),trajectory2.build()

                        ),
                        trajectory23.build()

                ));

        sleep(1000);



        robot.clawRotate.setPosition(0.3906);
        robot.clawLeftMove.setPosition(0.8);
        robot.clawPivot.setPosition(0.12);
        robot.claw.setPosition(0.84);

        robot.claw.setPosition(0.84);
        robot.extendo.setPower(0);

        Actions.runBlocking(

                new SequentialAction(
                        trajectory1e.build(),
                        dDown()

                )


        );
        robot.claw.setPosition(0.84);

        Actions.runBlocking(dsssss());
        Actions.runBlocking(
                new SequentialAction(
                        ose(),
                        new ParallelAction(
                                up(),trajectory2e.build()

                        ),
                        trajectory23.build()

                ));

        sleep(1000);


        robot.clawRotate.setPosition(0.3906);
        robot.clawLeftMove.setPosition(0.8);
        robot.clawPivot.setPosition(0.12);
        robot.claw.setPosition(0.84);

        robot.claw.setPosition(0.84);
        robot.extendo.setPower(0);

        Actions.runBlocking(
                new ParallelAction(
                        trajectorywww3.build(),
                        ddddddd()

                )

                );

        robot.claw.setPosition(0.6);

        robot.extendo.setPower(1);

        sleep(700);

        robot.leftSlide.setPower(-0.3);
        robot.rightSlide.setPower(-0.3);
        sleep(1000);

        robot.claw.setPosition(0.6);
        robot.clawRotate.setPosition(.3906);
        robot.clawLeftMove.setPosition(0.6);
        robot.clawPivot.setPosition(0.62);
        sleep(2000);



    }
}

