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
public class Right extends LinearOpMode {

    public static double timeSlidesUP = 1.65;

    public class Claw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.claw.setPosition(0.6);
            robot.clawRotate.setPosition(0.3906);

            robot.clawMove.setPosition(0.6);
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
               robot.centerSlide.setPower(1);
               robot.claw.setPosition(0.62);
               return true;
           } else {
               robot.leftSlide.setPower(0);
               robot.rightSlide.setPower(0);
               robot.centerSlide.setPower(0);
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
                robot.centerSlide.setPower(1);
                robot.clawRotate.setPosition(0.3906);
                robot.clawMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.clawMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                return true;
            } else {
                robot.extendo.setPower(-0.1);

                robot.centerSlide.setPower(0.1);
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
                robot.centerSlide.setPower(-1);
                return true;
            } else if (getRuntime() - stamp <1.2){
                robot.extendo.setPower(0);
                return true;
            }  else if (getRuntime()-stamp <1.7){
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                robot.centerSlide.setPower(0);

                robot.extendo.setPower(1);

                robot.clawMove.setPosition(0.55);

                robot.clawPivot.setPosition(0.93);
                robot.claw.setPosition(0.84);
                robot.clawRotate.setPosition(0.3906);
                return true;
            }

            else {
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                robot.centerSlide.setPower(0);
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
                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
                robot.extendo.setPower(1);
                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                robot.centerSlide.setPower(-1);
                robot.claw.setPosition(0.6);


                return true;
            } else if (getRuntime()-stamp<3){

                robot.claw.setPosition(0.84);
                robot.claw.setPosition(0.84);
                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
                robot.extendo.setPower(1);
                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                robot.centerSlide.setPower(-1);

                return true;
            }

            else if (getRuntime()-stamp<3.5){
                robot.claw.setPosition(0.84);
                robot.clawRotate.setPosition(0.3906);

                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);

                robot.claw.setPosition(0.84);

                robot.extendo.setPower(1);
                return true;
            }
 else {

                robot.centerSlide.setPower(0.1);
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
                robot.claw.setPosition(0.84);
                robot.clawMove.setPosition(0.48);
                robot.clawRotate.setPosition(0.3906);

                robot.clawPivot.setPosition(0.93);
                robot.clawMove.setPosition(0.48);
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
            if (grabTimer<0.2){
                robot.clawMove.setPosition(0.48);
                return  true;
            }
           else if (grabTimer<0.4){
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);
                    robot.claw.setPosition(0.58);
               return true;



           }

            else if (grabTimer<1.1){

                robot.clawRotate.setPosition(0.3906);
                robot.clawMove.setPosition(0.8);
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
                robot.centerSlide.setPower(0);
                robot.claw.setPosition(0.58);
                return true;



            }

            else if (grabTimer<1){

                robot.clawRotate.setPosition(0.3906);
                robot.clawMove.setPosition(0.8);
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




             if (getRuntime()-stamp<0.2){
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);
                    robot.claw.setPosition(0.58);
                robot.claw.setPosition(0.58);
                    return true;


            } else if (getRuntime()-stamp<0.5){

                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);

                robot.clawRotate.setPosition(0.3906);
                robot.clawMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);

                    return true;


            }

            else if (getRuntime()-stamp<0.8){
                if (robot.claw.getPosition()>0.58){
                    robot.claw.setPosition(0.58);
                }
                robot.clawRotate.setPosition(0.3906);
                robot.clawMove.setPosition(0.8);
                robot.clawPivot.setPosition(0.12);
                robot.extendo.setPower(-1);
                return true;
            } else {
                robot.extendo.setPower(-0.1);
                return false;
            }


        }
    }

    public class n implements Action {


        double stamp = getRuntime();



        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


        double grabTimer = getRuntime() - stamp;




            if ((grabTimer<0.2)){

                    robot.leftSlide.setPower(-1);
                    robot.rightSlide.setPower(-1);
                    robot.centerSlide.setPower(-1);

                    return true;


            } else if (grabTimer<0.4){

                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);

                    return true;



            } else if (grabTimer<0.7) {
                    robot.leftSlide.setPower(0);
                    robot.rightSlide.setPower(0);
                    robot.centerSlide.setPower(0);
                    robot.claw.setPosition(0.58);

                    return true;

            }
                else if (grabTimer<1.4){
                    if (robot.claw.getPosition()>0.58){
                        robot.claw.setPosition(0.58);
                    }



                robot.clawRotate.setPosition(0.95);
                robot.clawMove.setPosition(0.6);
                robot.clawPivot.setPosition(0.62);

                    robot.extendo.setPower(-1);
                    return true;

                } else {
                    robot.extendo.setPower(0);
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
                robot.centerSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
                return true;
            }   else {
                robot.leftSlide.setPower(0.1);
                robot.centerSlide.setPower(0.1);
                robot.rightSlide.setPower(0.1);
                return false;
            }


        }
    }

    public class outOut implements Action{
        double stamp = getRuntime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            double intakeTimer = getRuntime()-stamp;


            if (intakeTimer<0.5){
                if (robot.claw.getPosition()>0.6){
                    robot.claw.setPosition(0.6);
                }
                robot.clawRotate.setPosition(0.3906);
                robot.clawPivot.setPosition(0.62);
                robot.clawMove.setPosition(0.6);
                robot.extendo.setPower(1);

                return true;

            } else if (intakeTimer<0.8){
                robot.extendo.setPower(0);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                robot.centerSlide.setPower(1);

                return true;


            } else {
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                robot.centerSlide.setPower(0);
                robot.clawMove.setPosition(0.48);
                robot.clawPivot.setPosition(0.93);
                robot.claw.setPosition(0.84);
                return false;
            }
        }
    }
    public Action close(){
        return new s();
    }

    public Action inddt(){
        return new outOut();
    }

    public Action e(){
        return  new n();
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
                        .strafeToLinearHeading(new Vector2d(15,0), 0);

        TrajectoryActionBuilder trajectory1 = robot.drive.actionBuilder(new Pose2d(15, 0, 0))
                .strafeToLinearHeading(new Vector2d(20,-40.5), 0,
                        new TranslationalVelConstraint(30));


        TrajectoryActionBuilder trajectory2 = robot.drive.actionBuilder(new Pose2d(20, -40.5, 0))
                .strafeToLinearHeading(new Vector2d(10,-40.5), 0,
                        new TranslationalVelConstraint(30));

        TrajectoryActionBuilder trajectory3 = robot.drive.actionBuilder(new Pose2d(10, -40.5, 0))
                .strafeToLinearHeading(new Vector2d(23.5,-50), 0,
                        new TranslationalVelConstraint(30));

        TrajectoryActionBuilder trajectory4 = robot.drive.actionBuilder(new Pose2d(20, -48.5, 0))
                .strafeToLinearHeading(new Vector2d(10,-53), 0,
                        new TranslationalVelConstraint(30));

        TrajectoryActionBuilder trajectory5 = robot.drive.actionBuilder(new Pose2d(10, -53, 0))
                .strafeToLinearHeading(new Vector2d(20,-53), 0,
                        new TranslationalVelConstraint(30))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(7,-43 ),Math.PI,
                        new TranslationalVelConstraint(30));

        TrajectoryActionBuilder trajectory8 = robot.drive.actionBuilder(new Pose2d(6, -43, Math.PI))

                .strafeToLinearHeading(new Vector2d(3.75,-43 ),Math.PI,
                        new TranslationalVelConstraint(30));

        TrajectoryActionBuilder trajectory6 = robot.drive.actionBuilder(new Pose2d(4.25, -43, Math.PI))
                .strafeToLinearHeading(new Vector2d(11.7,5), 0);

        TrajectoryActionBuilder trajectory7 = robot.drive.actionBuilder(new Pose2d(11.3, 5, 0))
                .strafeToLinearHeading(new Vector2d(5,-43), 0);


        TrajectoryActionBuilder trajectory2e = robot.drive.actionBuilder(new Pose2d(17.2, 53, 0))

                .strafeToLinearHeading(new Vector2d(6,47), Math.toRadians(-45));

        TrajectoryActionBuilder trajectory23 = robot.drive.actionBuilder(new Pose2d(6, 46, Math.toRadians(-45)))

                .strafeToLinearHeading(new Vector2d(3.4,52), Math.toRadians(-45),
                        new TranslationalVelConstraint(20));

        TrajectoryActionBuilder trajectorywww3 = robot.drive.actionBuilder(new Pose2d(3.4, 52, Math.toRadians(-45)))
                .splineToSplineHeading(new Pose2d(53,27,Math.toRadians(-90)), Math.toRadians(0),
                        new TranslationalVelConstraint(40))
                .splineToSplineHeading(new Pose2d(53,18,Math.toRadians(-90)), Math.toRadians(0),
                        new TranslationalVelConstraint(40));



        robot.claw.setPosition(0.6);
        robot.clawMove.setPosition(0.5);
        robot.clawPivot.setPosition(0.2);
        robot.clawRotate.setPosition(0.38);

        robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.centerSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);










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

        Actions.runBlocking(close());

        Actions.runBlocking(trajectory2.build());

        robot.claw.setPosition(0.84);
        sleep(1000);

        Actions.runBlocking(
                new ParallelAction(

                        inddt(),
                        trajectory3.build()

                )
        );

        Actions.runBlocking(e());

        Actions.runBlocking(trajectory4.build());
        robot.clawRotate.setPosition(0.3906);
        robot.clawMove.setPosition(0.8);
        robot.clawPivot.setPosition(0.12);
        sleep(1000);
        robot.claw.setPosition(0.84);


        Actions.runBlocking(trajectory5.build());
        robot.claw.setPosition(0.6);
        robot.clawRotate.setPosition(0.95);
        robot.clawMove.setPosition(0.6);
        robot.clawPivot.setPosition(0.62);
        sleep(600);

        robot.claw.setPosition(0.84);
        Actions.runBlocking(trajectory8.build());

        robot.claw.setPosition(0.58);

        Actions.runBlocking(
                        trajectory6.build()
        );

        robot.claw.setPosition(0.58);

        Actions.runBlocking(claw());

        Actions.runBlocking(slides());

        robot.claw.setPosition(0.8);

        Actions.runBlocking(
                new ParallelAction(
                        trajectory7.build(),
                        down()

                )

        );





        sleep(2000);

    }
}

