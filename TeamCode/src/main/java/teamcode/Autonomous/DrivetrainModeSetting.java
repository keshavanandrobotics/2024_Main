package teamcode.Autonomous;


import static teamcode.Teleop.Singletons.VARS.DRIVE_MODE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous
public class DrivetrainModeSetting extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeInInit()){
            telemetry.addData("Drive Mode", DRIVE_MODE);
            telemetry.addLine("UP --> FC");
            telemetry.addLine("RIGHT --> FC2");
            telemetry.addLine("DOWN --> SAM");
            telemetry.addLine("LEFT --> !SAM");




            if (gamepad1.dpad_up){
                DRIVE_MODE = "FC";
            } else if (gamepad1.dpad_right){
                DRIVE_MODE = "FC2";
            } else if (gamepad1.dpad_down) {
                DRIVE_MODE = "SAM";
            } else if (gamepad1.dpad_left){
                DRIVE_MODE = "!SAM";
            }

            telemetry.update();


        }



        waitForStart();



    }
}
