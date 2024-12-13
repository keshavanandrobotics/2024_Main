package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled

@TeleOp
public class DistanceSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        AnalogInput us = hardwareMap.analogInput.get("a0");
        AnalogInput us1 = hardwareMap.analogInput.get("a1");



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double voltage = us.getVoltage();
          double distance = ((voltage-0.119)/0.0126);
          telemetry.addData("V",voltage);
          telemetry.addData("MV", us.getMaxVoltage());
          telemetry.addData("dist",distance);
          telemetry.update();
        }
    }
}