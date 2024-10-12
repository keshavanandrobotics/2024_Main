package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DistanceSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        AnalogInput us = hardwareMap.analogInput.get("US");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            int voltage = (int) ((us.getVoltage() / us.getMaxVoltage()) * 1023);
          double distance = voltage*6-300;
          telemetry.addData("V",voltage);
          telemetry.addData("dist",distance);
          telemetry.update();
        }
    }
}