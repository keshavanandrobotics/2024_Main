package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.MecanumDrive;

public class Robot {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx leftSlide;
    public DcMotorEx centerSlide;
    public DcMotorEx rightSlide;
    public DcMotorEx extendo;


    public MecanumDrive drive;

    public DcMotorEx leftPod;
    public DcMotorEx rightPod;
    public DcMotorEx sidePod;
    public DcMotorEx extendoEncoder;
    public Servo testServo;
    public Servo clawRotate;
    public Servo claw;

    public Servo clawPivot;

    public Servo clawMove;

    public IMU imu;

    public IMU.Parameters parameters;

    public AnalogInput frontExtPosition;
    public AnalogInput backExtPosition;






    public Robot(HardwareMap hardwareMap) {

        //Motors
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class,"backRight");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class,"frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRight");
        leftSlide = hardwareMap.get(DcMotorEx.class,"leftSlide");
        centerSlide = hardwareMap.get(DcMotorEx.class,"centerSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class,"rightSlide");
        extendo = hardwareMap.get(DcMotorEx.class,"extendo");



        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Servos:

        testServo = hardwareMap.servo.get("test");
        clawRotate =hardwareMap.servo.get("clawRotate");
        claw = hardwareMap.servo.get("claw");
        clawPivot = hardwareMap.servo.get("cPivot");
        clawMove = hardwareMap.servo.get("clawMove");





        //Odo Pods

        leftPod = frontLeftMotor;
        rightPod = backLeftMotor;
        sidePod = backRightMotor;

        extendoEncoder=extendo;

        //Misc

        imu = hardwareMap.get(IMU.class, "imu");

        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));









    }
}