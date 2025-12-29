package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.jar.Attributes;

@Autonomous(name = "Auto 1")

public class Auto_1  extends LinearOpMode {
    private DcMotorEx LFMotor;
    private DcMotorEx LBMotor;
    private DcMotorEx RBMotor;
    private DcMotorEx RFMotor;
    private DcMotorEx Potato1;

    private Servo Servo7;
    private Servo Servo8;

    static final double  TICKS_PER_MOTOR_REV = 0;
    static final double WHEEL_DIAMETER_INCHES = 0;

    static final double TICKS_PER_INCH = 0;




    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode () {

        LFMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LBMotor = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RBMotor = hardwareMap.get(DcMotorEx.class, "RB Motor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        Potato1 = hardwareMap.get(DcMotorEx.class, "Potato1");
        Servo7 = hardwareMap.get(Servo.class, "Servo 7");
        Servo8 = hardwareMap.get(Servo.class, "Servo 8");

        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Potato1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Potato1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(240.0, 0, 0.5, 15.8240);
        Potato1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();




    }




}
