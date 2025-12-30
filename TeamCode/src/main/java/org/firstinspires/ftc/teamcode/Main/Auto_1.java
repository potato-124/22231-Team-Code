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

    static final double  TICKS_PER_MOTOR_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 4.09449;
    static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV/WHEEL_DIAMETER_INCHES * Math.PI);

    static final double Drive_Speed = 0.6;
    static final double Turn_Speed = 0.5;




    private ElapsedTime runtime = new ElapsedTime();
    public void SetDrivePower (double LB, double LF, double RB, double RF){
        LBMotor.setPower(LB);
        LFMotor.setPower(LF);
        RBMotor.setPower(RB);
        RFMotor.setPower(RF);

    }
    public void SetLeftDrivePower (double LB, double LF){
        LBMotor.setPower(LB);
        LFMotor.setPower(LF);
    }
    public void SetRightDrivePower (double RB, double RF){

        RBMotor.setPower(RB);
        RFMotor.setPower(RF);

    }
    public void SetTargetPosition (int LB, int LF, int RB, int RF){
        LBMotor.setTargetPosition(LB);
        LFMotor.setTargetPosition(LF);
        RBMotor.setTargetPosition(RB);
        RFMotor.setTargetPosition(RF);

    }
    public void SetLeftTargetPosition (int LB, int LF){
        LBMotor.setTargetPosition(LB);
        LFMotor.setTargetPosition(LF);

    }
    public void SetRightTargetPosition ( int RB, int RF){
        RBMotor.setTargetPosition(RB);
        RFMotor.setTargetPosition(RF);

    }


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
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Potato1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Potato1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Potato1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(240.0, 0, 0.5, 15.8240);
        Potato1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addData("Starting at", "%7d :%7d",
                LFMotor.getCurrentPosition(),
                LBMotor.getCurrentPosition(),
                RFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition());



        waitForStart();





    }




}
