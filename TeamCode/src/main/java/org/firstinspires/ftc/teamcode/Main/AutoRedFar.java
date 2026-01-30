package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoRedFar extends LinearOpMode {
    double StateStartTime = 0;
    public enum RobotState {
        Idle,
        Revving,
        Shooting,
        Finished,
    }
    RobotState robotstate = RobotState.Idle;
    private DcMotorEx LFMotor;
    private DcMotorEx LBMotor;
    private DcMotorEx RBMotor;
    private DcMotorEx RFMotor;
    private DcMotorEx Potato1;
    private DcMotorEx Potato2;
    private DcMotorEx Potato3;

    private Servo Servo7;
    private Servo Servo8;






    public void runOpMode(){
        LFMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LBMotor = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RBMotor = hardwareMap.get(DcMotorEx.class, "RB Motor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        Potato1 = hardwareMap.get(DcMotorEx.class, "Potato1");
        Potato2 = hardwareMap.get(DcMotorEx.class, "Potato2");
        Potato3 = hardwareMap.get(DcMotorEx.class, "Potato3");
        Servo7 = hardwareMap.get(Servo.class, "Servo 7");
        Servo8 = hardwareMap.get(Servo.class, "Servo 8");

        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        waitForStart();
        while (opModeIsActive()) {

            switch (robotstate) {
                case Idle:

                    StateStartTime = getRuntime();
                    robotstate = RobotState.Revving;
                    break;
                case Revving:
                    Potato1.setVelocity(-1700);
                    StateStartTime = getRuntime();
                    robotstate = RobotState.Shooting;
                    break;

                case Shooting:
                    if (1700 - Math.abs(Potato1.getVelocity()) < 10) {
                        Potato2.setVelocity(-2000);
                        Servo7.setPosition(-1);
                        Servo8.setPosition(1);
                        Potato3.setPower(-1);
                        robotstate = RobotState.Finished;
                        StateStartTime = getRuntime();
                    }
                    break;
                case Finished:
                    if (getRuntime() - StateStartTime > 4) {
                        Potato2.setVelocity(0);
                        Servo7.setPosition(0.5);
                        Servo8.setPosition(0.5);
                        Potato3.setPower(0);
                        if (getRuntime() - StateStartTime > 5) {
                            Potato1.setVelocity(1);
                        }
                    }

            }
        }


    }

}
