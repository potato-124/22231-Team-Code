package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Autonomous

public class AutoBlueClose extends LinearOpMode {
    public enum robotState {
        Idle,
        Reversing,
        Shooting,
        Outtake,
        Travel,

        Intake,
        Shooting2,
        Outtake2,
        Travel2,

        Intake2,
        Shooting3,
        Outtake3,
        Travel3,
        Intake3,
        Shooting4,
        Outtake4,
        Finished,
    }

    private robotState nextStateAfterDrive;
    robotState RobotState = robotState.Idle;
    // Inside your Hardware Map / Init section

    double StateStartTime = 0;
    private DcMotorEx LFMotor;
    private DcMotorEx LBMotor;
    private DcMotorEx RBMotor;
    private DcMotorEx RFMotor;
    private DcMotorEx Potato1;
    private DcMotorEx Potato2;
    private DcMotorEx Potato3;

    private Servo Servo7;
    private Servo Servo8;

    static final double TICKS_PER_MOTOR_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 4.09449;
    static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI));

    static final double Drive_Speed = 0.6;
    static final double Turn_Speed = 0.5;


    private ElapsedTime runtime = new ElapsedTime();

    // Add "IMU imu" to the parentheses

    public void SetDrivePower(double LB, double LF, double RB, double RF) {
        LBMotor.setPower(LB);
        LFMotor.setPower(LF);
        RBMotor.setPower(RB);
        RFMotor.setPower(RF);

    }

    public void SetLeftDrivePower(double LB, double LF) {
        LBMotor.setPower(LB);
        LFMotor.setPower(LF);
    }

    public void SetRightDrivePower(double RB, double RF) {

        RBMotor.setPower(RB);
        RFMotor.setPower(RF);

    }

    public void SetTargetPosition(int LB, int LF, int RB, int RF) {
        LBMotor.setTargetPosition(LB);
        LFMotor.setTargetPosition(LF);
        RBMotor.setTargetPosition(RB);
        RFMotor.setTargetPosition(RF);

    }

    public void SetLeftTargetPosition(int LB, int LF) {
        LBMotor.setTargetPosition(LB);
        LFMotor.setTargetPosition(LF);

    }

    public void SetRightTargetPosition(int RB, int RF) {
        RBMotor.setTargetPosition(RB);
        RFMotor.setTargetPosition(RF);

    }


    public void runOpMode() {



// ... inside class ...


// ... inside runOpMode() before waitForStart() ...


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

        telemetry.addData("Starting at", "%7d :%7d",
                LFMotor.getCurrentPosition(),
                LBMotor.getCurrentPosition(),
                RFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition());


        waitForStart();
        while (opModeIsActive()) {

            switch (RobotState) {
                case Idle:
                    RobotState = robotState.Reversing;
                    StateStartTime = getRuntime();
                    break;
                case Reversing:
                    encoderDrive(
                            0.8, -35.83, -35.83, -36, -36, 8);

                    Potato1.setVelocity(-1400);
                   RobotState = robotState.Shooting;
                    StateStartTime = getRuntime();


                    break;

                case Shooting:
                    if (1400 - Math.abs(Potato1.getVelocity()) < 10) {
                        Potato3.setPower(-1);
                        Potato2.setVelocity(-2000);
                        Servo7.setPosition(1);
                        Servo8.setPosition(-1);
                        StateStartTime = getRuntime();
                        RobotState = robotState.Outtake;


                    }
                    break;
                case Outtake:
                    if (getRuntime() - StateStartTime > 5) {
                        Potato3.setPower(0);
                        Potato2.setVelocity(0);
                        Servo7.setPosition(0.5);
                        Servo8.setPosition(0.5);
                        if (getRuntime() - StateStartTime > 5.5) {
                            Potato1.setVelocity(1);
                            RobotState = robotState.Travel;
                            StateStartTime = getRuntime();

                        }
                    }
                    break;

                case Travel:
                    encoderDrive(0.5, -4.5, -4.5, 4.5, 4.5, 8);
                    if (getRuntime() - StateStartTime > 1) {


                        encoderDrive(0.9, -20.8661, 20.8661, 20.8661, -20.8661, 8);
                        Potato2.setVelocity(-2000);
                        Servo7.setPosition(1);
                        Servo8.setPosition(-1);
                        Potato3.setPower(-1);
                        encoderDrive(0.6, 29.5276, 29.5276, 29.5276, 29.5276, 8);

                        StateStartTime = getRuntime();
                        RobotState = robotState.Intake;
                    }


                    break;
                case Intake:
                    Potato2.setVelocity(-0);
                    Servo7.setPosition(0.5);
                    Servo8.setPosition(0.5);
                    Potato3.setPower(0);
                    encoderDrive(0.6, -29.53, -29.53, -29.53, -29.53, 8);
                    encoderDrive(0.9, 20.8661, -20.8661, -20.8661, 20.8661, 8);
                    encoderDrive(0.5, 4.5, 4.5, -4.5, -4.5, 8);
                    Potato1.setVelocity(-1350);
                    RobotState = robotState.Shooting2;
                    StateStartTime = getRuntime();
                    break;

                case Shooting2:
                    if (1400 - Math.abs(Potato1.getVelocity()) < 10) {
                        Potato3.setPower(-1);
                        Potato2.setVelocity(-2000);
                        Servo7.setPosition(1);
                        Servo8.setPosition(-1);
                        StateStartTime = getRuntime();
                        RobotState = robotState.Outtake2;


                    }
                    break;
                case Outtake2:
                    if (getRuntime() - StateStartTime > 3) {
                        Potato3.setPower(0);
                        Potato2.setVelocity(0);
                        Servo7.setPosition(0.5);
                        Servo8.setPosition(0.5);
                        if (getRuntime() - StateStartTime > 3.3) {
                            Potato1.setVelocity(1);
                            RobotState = robotState.Travel2;

                        }
                    }
                    break;
                case Travel2:
                    encoderDrive(0.5, -4.5, -4.5, 4.5, 4.5, 8);
                    if (getRuntime() - StateStartTime > 1) {


                        encoderDrive(0.9, -42.9134, 42.9134, 42.9134, -42.9134, 8);
                        Potato2.setVelocity(-2000);
                        Servo7.setPosition(-1);
                        Servo8.setPosition(1);
                        Potato3.setPower(-1);
                        encoderDrive(0.6, 29.5276, 29.5276, 29.5276, 29.5276, 8);

                        StateStartTime = getRuntime();
                        RobotState = robotState.Intake2;
                    }
                case Intake2:
                    Potato2.setVelocity(-0);
                    Servo7.setPosition(0.5);
                    Servo8.setPosition(0.5);
                    Potato3.setPower(0);
                    encoderDrive(0.6, -29.53, -29.53, -29.53, -29.53, 8);
                    encoderDrive(0.9, 42.9134, -42.9134, -42.9134, 42.9134, 8);
                    encoderDrive(0.5, 4.5, 4.5, -4.5, -4.5, 8);
                    Potato1.setVelocity(-1350);
                    RobotState = robotState.Shooting3;
                    StateStartTime = getRuntime();
                    break;
                case Shooting3:
                    if (1350 - Math.abs(Potato1.getVelocity()) < 10) {
                        Potato3.setPower(-1);
                        Potato2.setVelocity(-2000);
                        Servo7.setPosition(1);
                        Servo8.setPosition(-1);
                        StateStartTime = getRuntime();
                        RobotState = robotState.Outtake3;


                    }
                    break;
                case Outtake3:
                    if (getRuntime() - StateStartTime > 4) {
                        Potato3.setPower(0);
                        Potato2.setVelocity(0);
                        Servo7.setPosition(0.5);
                        Servo8.setPosition(0.5);
                        if (getRuntime() - StateStartTime > 3.3) {
                            Potato1.setVelocity(1);
                            RobotState = robotState.Travel3;

                        }


                    } break;
                case Travel3:
                    encoderDrive(0.5, -4.5, -4.5, 4.5, 4.5, 8);
                    if (getRuntime() - StateStartTime > 1) {


                        encoderDrive(0.9, -65, 65, 65, -65, 8);
                        Potato2.setVelocity(-2000);
                        Servo7.setPosition(1);
                        Servo8.setPosition(-1);
                        Potato3.setPower(-1);
                        encoderDrive(0.6, 29.5276, 29.5276, 29.5276, 29.5276, 8);

                        StateStartTime = getRuntime();
                        RobotState = robotState.Intake3;

                    }
                    break;
                case Intake3:
                    Potato2.setVelocity(-0);
                    Servo7.setPosition(0.5);
                    Servo8.setPosition(0.5);
                    Potato3.setPower(0);
                    encoderDrive(0.6, -29.53, -29.53, -29.53, -29.53, 8);
                    encoderDrive(0.9, 42.9134, -42.9134, -42.9134, 42.9134, 8);
                    encoderDrive(0.5, 4.5, 4.5, -4.5, -4.5, 8);
                    Potato1.setVelocity(-1350);
                    RobotState = robotState.Shooting4;
                    StateStartTime = getRuntime();
                    break;
                case Shooting4:
                    if (1350 - Math.abs(Potato1.getVelocity()) < 10) {
                        Potato3.setPower(-1);
                        Potato2.setVelocity(-2000);
                        Servo7.setPosition(1);
                        Servo8.setPosition(-1);
                        StateStartTime = getRuntime();
                        RobotState = robotState.Outtake4;


                    }
                    break;


                case Outtake4:
                    if (getRuntime() - StateStartTime > 4) {
                        Potato3.setPower(0);
                        Potato2.setVelocity(0);
                        Servo7.setPosition(0.5);
                        Servo8.setPosition(0.5);
                        if (getRuntime() - StateStartTime > 3.3) {
                            Potato1.setVelocity(1);
                            RobotState = robotState.Finished;

                        }


                    } break;
                case Finished:






            }


        }
    }


    public void encoderDrive(double speed,
                             double LeftFrontInches, double LeftBackInches, double RightFrontInches,
                             double RightBackInches,
                             double timeoutS) {
        int LFTarget;
        int LBTarget;
        int RFTarget;
        int RBTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LFTarget = LFMotor.getCurrentPosition() + (int) (LeftFrontInches * TICKS_PER_INCH);
            LBTarget = LBMotor.getCurrentPosition() + (int) (LeftBackInches * TICKS_PER_INCH);
            RFTarget = RFMotor.getCurrentPosition() + (int) (RightFrontInches * TICKS_PER_INCH);
            RBTarget = RBMotor.getCurrentPosition() + (int) (RightBackInches * TICKS_PER_INCH);

            LFMotor.setTargetPosition(LFTarget);
            LBMotor.setTargetPosition(LBTarget);
            RFMotor.setTargetPosition(RFTarget);
            RBMotor.setTargetPosition(RBTarget);
            // Turn On RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            LFMotor.setPower(Math.abs(speed));
            LBMotor.setPower(Math.abs(speed));
            RFMotor.setPower(Math.abs(speed));
            RBMotor.setPower(Math.abs(speed));
            int avgError =
                    (
                            Math.abs(LFMotor.getTargetPosition() - LFMotor.getCurrentPosition()) +
                                    Math.abs(LBMotor.getTargetPosition() - LBMotor.getCurrentPosition()) + Math.abs(RFMotor.getTargetPosition() - RFMotor.getCurrentPosition()) + Math.abs(RBMotor.getTargetPosition() - RBMotor.getCurrentPosition())
                    ) / 4;


            double slowDownRange = 1200; // ticks

            double adjustedSpeed = speed;
            if (avgError < slowDownRange) {
                adjustedSpeed = Math.max(0.15, speed * (avgError / slowDownRange));
            }

            LFMotor.setPower(adjustedSpeed);
            LBMotor.setPower(adjustedSpeed);
            RFMotor.setPower(adjustedSpeed);
            RBMotor.setPower(adjustedSpeed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LFMotor.isBusy() && RFMotor.isBusy()) && LBMotor.isBusy() && RBMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", LFTarget, RFTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        LFMotor.getCurrentPosition(), RFMotor.getCurrentPosition());
                telemetry.update();
            }


            LFMotor.setPower(0);
            LBMotor.setPower(0);
            RFMotor.setPower(0);
            RBMotor.setPower(0);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Turn off RUN_TO_POSITION

            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderDriveB(double speedLeft, double speedRight,
                              double LeftFrontInches, double LeftBackInches, double RightFrontInches,
                              double RightBackInches,
                              double timeoutS) {
        int LFTarget;
        int LBTarget;
        int RFTarget;
        int RBTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LFTarget = LFMotor.getCurrentPosition() + (int) (LeftFrontInches * TICKS_PER_INCH);
            LBTarget = LBMotor.getCurrentPosition() + (int) (LeftBackInches * TICKS_PER_INCH);
            RFTarget = RFMotor.getCurrentPosition() + (int) (RightFrontInches * TICKS_PER_INCH);
            RBTarget = RBMotor.getCurrentPosition() + (int) (RightBackInches * TICKS_PER_INCH);

            LFMotor.setTargetPosition(LFTarget);
            LBMotor.setTargetPosition(LBTarget);
            RFMotor.setTargetPosition(RFTarget);
            RBMotor.setTargetPosition(RBTarget);
            // Turn On RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            LFMotor.setPower(Math.abs(speedLeft));
            LBMotor.setPower(Math.abs(speedLeft));
            RFMotor.setPower(Math.abs(speedRight));
            RBMotor.setPower(Math.abs(speedRight));
            int avgError =
                    (
                            Math.abs(LFMotor.getTargetPosition() - LFMotor.getCurrentPosition()) +
                                    Math.abs(LBMotor.getTargetPosition() - LBMotor.getCurrentPosition()) + Math.abs(RFMotor.getTargetPosition() - RFMotor.getCurrentPosition()) + Math.abs(RBMotor.getTargetPosition() - RBMotor.getCurrentPosition())
                    ) / 4;


            double slowDownRange = 1200; // ticks

            double adjustedSpeedLeft = speedLeft;
            double adjustedSpeedRight = speedRight;
            if (avgError < slowDownRange) {
                adjustedSpeedLeft = Math.max(0.15, speedLeft * (avgError / slowDownRange));
                adjustedSpeedRight = Math.max(0.15, speedRight * (avgError / slowDownRange));

            }

            LFMotor.setPower(adjustedSpeedLeft);
            LBMotor.setPower(adjustedSpeedLeft);
            RFMotor.setPower(adjustedSpeedRight);
            RBMotor.setPower(adjustedSpeedRight);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LFMotor.isBusy() && RFMotor.isBusy()) && LBMotor.isBusy() && RBMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", LFTarget, RFTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        LFMotor.getCurrentPosition(), RFMotor.getCurrentPosition());
                telemetry.update();
            }


            LFMotor.setPower(0);
            LBMotor.setPower(0);
            RFMotor.setPower(0);
            RBMotor.setPower(0);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Turn off RUN_TO_POSITION

            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }

}





