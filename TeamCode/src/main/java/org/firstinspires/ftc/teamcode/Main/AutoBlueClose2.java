package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class AutoBlueClose2 extends LinearOpMode {
    public enum robotState {
        Idle, Reversing, Shooting, Outtake, Travel, Intake,
        Shooting2, Outtake2, Travel2, Intake2,
        Shooting3, Outtake3, Travel3, Intake3,
        Shooting4, Outtake4, Finished
    }

    robotState RobotState = robotState.Idle;
    double StateStartTime = 0;

    private DcMotorEx LFMotor, LBMotor, RBMotor, RFMotor;
    private DcMotorEx Potato1, Potato2, Potato3;
    private Servo Servo7, Servo8;
    private IMU imu;

    static final double TICKS_PER_MOTOR_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 4.09449;
    static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI));
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // --- HARDWARE MAPPING ---
        LFMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LBMotor = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RBMotor = hardwareMap.get(DcMotorEx.class, "RB Motor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        Potato1 = hardwareMap.get(DcMotorEx.class, "Potato1");
        Potato2 = hardwareMap.get(DcMotorEx.class, "Potato2");
        Potato3 = hardwareMap.get(DcMotorEx.class, "Potato3");
        Servo7 = hardwareMap.get(Servo.class, "Servo 7");
        Servo8 = hardwareMap.get(Servo.class, "Servo 8");
        imu = hardwareMap.get(IMU.class, "imu");

        // --- IMU SETUP ---
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        // --- MOTOR CONFIG ---
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Potato1.setDirection(DcMotorSimple.Direction.REVERSE);

        setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        resetDriveEncoders();

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(240.0, 0, 0.5, 15.8240);
        Potato1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            switch (RobotState) {
                case Idle:
                    RobotState = robotState.Reversing;
                    break;

                case Reversing:
                    encoderDrive(0.8, -35.83, -35.83, -36, -36, 8, 0);
                    Potato1.setVelocity(-1400);
                    RobotState = robotState.Shooting;
                    break;

                case Shooting:
                    if (1400 - Math.abs(Potato1.getVelocity()) < 15) {
                        runIntakeSystem(-2000, -1, 1, -1);
                        StateStartTime = getRuntime();
                        RobotState = robotState.Outtake;
                    }
                    break;

                case Outtake:
                    if (getRuntime() - StateStartTime > 5.0) {
                        stopIntakeSystem();
                        if (getRuntime() - StateStartTime > 5.5) {
                            Potato1.setVelocity(0);
                            RobotState = robotState.Travel;
                            StateStartTime = getRuntime();
                        }
                    }
                    break;

                case Travel:
                    encoderDrive(0.5, -4.5, -4.5, 4.5, 4.5, 3, 45);
                    encoderDrive(0.9, -20.86, 20.86, 20.86, -20.86, 4, 45);
                    runIntakeSystem(-2000, -1, 1, -1);
                    encoderDrive(0.6, 29.52, 29.52, 29.52, 29.52, 5, 45);
                    RobotState = robotState.Intake;
                    break;

                case Intake:
                    stopIntakeSystem();
                    encoderDrive(0.6, -29.53, -29.53, -29.53, -29.53, 5, 45);
                    encoderDrive(0.9, 20.86, -20.86, -20.86, 20.86, 4, 45);
                    encoderDrive(0.5, 4.5, 4.5, -4.5, -4.5, 3, 0);
                    Potato1.setVelocity(-1350);
                    RobotState = robotState.Shooting2;
                    break;

                case Shooting2:
                    if (1350 - Math.abs(Potato1.getVelocity()) < 15) {
                        runIntakeSystem(-2000, -1, 1, -1);
                        StateStartTime = getRuntime();
                        RobotState = robotState.Outtake2;
                    }
                    break;

                case Outtake2:
                    if (getRuntime() - StateStartTime > 3.0) {
                        stopIntakeSystem();
                        if (getRuntime() - StateStartTime > 3.3) {
                            Potato1.setVelocity(0);
                            RobotState = robotState.Travel2;
                            StateStartTime = getRuntime();
                        }
                    }
                    break;

                case Travel2:
                    encoderDrive(0.5, -4.5, -4.5, 4.5, 4.5, 3, 45);
                    encoderDrive(0.9, -42.91, 42.91, 42.91, -42.91, 5, 45);
                    runIntakeSystem(-2000, -1, -1, 1);
                    encoderDrive(0.6, 29.52, 29.52, 29.52, 29.52, 5, 45);
                    RobotState = robotState.Intake2;
                    break;

                case Intake2:
                    stopIntakeSystem();
                    encoderDrive(0.6, -29.53, -29.53, -29.53, -29.53, 5, 45);
                    encoderDrive(0.9, 42.91, -42.91, -42.91, 42.91, 5, 45);
                    encoderDrive(0.5, 4.5, 4.5, -4.5, -4.5, 3, 0);
                    Potato1.setVelocity(-1350);
                    RobotState = robotState.Shooting3;
                    break;

                case Shooting3:
                    if (1350 - Math.abs(Potato1.getVelocity()) < 15) {
                        runIntakeSystem(-2000, -1, 1, -1);
                        StateStartTime = getRuntime();
                        RobotState = robotState.Outtake3;
                    }
                    break;

                case Outtake3:
                    if (getRuntime() - StateStartTime > 4.0) {
                        stopIntakeSystem();
                        if (getRuntime() - StateStartTime > 4.3) {
                            Potato1.setVelocity(0);
                            RobotState = robotState.Travel3;
                            StateStartTime = getRuntime();
                        }
                    }
                    break;

                case Travel3:
                    encoderDrive(0.5, -4.5, -4.5, 4.5, 4.5, 3, 45);
                    encoderDrive(0.9, -65, 65, 65, -65, 6, 45);
                    runIntakeSystem(-2000, -1, 1, -1);
                    encoderDrive(0.6, 29.52, 29.52, 29.52, 29.52, 5, 45);
                    RobotState = robotState.Intake3;
                    break;

                case Intake3:
                    stopIntakeSystem();
                    encoderDrive(0.6, -29.53, -29.53, -29.53, -29.53, 5, 45);
                    encoderDrive(0.9, 65, -65, -65, 65, 6, 45);
                    encoderDrive(0.5, 4.5, 4.5, -4.5, -4.5, 3, 0);
                    Potato1.setVelocity(-1350);
                    RobotState = robotState.Shooting4;
                    break;

                case Shooting4:
                    if (1350 - Math.abs(Potato1.getVelocity()) < 15) {
                        runIntakeSystem(-2000, -1, 1, -1);
                        StateStartTime = getRuntime();
                        RobotState = robotState.Outtake4;
                    }
                    break;

                case Outtake4:
                    if (getRuntime() - StateStartTime > 4.0) {
                        stopIntakeSystem();
                        Potato1.setVelocity(0);
                        RobotState = robotState.Finished;
                    }
                    break;

                case Finished:
                    stopDrive();
                    break;
            }
        }
    }

    // --- DRIVE CORE ---

    public void encoderDrive(double speed, double lfIn, double lbIn, double rfIn, double rbIn, double timeoutS, double targetHeading) {
        if (!opModeIsActive()) return;

        int LFT = LFMotor.getCurrentPosition() + (int) (lfIn * TICKS_PER_INCH);
        int LBT = LBMotor.getCurrentPosition() + (int) (lbIn * TICKS_PER_INCH);
        int RFT = RFMotor.getCurrentPosition() + (int) (rfIn * TICKS_PER_INCH);
        int RBT = RBMotor.getCurrentPosition() + (int) (rbIn * TICKS_PER_INCH);

        LFMotor.setTargetPosition(LFT);
        LBMotor.setTargetPosition(LBT);
        RFMotor.setTargetPosition(RFT);
        RBMotor.setTargetPosition(RBT);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeoutS && (LFMotor.isBusy() && RFMotor.isBusy())) {
            // 1. Calculate the distance remaining (Error)
            int avgError = (Math.abs(LFT - LFMotor.getCurrentPosition()) + Math.abs(RFT - RFMotor.getCurrentPosition())) / 2;

            // 2. Slow down logic: The "0.2" ensures we don't drop to zero power too early
            double adjustedSpeed = (avgError < 1200) ? Math.max(0.2, speed * (avgError / 1200.0)) : speed;

            // 3. Get Correction
            double correction = getSteeringCorrection(targetHeading, 0.012);

            // 4. IMPORTANT: Scale the correction by the speed!
            // This prevents the correction from overpowering the motors during the "Slow Down" phase.
            double finalCorrection = correction * (adjustedSpeed / speed);

            LFMotor.setPower(adjustedSpeed - finalCorrection);
            LBMotor.setPower(adjustedSpeed - finalCorrection);
            RFMotor.setPower(adjustedSpeed + finalCorrection);
            RBMotor.setPower(adjustedSpeed + finalCorrection);
        }
        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getSteeringCorrection(double targetAngle, double P_COEFF) {
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = targetAngle - currentAngle;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return (Math.abs(error) < 0.5) ? 0 : error * P_COEFF;
    }

    // --- HELPERS ---

    private void runIntakeSystem(double pot2Vel, double pot3Pow, double s7Pos, double s8Pos) {
        Potato2.setVelocity(pot2Vel);
        Potato3.setPower(pot3Pow);
        Servo7.setPosition(s7Pos);
        Servo8.setPosition(s8Pos);
    }

    private void stopIntakeSystem() {
        runIntakeSystem(0, 0, 0.5, 0.5);
    }

    private void stopDrive() {
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        LFMotor.setMode(mode);
        LBMotor.setMode(mode);
        RFMotor.setMode(mode);
        RBMotor.setMode(mode);
    }

    private void resetDriveEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDriveZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        LFMotor.setZeroPowerBehavior(behavior);
        LBMotor.setZeroPowerBehavior(behavior);
        RFMotor.setZeroPowerBehavior(behavior);
        RBMotor.setZeroPowerBehavior(behavior);
    }
}