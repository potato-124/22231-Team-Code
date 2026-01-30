package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto 1 - Clean")
public class AutoChat extends LinearOpMode {

    // Drive motors
    DcMotorEx LF, LB, RF, RB;

    // Mechanisms
    DcMotorEx Shooter, Feeder, Intake;
    Servo Servo7, Servo8;

    // Constants
    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER = 4.09449;
    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ===== Hardware Map =====
        LF = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LB = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RF = hardwareMap.get(DcMotorEx.class, "RF Motor");
        RB = hardwareMap.get(DcMotorEx.class, "RB Motor");

        Shooter = hardwareMap.get(DcMotorEx.class, "Potato1");
        Feeder  = hardwareMap.get(DcMotorEx.class, "Potato2");
        Intake  = hardwareMap.get(DcMotorEx.class, "Potato3");

        Servo7 = hardwareMap.get(Servo.class, "Servo 7");
        Servo8 = hardwareMap.get(Servo.class, "Servo 8");

        // ===== Motor Setup =====
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();

        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(240, 0, 0.5, 15.824)
        );

        Servo7.setPosition(0.5);
        Servo8.setPosition(0.5);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ==============================
        // 1️⃣ DRIVE BACK
        // ==============================
        encoderDrive(0.8, -35.83, -35.83, -35.83, -35.83, 6);

        // ==============================
        // 2️⃣ SHOOT FIRST SET
        // ==============================
        shootRings(1250, 3.0);

        // ==============================
        // 3️⃣ MOVE + INTAKE
        // ==============================
        encoderDrive(0.5,  2.25,  2.25, -2.25, -2.25, 3);
        encoderDrive(0.9, -20.87, 20.87, 20.87, 20.87, 4);

        Intake.setPower(-1);
        encoderDrive(0.6, 29.53, 29.53, 29.53, 29.53, 4);
        Intake.setPower(0);

        // ==============================
        // 4️⃣ RETURN
        // ==============================
        encoderDrive(0.6, -29.53, -29.53, -29.53, -29.53, 4);
        encoderDrive(0.9, 20.87, -20.87, -20.87, 20.87, 4);
        encoderDrive(0.5, -2.25, -2.25, 2.25, 2.25, 3);

        // ==============================
        // 5️⃣ SHOOT SECOND SET
        // ==============================
        shootRings(1250, 3.0);

        telemetry.addLine("Auto Complete");
        telemetry.update();
        sleep(1000);
    }

    // =====================================
    // METHODS
    // =====================================

    void shootRings(double velocity, double time) {
        Shooter.setVelocity(-velocity);

        while (opModeIsActive()
                && Math.abs(Shooter.getVelocity()) < velocity - 10) {
            idle();
        }

        Feeder.setVelocity(-2000);
        Intake.setPower(-1);
        Servo7.setPosition(0);
        Servo8.setPosition(1);

        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            idle();
        }

        Feeder.setVelocity(0);
        Intake.setPower(0);
        Servo7.setPosition(0.5);
        Servo8.setPosition(0.5);
        Shooter.setVelocity(0);
    }

    void resetDriveEncoders() {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void encoderDrive(double speed,
                      double lf, double lb,
                      double rf, double rb,
                      double timeout) {

        int lfTarget = LF.getCurrentPosition() + (int)(lf * TICKS_PER_INCH);
        int lbTarget = LB.getCurrentPosition() + (int)(lb * TICKS_PER_INCH);
        int rfTarget = RF.getCurrentPosition() + (int)(rf * TICKS_PER_INCH);
        int rbTarget = RB.getCurrentPosition() + (int)(rb * TICKS_PER_INCH);

        LF.setTargetPosition(lfTarget);
        LB.setTargetPosition(lbTarget);
        RF.setTargetPosition(rfTarget);
        RB.setTargetPosition(rbTarget);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LF.setPower(speed);
        LB.setPower(speed);
        RF.setPower(speed);
        RB.setPower(speed);

        timer.reset();
        while (opModeIsActive()
                && timer.seconds() < timeout
                && LF.isBusy() && LB.isBusy()
                && RF.isBusy() && RB.isBusy()) {
            idle();
        }

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(150);
    }
}

