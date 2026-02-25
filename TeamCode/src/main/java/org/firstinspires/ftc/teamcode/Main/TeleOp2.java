

package org.firstinspires.ftc.teamcode.Main;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOp2")
public class TeleOp2 extends OpMode {

    // Declare Motors and Servos

    private DcMotorEx LFMotor;
    private DcMotorEx LBMotor;
    private DcMotorEx RBMotor;
    private DcMotorEx RFMotor;
    private DcMotorEx Potato1;
    private DcMotorEx Potato2;
    private DcMotorEx Potato3;
    private Servo Servo7;
    private Servo Servo8;


    //Declare Variables
    int Toggle;
    int Toggle2;
    int Toggle3;
    int Toggle4;
    double error;
    boolean CircleWasPressed;
    boolean SquareWasPressed;
    double TargetVelocity;
<<<<<<< Updated upstream
    double[] Velocity = {-1200, -1300, -1700, -1800, -1900, -2000};
=======
    double[] Velocity = { 900, 1200, 1500, 2100, 2200, 2300, 2400, 2500};
>>>>>>> Stashed changes

    int VelocityIndex = 2;
    ShooterState shooterstate = ShooterState.Idle;
    double StateStartTime = 0;
    boolean motorIsRevving = false;

    //Below is the enumeration for all the shooter states used in the state machine for the shooter.
    public enum ShooterState {
        Idle,
        Calibration,
        Shoot,
        Reverse,
        Outtake,


    }


    @Override
    public void init() {
        //Assign the declared motors and servos to their respective ports
        LFMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LBMotor = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RBMotor = hardwareMap.get(DcMotorEx.class, "RB Motor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        Potato1 = hardwareMap.get(DcMotorEx.class, "Potato1");
        Potato2 = hardwareMap.get(DcMotorEx.class, "Potato2");
        Potato3 = hardwareMap.get(DcMotorEx.class, "Potato3");
        Servo7 = hardwareMap.get(Servo.class, "Servo 7");
        Servo8 = hardwareMap.get(Servo.class, "Servo 8");

        //Initialize variables and set motors behavior
        Toggle4 = 1;
        Toggle3 = 1;
        Toggle2 = 1;
        Toggle = 1;
        CircleWasPressed = false;
        SquareWasPressed = false;
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Potato1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Potato1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Potato1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(240.0, 0, 0.5, 12.8240);
        Potato1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Servo7.setPosition(0.5);
        Servo8.setPosition(0.5);


    }

    //Below is the main loop that runs during the OpMode
    @Override
    public void loop() {
        DriveControl();
        Telemetry();
        ShooterLogic();


    }

    //Below is what controls the driving mechanism
    private void DriveControl() {
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // Multiplier to counteract strafe friction
        double rx = gamepad1.right_stick_x;

// 2. Calculate powers
        double frontLeftPower  = y - x - rx;
        double backLeftPower   = y + x - rx;
        double frontRightPower = y + x + rx;
        double backRightPower  = y - x + rx;

// 3. Normalize values
// Find the largest power requested
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backRightPower));

// If the max is greater than 1.0, divide all powers by that max
        if (max > 1.0) {
            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;
        }

// 4. Send to motors
        LFMotor.setPower(frontLeftPower);
        LBMotor.setPower(backLeftPower);
        RFMotor.setPower(frontRightPower);
        RBMotor.setPower(backRightPower);

    }


    // Below is what controls what shows up on the robot driver dashboard
    private void Telemetry() {

        telemetry.addData("Current Velocity", Potato1.getVelocity());
        telemetry.addData("Target Velocity", TargetVelocity);
        telemetry.addData("Error", error);

        telemetry.addData("State", shooterstate);
        if (motorIsRevving) {
            telemetry.addData("Revving", "On");
        } else {
            telemetry.addData("Revving", "Off");

        }
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.update();
    }


    //Below is the code for controlling the Shooter, intake into the shooter and the reverse intake.
    private void ShooterLogic() {
        if (gamepad1.dpadUpWasPressed()) {
            VelocityIndex = (VelocityIndex + 1) % Velocity.length;
        }


        if (gamepad1.dpadDownWasPressed()) {
            VelocityIndex = (VelocityIndex - 1 + Velocity.length) % Velocity.length;
        }

        TargetVelocity = Velocity[VelocityIndex];


        if (gamepad1.squareWasPressed() & !SquareWasPressed) {
            Toggle2 += 1;
            SquareWasPressed = true;

        } else {
            SquareWasPressed = false;
        }


        if (Toggle2 > 2) {
            Toggle2 = 1;
        }


        error = TargetVelocity - Potato1.getVelocity();


        //Below is the state machine for the shooter states
        switch (shooterstate) {
            case Idle:  //The state the robot is in when it isn't doing anything
                if (gamepad1.circle) {
                    // Mode: Circle Pressed
                    Potato2.setVelocity(2000);
                    Servo7.setPosition(-1.0);
                    Servo8.setPosition(1);
                }
                else if (gamepad1.left_trigger > 0.5) {
                    Potato2.setVelocity(-2000);
                    Servo7.setPosition(1);
                    Servo8.setPosition(-1);
                }
                else {
                    Potato2.setVelocity(0);
                    Servo7.setPosition(0.5);
                    Servo8.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                    Potato3.setPower(-1);
                } else {
                    Potato3.setPower(0);
                }



                if (gamepad1.right_trigger > 0.1) {
                    Potato1.setVelocity(TargetVelocity);
                    StateStartTime = getRuntime();
                    shooterstate = ShooterState.Calibration;      //State transition into calibration state
                }


                if (shooterstate == ShooterState.Idle) {


                    if (Toggle2 == 2) {
                        Potato1.setVelocity(TargetVelocity);
                        motorIsRevving = true;
                    } else {
                        motorIsRevving = false;
                    }


                    if (Potato1.getVelocity() > 50 && !motorIsRevving || Potato1.getVelocity() < -50 && !motorIsRevving) {
                        Potato1.setVelocity(1);
                    }

                }
                break;
            case Calibration:
                if (error < 5 && error > -5) {
                    shooterstate = ShooterState.Shoot;
                    StateStartTime = getRuntime();
                }
                break;

            case Shoot:


                if (error < 5 && error > -5 && getRuntime() - StateStartTime > 0.18) {
                    Potato3.setPower(-1);

                    shooterstate = ShooterState.Outtake;
                    StateStartTime = getRuntime();

                } else if (getRuntime() - StateStartTime > 0.19) {
                    shooterstate = ShooterState.Calibration;
                }

                break;
            case Reverse:

                if (getRuntime() - StateStartTime > 0.2) {
                    Potato1.setVelocity(0);
                    Servo7.setPosition(1);
                    Servo8.setPosition(-1);
                    if (getRuntime() - StateStartTime > 0.4) {

                        Servo7.setPosition(0.5);
                        Servo8.setPosition(0.5);
                        shooterstate = ShooterState.Idle;

                    }


                }
                break;
            case Outtake:
                if (getRuntime() - StateStartTime > 2) {
                    Potato3.setVelocity(0);
                    if (getRuntime() - StateStartTime > 2.3)
                        Potato1.setVelocity(0);
                    shooterstate = ShooterState.Idle;
                }
                break;

        }

    }
    }
