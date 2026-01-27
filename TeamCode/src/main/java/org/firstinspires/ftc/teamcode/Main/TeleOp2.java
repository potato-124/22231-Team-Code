

package org.firstinspires.ftc.teamcode.Main;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp2")
public class TeleOp2 extends OpMode {

    // Declare Motors and Servos
    private DcMotorEx LFMotor;
    private DcMotorEx LBMotor;
    private DcMotorEx RBMotor;
    private DcMotorEx RFMotor;
    private DcMotorEx Potato1;
    private Servo Servo7;
    private Servo Servo8;
    private DcMotorEx Potato2;

    //Declare Variables
    float LeftStickY;
    float LeftStickX;
    int Toggle;
    int Toggle2;
    int Toggle3;
    int Toggle4;
    double error;
    boolean CircleWasPressed;
    boolean SquareWasPressed;
    boolean CrossWasPressed;
    Boolean LeftBumperWasPressed = false;
    float RightStickX;
    double TargetVelocity;
    double[] Velocity = { 900, 1200, 1500, 2100};

    int VelocityIndex = 1;
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
        Servo7 = hardwareMap.get(Servo.class, "Servo 7");
        Servo8 = hardwareMap.get(Servo.class, "Servo 8");
        Potato2 = hardwareMap.get(DcMotorEx.class, "Potato2");
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
        LFMotor.setPower(Range.clip(LeftStickY - RightStickX + LeftStickX, -1, 1));
        LBMotor.setPower(Range.clip(LeftStickY - RightStickX - LeftStickX, -1, 1));
        RBMotor.setPower(Range.clip(LeftStickY + RightStickX + LeftStickX, -1, 1));
        RFMotor.setPower(Range.clip(LeftStickY + RightStickX - LeftStickX, -1, 1));
        LeftStickY = gamepad1.left_stick_y;
        RightStickX = gamepad1.right_stick_x;
        LeftStickX = gamepad1.left_stick_x;
    }


    // Below is what controls what shows up on the robot driver dashboard
    private void Telemetry() {

        telemetry.addData("Current Velocity", Potato1.getVelocity());
        telemetry.addData("Target Velocity", TargetVelocity);
        telemetry.addData("Error", error);

        telemetry.addData("State", shooterstate);
        if(motorIsRevving){
            telemetry.addData("Revving", "On" );
        }
        else { telemetry.addData("Revving", "Off");
        }
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


        if (gamepad1.squareWasPressed() & !SquareWasPressed){
            Toggle2 += 1;
            SquareWasPressed = true;

        } else {SquareWasPressed = false;}


        if (Toggle2 > 2){
            Toggle2 = 1;
        }


        error = TargetVelocity - Potato1.getVelocity();


        //Below is the state machine for the shooter states
        switch (shooterstate) {
            case Idle:  //The state the robot is in when it isn't doing anything

                if (gamepad1.rightBumperWasPressed()) {
                    Potato1.setVelocity(TargetVelocity);
                    StateStartTime = getRuntime();
                    shooterstate = ShooterState.Calibration;      //State transition into calibration state
                }

                if (gamepad1.triangleWasPressed()) {
                    Potato1.setVelocity(-600);
                    StateStartTime = getRuntime();
                    shooterstate = ShooterState.Reverse;
                }

                if(gamepad1.crossWasPressed() & !CrossWasPressed){
                    Toggle3 += 1;
                    CrossWasPressed = true;
                }else {CrossWasPressed = false;}


                if (Toggle3 > 2){
                    Toggle3 = 1;
                }




                if(Toggle3 == 2 && Toggle == 1){
                    Potato2.setVelocity(-2000);
                    Servo7.setPosition(-1);
                    Servo8.setPosition(1);
                }else if(Toggle3 == 1 & Toggle == 1){Potato2.setVelocity(0);
                    Servo7.setPosition(0.5);
                    Servo8.setPosition(0.5);}


                if(gamepad1.circleWasPressed() && !CircleWasPressed){
                    Toggle += 1;
                    CircleWasPressed = true;
                }else {CircleWasPressed = false;}


                if(Toggle > 2)
                {Toggle = 1;}


                if(Toggle == 2 && Toggle3 == 1){
                    Potato2.setVelocity(2000);
                }else if(Toggle3 == 1 & Toggle == 1) {Potato2.setVelocity(0);}


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
                   Servo7.setPosition(1);
                   Servo8.setPosition(-1);
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
                if (getRuntime() - StateStartTime > 0.2) {
                   Servo7.setPosition(0.5);
                   Servo8.setPosition(0.5);
                    if (getRuntime() - StateStartTime > 0.5)
                        Potato1.setVelocity(0);
                    shooterstate = ShooterState.Idle;
                }
                break;

        }


        }


    }
