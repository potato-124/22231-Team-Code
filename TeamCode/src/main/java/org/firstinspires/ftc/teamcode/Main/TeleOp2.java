

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

    //Declare Variables
    boolean ReverseDrive;
    float LeftStickY;
    float LeftStickX;
    int Toggle;
    int Toggle2;
    double error;
    boolean CircleWasPressed;
    boolean SquareWasPressed;
    float RightStickX;
    double TargetVelocity;
    double[] Velocity = { 900, 1200, 1500, 2100};

    int VelocityIndex = 1;
    ShooterState shooterstate = ShooterState.Idle;
    double StateStartTime = 0;
    boolean motorIsRevving = false;

    public enum ShooterState {
        Idle,
        Shoot,
        Juggle,
        Reverse,

        Outtake,

    }


    @Override
    public void init() {
        LFMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LBMotor = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RBMotor = hardwareMap.get(DcMotorEx.class, "RB Motor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        Potato1 = hardwareMap.get(DcMotorEx.class, "Potato1");
        Servo7 = hardwareMap.get(Servo.class, "Servo 7");
        Servo8 = hardwareMap.get(Servo.class, "Servo 8");

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
        Potato1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Potato1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(240.0, 0, 0.5, 15.8240);
        Potato1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);



    }

    //Below is the main loop that runs during the OpMode
    @Override
    public void loop() {
        DriveControl();
        Telemetry();
        ShooterLogic();
        Reverse_Drive();


    }

    //Below is what controls the driving mechanism
    private void DriveControl() {
        LFMotor.setPower(Range.clip(LeftStickY - RightStickX + LeftStickX, -1, 1));
        LBMotor.setPower(Range.clip(LeftStickY - RightStickX - LeftStickX, -1, 1));
        RBMotor.setPower(Range.clip(LeftStickY + RightStickX + LeftStickX, -1, 1));
        RFMotor.setPower(Range.clip(LeftStickY + RightStickX - LeftStickX, -1, 1));
    }


    // Below is what controls the what shows up on the robot driver dashboard
    private void Telemetry() {

        if (ReverseDrive) {
            telemetry.addData("Reverse Drive", "On");

        }
        if (!ReverseDrive) {
            telemetry.addData("Reverse Drive", "Off");

        }
        telemetry.addData("Current Velocity", Potato1.getVelocity());
        telemetry.addData("Target Velocity", TargetVelocity);
        telemetry.addData("Error", error);

        telemetry.addData("State", shooterstate);
        telemetry.addData("Revving", Toggle2);
        telemetry.update();
    }

    //below is the Code controlling the ability to reverse directions
    private void Reverse_Drive() {
        if (ReverseDrive) {
            LeftStickY = -gamepad1.left_stick_y;
            RightStickX = gamepad1.right_stick_x;
            LeftStickX = -gamepad1.left_stick_x;
        } else {

            LeftStickY = gamepad1.left_stick_y;
            RightStickX = gamepad1.right_stick_x;
            LeftStickX = gamepad1.left_stick_x;
        }
        if (gamepad1.circleWasReleased() && !CircleWasPressed) {
            Toggle += 1;
            CircleWasPressed = true;

        } else {
            CircleWasPressed = false;

        }
        ReverseDrive = Toggle == 2;
        if (Toggle > 2) {
            Toggle = 1;
        }


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
            case Idle:
                if (gamepad1.rightBumperWasPressed()) {
                    Potato1.setVelocity(TargetVelocity);
                    StateStartTime = getRuntime();
                    shooterstate = ShooterState.Shoot;
                }
                if (gamepad1.leftBumperWasPressed()) {
                    Potato1.setVelocity(450);
                    StateStartTime = getRuntime();
                    shooterstate = ShooterState.Juggle;
                }
                if (gamepad1.triangleWasPressed()) {
                    Potato1.setVelocity(-600);
                    StateStartTime = getRuntime();
                    shooterstate = ShooterState.Reverse;
                }
                if (shooterstate == ShooterState.Idle){


                    if(Toggle2 == 2){
                        Potato1.setVelocity(1450);
                        motorIsRevving = true;
                    } else {motorIsRevving = false;}
                    if(Potato1.getVelocity() > 50 && !motorIsRevving || Potato1.getVelocity() <-50 && !motorIsRevving){
                        Potato1.setVelocity(1);
                    }
                    break;
                }

            case Shoot:


                if (error < 10) {
                    Servo7.setPosition(-1);
                    Servo8.setPosition(1);
                    shooterstate = ShooterState.Outtake;
                    StateStartTime = getRuntime();

                }

                break;

            case Juggle:

                if (getRuntime() - StateStartTime > 0.4) {
                    Servo7.setPosition(-1);
                    Servo8.setPosition(1);
                    if (getRuntime() - StateStartTime > 0.75) {
                        Servo7.setPosition(0.5);
                        Servo8.setPosition(0.5);
                        if (getRuntime() - StateStartTime > 1.05){
                            Potato1.setVelocity(0);
                            shooterstate = ShooterState.Idle;
                        }



                    }


                }
                break;

            case Reverse:

                if (getRuntime() - StateStartTime > 0.3) {
                    Servo7.setPosition(1);
                    Servo8.setPosition(-1);
                    if (getRuntime() - StateStartTime > 0.6) {
                        Potato1.setVelocity(0);
                        Servo7.setPosition(0.5);
                        Servo8.setPosition(0.5);
                        shooterstate = ShooterState.Idle;

                    }


                }
                break;
            case Outtake:
                if(getRuntime() - StateStartTime > 0.3){
                    Servo7.setPosition(0.5);
                    Servo8.setPosition(0.5);
                    if(getRuntime() - StateStartTime > 0.6)
                        Potato1.setVelocity(0);
                    shooterstate = ShooterState.Idle;
                }


        }


    }
}