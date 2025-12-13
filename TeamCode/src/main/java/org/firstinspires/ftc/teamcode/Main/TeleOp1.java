package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp1")
public class TeleOp1 extends OpMode {

    // Declare Motors and Servos
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;
    private DcMotor RFMotor;
    private DcMotor Potato1;
    private Servo Servo7;
    private Servo Servo8;

    //Declare Variables
    double ShooterPower;
    boolean DownPressed;
    boolean ReverseDrive;
    float LeftStickY;
    boolean UpPressed;
    float RightStickY;
    int Toggle;
    boolean CircleWasPressed;
    float RightStickX;


        @Override
        public void init(){
            LFMotor = hardwareMap.get(DcMotor.class, "LF Motor");
            LBMotor = hardwareMap.get(DcMotor.class, "LB Motor");
            RBMotor = hardwareMap.get(DcMotor.class, "RB Motor");
            RFMotor = hardwareMap.get(DcMotor.class, "RF Motor");
            Potato1 = hardwareMap.get(DcMotor.class, "Potato1");
            Servo7 = hardwareMap.get(Servo.class, "Servo 7");
            Servo8 = hardwareMap.get(Servo.class, "Servo 8");

            ShooterPower = -0.6;
            DownPressed = false;
            UpPressed = false;
            Toggle = 1;
            CircleWasPressed = false;
            RBMotor.setDirection(DcMotor.Direction.REVERSE);
            RFMotor.setDirection(DcMotor.Direction.REVERSE);
            LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Potato1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        }
//Below is the main loop that runs during the OpMode
        @Override
    public void loop() {
        DriveControl ();
        IntakeLogic ();
        Telemetry();
        ShoterLogic();
        Reverse_Drive();


        }
//Below is what controls the driving mechanism
    private void DriveControl () {
        LFMotor.setPower(Range.clip(LeftStickY - RightStickX, -1, 1));
        LBMotor.setPower(Range.clip(LeftStickY + RightStickX, -1, 1));
        RBMotor.setPower(Range.clip(RightStickY - RightStickX, -1, 1));
        RFMotor.setPower(Range.clip(RightStickY + RightStickX, -1, 1));
    }

    // Below is what controls the intake into the shooter and the reverse intake.
    private void IntakeLogic () {
        if (gamepad1.left_bumper) {
            Servo7.setPosition(-1);
            Servo8.setPosition(1);
        } else if (gamepad1.leftBumperWasReleased()) {
            Servo7.setPosition(0.5);
            Servo8.setPosition(0.5);
        }
        if (gamepad1.triangle) {
            Potato1.setPower(1);
            Servo7.setPosition(1);
            Servo8.setPosition(-1);
        } else if (gamepad1.triangleWasReleased()) {
            Potato1.setPower(0);
            Servo7.setPosition(0.5);
            Servo8.setPosition(0.5);
        }



    }
    // Below is what controls the what shows up on the robot driver dashboard
    private void Telemetry(){

        telemetry.addData("Shooter Power", ShooterPower);
        if(ReverseDrive){
            telemetry.addData("Reverse Drive" , "On");

        }
        if(!ReverseDrive){
            telemetry.addData("Reverse Drive", "Off");

        }
        telemetry.update();
    }
//below is the Code controlling the ability to reverse directions
    private void Reverse_Drive(){
        if(ReverseDrive){
      LeftStickY = -gamepad1.right_stick_y;
      RightStickY = -gamepad1.left_stick_y;
      RightStickX = -gamepad1.right_stick_x;
        }
        else {

            LeftStickY = gamepad1.left_stick_y;
            RightStickY = gamepad1.right_stick_y;
            RightStickX = gamepad1.right_stick_x;
        }
        if(gamepad1.circleWasReleased() && !CircleWasPressed){
            Toggle += 1;
            CircleWasPressed = true;

        }
        else {
            CircleWasPressed = false;

        }
        if (Toggle == 2){
            ReverseDrive = true;
        }
        else {
            ReverseDrive = false;
        }
        if (Toggle > 2){
            Toggle = 1;
        }


    }

    //Below is the code for controlling the Shooter
    private void ShoterLogic(){
        if(gamepad1.dpad_up && !UpPressed){
            ShooterPower -= 0.1;
            UpPressed = true;

        }
        else if (!gamepad1.dpad_up) {
            UpPressed = false;
        }
        if(gamepad1.dpad_down && !DownPressed){
            ShooterPower += 0.1;
            DownPressed = true;
        }
        else if (!gamepad1.dpad_down)
        {DownPressed = false;}
        if(ShooterPower < -1)
        {ShooterPower = -1;}
        if(ShooterPower > 0)
        {ShooterPower = 0;}
        if(gamepad1.right_bumper)
        {Potato1.setPower(ShooterPower);}
        else
        {Potato1.setPower(0);}
    }
}

