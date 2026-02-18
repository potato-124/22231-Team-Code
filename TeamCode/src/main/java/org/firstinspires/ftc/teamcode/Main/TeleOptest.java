package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOptest")
public class TeleOptest extends OpMode {
    DcMotorEx LFMotor;
    DcMotorEx LBMotor;
    DcMotorEx RBMotor;
    DcMotorEx RFMotor;
    DcMotorEx Potato1;
    Servo Servo7;
    Servo Servo8;
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
    public void init() {
        // Declare Motors and Servos

        LFMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LBMotor = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RBMotor = hardwareMap.get(DcMotorEx.class, "RB Motor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        Potato1 = hardwareMap.get(DcMotorEx.class, "Potato1");
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
        @Override
        public void loop() {
            ShooterLogic();
        }

    private void ShooterLogic() {
    }

}