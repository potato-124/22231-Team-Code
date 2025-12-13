package org.firstinspires.ftc.teamcode.FlyWheelTuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlyWheelPIDF extends OpMode {

    public DcMotorEx FlyWheelMotor;

    public double HighVelocity = 1500;
    public double LowVelocity = 900;

    double curTargetVelocity = HighVelocity;

    double F = 0;
    double P = 0;

    double[] StepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int StepIndex = 1;




    @Override
    public void init(){
        FlyWheelMotor = hardwareMap.get(DcMotorEx.class, "Potato1");
        FlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlyWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("init complete");

    }
    @Override
  public void loop(){
        //Get all our gamepad commands
        //Set target velocity
        //Update Telemetry

        if (gamepad1.yWasPressed()){
            if(curTargetVelocity == HighVelocity){
                curTargetVelocity = LowVelocity;

            }else{ curTargetVelocity = HighVelocity;

            }

        }
        if(gamepad1.bWasPressed()){
            StepIndex = (StepIndex + 1) % StepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            F -= StepSizes[StepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            F += StepSizes[StepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            P += StepSizes[StepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= StepSizes[StepIndex];
        }
        //Set new PIDF CoEfficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        FlyWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //Set Velocity
        FlyWheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = FlyWheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad U/D)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", StepSizes[StepIndex]);

   }

  }
