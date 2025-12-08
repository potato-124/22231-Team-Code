package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HelloWorld" , group = "OpMode")
public class HelloWorld extends OpMode {

  @Override
  public void init(){

        telemetry.addData("Hello:" , "World");

    }

    @Override
    public void loop(){


    }
}
