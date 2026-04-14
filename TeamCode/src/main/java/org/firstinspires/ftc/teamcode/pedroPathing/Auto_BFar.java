package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Auto_BFar extends OpMode {
    double stateStartTime = 0;
    private DcMotorEx Potato1;
    private DcMotorEx Potato2;
    private DcMotorEx Potato3;
    private Servo Servo7;
    private Servo Servo8;
    private Follower follower;
    private Timer pathTimer, OpModeTimer;

    public enum PathState {
        shoot,
        shoot_preload,
        preload_load,
        load_shoot,
        shoot2,
        shoot_preload2,
        preload_load2,
        load_shoot2,
        shoot3,
        shoot_preload3,
        preload_load3,
        load_shoot3,
        shoot4,
        Default,
    }
    PathState pathState;
    private final Pose startshootpose = new Pose();





    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
