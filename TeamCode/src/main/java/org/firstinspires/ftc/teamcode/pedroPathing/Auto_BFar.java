package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private final Pose startshootpose = new Pose(60.81754504459763,11.811756292337966, Math.toRadians(120));
    private final Pose preload = new Pose(60.615896223603976,35.24402550706212, Math.toRadians(180));
    private final Pose load = new Pose(6.298591549295773,35.6169014084507,Math.toRadians(180) );
    private final Pose shoot = new Pose(60.8478872164874,11.763380281690141, Math.toRadians(120));
    private final Pose preload2 = new Pose(60.53239436619719,60.3492957746479, Math.toRadians(180));
    private final Pose load2 = new Pose(6.1746478873239425,59.91830985915493, Math.toRadians(180));
    private final Pose shoot2 = new Pose(60.777464788732395,11.881690140845063, Math.toRadians(120));
    private final Pose preload3 = new Pose(60.10985915492959,83.96901408450702, Math.toRadians(180));
    private final Pose load3 = new Pose(6.811267605633803,84.11830985915492, Math.toRadians(180));
    private final Pose shoot3 = new Pose(60.61126760563381,11.97183098591549, Math.toRadians(120));
    private PathChain ShootPreload;
    private PathChain PreloadLoad;
    private PathChain LoadShoot;
    private PathChain ShootPreload2;
    private PathChain PreloadLoad2;
    private PathChain LoadShoot2;
    private PathChain ShootPreload3;
    private PathChain PreloadLoad3;
    private PathChain LoadShoot3;

    public void buildPaths(){
         ShootPreload = follower.pathBuilder()
                .addPath(new BezierLine(startshootpose,preload ))
                .setLinearHeadingInterpolation(startshootpose.getHeading(), preload.getHeading())
                .build();
         PreloadLoad = follower.pathBuilder()
                .addPath(new BezierLine(preload,load ))
                .setTangentHeadingInterpolation()
                .build();
         LoadShoot = follower.pathBuilder()
                .addPath(new BezierLine(load,shoot ))
                .setLinearHeadingInterpolation(load.getHeading(), shoot.getHeading())
                .build();
         ShootPreload2 = follower.pathBuilder()
                 .addPath(new BezierLine(shoot,preload2 ))
                 .setLinearHeadingInterpolation(shoot.getHeading(), preload2.getHeading())
                .build();
        PreloadLoad2 = follower.pathBuilder()
                .addPath(new BezierLine(preload2,load2 ))
                .setTangentHeadingInterpolation()
                .build();
        LoadShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(load2,shoot2 ))
                .setLinearHeadingInterpolation(load2.getHeading(), shoot2.getHeading())
                .build();
        ShootPreload3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2,preload3 ))
                .setLinearHeadingInterpolation(shoot2.getHeading(), preload3.getHeading())
                .build();
        PreloadLoad3 = follower.pathBuilder()
                .addPath(new BezierLine(preload3,load3 ))
                .setTangentHeadingInterpolation()
                .build();
        LoadShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(load3,shoot3 ))
                .setLinearHeadingInterpolation(load3.getHeading(), shoot3.getHeading())
                .build();

    }
    public void statePathUpdate(){
        switch (pathState){
            case shoot:
                if(!follower.isBusy() ){
                    Potato1.setVelocity(1300);
                    if (Math.abs(1300 - Potato1.getVelocity()) < 10) {
                        intake(1, 1);
                        setPathState(PathState.shoot_preload);
                    }
                }
                break;
            case shoot_preload:
                if(!follower.isBusy() && getRuntime() - stateStartTime > 3) {
                    intake(0, 0.5);
                    Potato1.setVelocity(0);
                    follower.followPath(ShootPreload, true);
                    setPathState(PathState.preload_load);
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.preload_load);
                }
                break;
            case preload_load:
                if(!follower.isBusy()){
                    intake(1, 1);
                    follower.setMaxPower(0.5);
                    follower.followPath(PreloadLoad, true);
                    setPathState(PathState.load_shoot);
                }
                break;
            case load_shoot:
                if(!follower.isBusy()){
                    intake(0, 0);
                    follower.setMaxPower(0.7);
                    follower.followPath(LoadShoot, true);
                    setPathState(PathState.shoot2);
                }
                break;
            case shoot2:
                if(!follower.isBusy() ){
                    Potato1.setVelocity(1300);
                    if (Math.abs(1300 - Potato1.getVelocity()) < 10) {
                        intake(1, 1);
                        setPathState(PathState.shoot_preload2);
                    }
                }
                break;
            case shoot_preload2:
                if(!follower.isBusy() && getRuntime() - stateStartTime > 3) {
                    intake(0, 0.5);
                    Potato1.setVelocity(0);
                    follower.followPath(ShootPreload2, true);
                    setPathState(PathState.preload_load2);
                    telemetry.addLine("Done Path 2");
                    setPathState(PathState.preload_load2);
                }
                break;
            case preload_load2:
                if(!follower.isBusy()){
                    intake(1, 1);
                    follower.setMaxPower(0.5);
                    follower.followPath(PreloadLoad2, true);
                    setPathState(PathState.load_shoot2);
                }
                break;
            case load_shoot2:
                if(!follower.isBusy()){
                    intake(0, 0);
                    follower.setMaxPower(0.7);
                    follower.followPath(LoadShoot2, true);
                    setPathState(PathState.shoot3);
                }
                break;
            case shoot3:
                if(!follower.isBusy() ){
                    Potato1.setVelocity(1300);
                    if (Math.abs(1300 - Potato1.getVelocity()) < 10) {
                        intake(1, 1);
                        setPathState(PathState.shoot_preload3);
                    }
                }
                break;
            case shoot_preload3:
                if(!follower.isBusy() && getRuntime() - stateStartTime > 3) {
                    intake(0, 0.5);
                    Potato1.setVelocity(0);
                    follower.followPath(ShootPreload3, true);
                    setPathState(PathState.preload_load3);
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.preload_load3);
                }
                break;
            case preload_load3:
                if(!follower.isBusy()){
                    intake(1, 1);
                    follower.setMaxPower(0.5);
                    follower.followPath(PreloadLoad3, true);
                    setPathState(PathState.load_shoot3);
                }
                break;
            case load_shoot3:
                if(!follower.isBusy()){
                    intake(0, 0);
                    follower.setMaxPower(0.7);
                    follower.followPath(LoadShoot3, true);
                    setPathState(PathState.shoot4);
                }
                break;
            case shoot4:
                if(!follower.isBusy() ){
                    Potato1.setVelocity(1300);
                    if (Math.abs(1300 - Potato1.getVelocity()) < 10) {
                        intake(1, 1);
                        setPathState(PathState.Default);
                    }
                }
                break;
            case Default:
                telemetry.addLine("No state commanded");
                break;
        }
    }


    public void setPathState(Auto_BFar.PathState newState) {
        pathState = newState;
        stateStartTime = getRuntime();
    }

    @Override
    public void init() {
        pathState = Auto_BFar.PathState.shoot;
        pathTimer = new Timer();
        OpModeTimer = new Timer();
        OpModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add other mechanisms
        Potato1 = hardwareMap.get(DcMotorEx.class, "Potato1");
        Potato2 = hardwareMap.get(DcMotorEx.class, "Potato2");
        Potato3 = hardwareMap.get(DcMotorEx.class, "Potato3");
        Servo7 = hardwareMap.get(Servo.class, "Servo 7");
        Servo8 = hardwareMap.get(Servo.class, "Servo 8");
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(240.0, 0, 0.5, 12.8240);
        Potato1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Potato2.setDirection(DcMotorSimple.Direction.REVERSE);
        Potato3.setDirection(DcMotorSimple.Direction.REVERSE);

        buildPaths();
        follower.setPose(startshootpose);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X:", follower.getPose().getX());
        telemetry.addData("Y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Path Time:", pathTimer.getElapsedTimeSeconds());

    }
    public void intake(double motorPower,double servoSpeed) {
        Potato2.setPower(motorPower);
        Potato3.setPower(motorPower);
        Servo7.setPosition(servoSpeed);
        Servo8.setPosition(servoSpeed * -1);
        if (servoSpeed == 0.5) {
            Servo7.setPosition(0.5);
            Servo8.setPosition(0.5);
        }
    }
}
