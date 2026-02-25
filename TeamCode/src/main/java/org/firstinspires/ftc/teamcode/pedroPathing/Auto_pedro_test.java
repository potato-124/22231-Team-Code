package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Auto_pedro_test extends OpMode {
    double stateStartTime = 0;
    private DcMotorEx Potato1;
    private DcMotorEx Potato2;
    private DcMotorEx Potato3;
    private Servo Servo7;
    private Servo Servo8;
    private Follower follower;
    private Timer opModeTimer;


    public enum PathState {
        //Start position to end position
        //Drive > Movement state
        //Shoot > Attempt to score the artifact
        back_Up,
        shoot,
        move_To_CollectPos1,
        collect,
        move_To_ShootPos1,
        move_To_CollectPos2,
        collect2,
        move_To_ShootPos2,
        Default
    }

    PathState currentState;
    PathState nextState;
    private final Pose startPose = new Pose(21.000, 122.000, Math.toRadians(138));
    private final Pose shootPose = new Pose(44.000, 99.000, Math.toRadians(138));
    private final Pose collectStartPos1 = new Pose(44, 84, Math.toRadians(180));
    private final Pose collectEndPos1 = new Pose(16, 84, Math.toRadians(180));
    private final Pose collectStartPos2 = new Pose(44, 60, Math.toRadians(180));
    private final Pose collectEndPos2 = new Pose(15, 60, Math.toRadians(180));
    private final Pose collectStartPos3 = new Pose(44, 36, Math.toRadians(180));
    private final Pose collectEndPos3 = new Pose(17, 36, Math.toRadians(180));
    
    private PathChain start_To_Shoot;
    private PathChain shoot_To_CollectStart1;
    private PathChain collectStart1_To_CollectEnd1;
    private PathChain collectEnd1_To_Shoot;
    private PathChain shoot_To_CollectStart2;
    private PathChain collectStart2_To_CollectEnd2;

    private PathChain collectEnd2_To_Shoot;
    private PathChain shoot_To_CollectStart3;
    private PathChain collectStart3_To_CollectEnd3;
    private PathChain CollectEnd3_To_Shoot;


    public void buildPaths() {
        //put in coordinates for start pos then put in coordinates for end pos
        start_To_Shoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shoot_To_CollectStart1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectStartPos1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectStartPos1.getHeading())
                .build();
        collectStart1_To_CollectEnd1 = follower.pathBuilder()
                .addPath(new BezierLine(collectStartPos1, collectEndPos1))
                .setLinearHeadingInterpolation(180, 180 )
                .build();
        collectEnd1_To_Shoot = follower.pathBuilder()
                .addPath(new BezierLine(collectEndPos1, shootPose))
                .setLinearHeadingInterpolation(180, 138)
                .build();
        shoot_To_CollectStart2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectStartPos2))
                .setLinearHeadingInterpolation(138, 180)
                .build();
        collectStart2_To_CollectEnd2 = follower.pathBuilder()
                .addPath(new BezierLine(collectStartPos2, collectEndPos2))
                .setLinearHeadingInterpolation(180, 180)
                .build();
        collectEnd2_To_Shoot = follower.pathBuilder()
                .addPath(new BezierLine(collectEndPos2, shootPose))
                .setLinearHeadingInterpolation(180, 138)
                .build();


    }

    public void statePathUpdate() {
        switch (currentState) {
            case back_Up:
                follower.followPath(start_To_Shoot, true);
                setPathState(PathState.shoot);
                nextState = PathState.move_To_CollectPos1;
                break;
            case shoot:
                if (!follower.isBusy()) {
                    Potato1.setVelocity(1200);
                    if (Math.abs(1200 - Potato1.getVelocity()) < 10) {
                        intake(1, 1);

                        setNextState();
                    }

                }
                break;
            case move_To_CollectPos1:
                if (getRuntime() - stateStartTime > 3){
                    intake(0, 0.5);
                    Potato1.setVelocity(0);
                follower.followPath(shoot_To_CollectStart1, true);
                    setPathState(PathState.collect);

                  }
                break;
            case collect:
                if(!follower.isBusy()){
                    intake(1, 1);
                    follower.followPath(collectStart1_To_CollectEnd1, true);
                    setPathState(PathState.move_To_ShootPos1);

                }

                break;
            case move_To_ShootPos1:
                if(!follower.isBusy()){
                    intake(0, 0);
                    follower.followPath(collectEnd1_To_Shoot);
                    setPathState(PathState.shoot);
                    nextState = PathState.move_To_CollectPos2;
                }
            case move_To_CollectPos2:
                if (getRuntime() - stateStartTime > 3){
                    intake(0, 0.5);
                    Potato1.setVelocity(0);
                    follower.followPath(shoot_To_CollectStart2, true);
                    setPathState(PathState.collect2);
                }
            case collect2:
                if(!follower.isBusy()){
                    intake(1, 1);
                    follower.followPath(collectStart2_To_CollectEnd2, true);
                    setPathState(PathState.move_To_ShootPos2);
                }
            case move_To_ShootPos2:

                if(!follower.isBusy()){
                    intake(0, 0);
                    follower.followPath(collectEnd2_To_Shoot);
                    setPathState(PathState.shoot);
                    nextState = PathState.Default;
                }
            case Default:
                telemetry.addLine("no state commanded");




        }
    }

    public void setPathState(PathState newState) {
        currentState = newState;
        stateStartTime = getRuntime();
    }
    public void setNextState(){
        currentState = nextState;
        stateStartTime = getRuntime();
    }

    @Override
    public void init() {
        currentState = PathState.back_Up;
        nextState = PathState.Default;
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //Add in the rest of your init mechanisms
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
        follower.setPose(startPose);


    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(currentState);
    }

    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("State", currentState);

    }

    public void intake(double motorPower,double servoSpeed) {
        Potato2.setPower(motorPower);
        Potato3.setPower(motorPower);
        Servo7.setPosition(servoSpeed);
        Servo8.setPosition(servoSpeed * -1);
        if(servoSpeed == 0.5){
            Servo7.setPosition(0.5);
            Servo8.setPosition(0.5);
        }


    }

}
