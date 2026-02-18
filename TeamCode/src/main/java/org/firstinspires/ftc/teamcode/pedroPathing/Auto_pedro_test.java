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
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        //Start position to end position
        //Drive > Movement state
        //Shoot > Attempt to score the artifact
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS,
        Default
    }

    PathState pathState;
    private final Pose startPose = new Pose(20.53591563994872, 121.92513368983957, Math.toRadians(138));
    private final Pose shootPose = new Pose(36.193669650643905, 106.26737967914436, Math.toRadians(138));
    private final Pose endPose = new Pose(63.5721925133689863, 113.95721925133691, Math.toRadians(138));
    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosEndPos;

    public void buildPaths() {
        //put in coordinates for start pos then put in coordinates for end pos
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    Potato1.setVelocity(1200);
                    if (Math.abs(1200 - Potato1.getVelocity()) < 10) {
                        intake(1, 1);
                        setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                        stateStartTime = getRuntime();
                    }

                }
                break;
            case DRIVE_SHOOTPOS_ENDPOS:
                if (getRuntime() - stateStartTime > 3){
                    intake(0, 0.5);
                follower.followPath(driveShootPosEndPos, true);
                if (!follower.isBusy()) {
                    pathState = PathState.Default;

                }}
                break;
            case Default:
                Potato1.setVelocity(0);

                telemetry.addLine("No state commanded");
                break;


        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
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
        setPathState(pathState);
    }

    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("State", pathState);

    }

    public void intake(double motorPower,double servoSpeed) {
        Potato2.setPower(motorPower);
        Potato3.setPower(motorPower);
        Servo7.setPosition(servoSpeed * -1);
        Servo8.setPosition(servoSpeed);
        if(servoSpeed == 0.5){
            Servo7.setPosition(0.5);
            Servo8.setPosition(0.5);
        }


    }
}
