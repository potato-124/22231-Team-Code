package org.firstinspires.ftc.teamcode.Main;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoPedro extends OpMode {
    private Follower follower;
    private Timer pathtimer, OpModeTimer;
    public enum PathState {
        //From START POSITION-END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD

    }
    PathState pathstate;
    private final Pose startPose = new Pose();
    private final Pose shootPose = new Pose();
    private PathChain driveStartPosShootPos;

    public void buildPaths() {
        //Put in coordinates for starting Pose > End Pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

    }
    public void statePathUpdate() {
        switch(pathstate) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);//Reset timer and make new state
                break;
            case SHOOT_PRELOAD:
                //Check if follower is done with the path
                if(follower.isBusy()){
                    //TODO add flywheel logic
                    telemetry.addLine("Done Path 1");
                }
                break;
            default:
                telemetry.addLine("No State commanded");
                break;

        }
    }
    public void setPathState(PathState newState) {
        pathstate = newState;
        pathtimer.resetTimer();
    }
    @Override
    public void init(){
        pathstate = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathtimer = new Timer();
        OpModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);


    }

    public void start() {
        OpModeTimer.resetTimer();
        setPathState(pathstate);
    }


    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

    }
}
