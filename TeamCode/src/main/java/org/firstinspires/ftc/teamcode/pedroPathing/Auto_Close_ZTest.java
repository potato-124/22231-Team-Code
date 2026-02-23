package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
@Autonomous
public class Auto_Close_ZTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, OpModeTimer;
    public enum PathState{
        //from start to end position
        //drive state
        //shoot state
        drive_start_shoot,
        shoot_preload,


    }
    PathState pathState;
    private final Pose startPose = new Pose(20.45698166431594, 122.9562764456982, Math.toRadians(140));
    private final Pose shootPose = new Pose(43.20451339915375,99.77433004231311,Math.toRadians(140) );
    private PathChain driveStartShoot;
    public void buildPaths(){
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case drive_start_shoot:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.shoot_preload);
                break;
            case shoot_preload:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");

                }
                break;
            default:
                telemetry.addLine("No state commanded");
                break;
        }


    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.drive_start_shoot;
        pathTimer = new Timer();
        OpModeTimer = new Timer();
        OpModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add other mechanisms

        buildPaths();
        follower.setPose(startPose);

    }

    @Override
    public void loop() {

    }
}
