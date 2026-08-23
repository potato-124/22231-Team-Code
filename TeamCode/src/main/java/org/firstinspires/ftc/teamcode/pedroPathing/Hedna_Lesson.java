package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Hedna_Lesson extends OpMode {

        double stateStartTime = 0;
        private DcMotorEx Potato1;
        private DcMotorEx Potato2;
        private DcMotorEx Potato3;
        private Servo Servo7;
        private Servo Servo8;
        private Follower follower;
        private Timer pathTimer, OpModeTimer;

        public enum PathState{

            reverse_to_shooting,
            first_3,
            preload,
            load,
            load_shoot,
            second_3,
            preload2,
            load2,
            load_shooting2,
            third_3,
            preload3,
            load3,
            load_shooting3,
            fourth_3,
            Default,


        }
        PathState pathState;
        private final Pose startshootpose = new Pose(20.456981664315936,22.3469675599436, Math.toRadians(140));
    private final Pose shootpose = new Pose(43.40761636107194,99.77433004231312, Math.toRadians(140));
    private final Pose preloadpose = new Pose(43.471086036671366,83.90973201692523,Math.toRadians(180));
    private final Pose loadpose = new Pose(12.119887165021163,83.68406205923836, Math.toRadians(180));
    private final Pose shootpose1 = new Pose(43.473906911142464,99.81241184767278, Math.toRadians(140));
    private final Pose preloadpose1 = new Pose(43.222849083215806,59.35966149506348, Math.toRadians(180));
    private final Pose loadpose1 = new Pose(11.983074753173506,59.16784203102963, Math.toRadians(180));
    private final Pose shootpose2 = new Pose(43.210155148095915,99.5937940761636, Math.toRadians(140));
    private final Pose preloadpose3 = new Pose(12.086036671368124,35.31452750352609, Math.toRadians(140));
    private final Pose shootpose3 = new Pose(43.710638297872336,99.22336524322542, Math.toRadians(140));
    private PathChain drivetoshooting;
    private PathChain shootpreload;
    private PathChain xx; //rename the xx just needed to remove source of error -Z













    public void setPathState(Hedna_Lesson.PathState newState) {
        pathState = newState;
        stateStartTime = getRuntime();

   // @Override
   // public void init() {
        //follower.update();
       // statePathUpdate();
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X:", follower.getPose().getX());
        telemetry.addData("Y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Path Time:", pathTimer.getElapsedTimeSeconds());}

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        follower.update();
        //statePathUpdate();
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X:", follower.getPose().getX());
        telemetry.addData("Y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.addData("Path Time:", pathTimer.getElapsedTimeSeconds());


    }
}
