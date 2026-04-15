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
