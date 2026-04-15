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
public class Auto_BClose_ZTest extends OpMode {
    double stateStartTime = 0;
    private DcMotorEx Potato1;
    private DcMotorEx Potato2;
    private DcMotorEx Potato3;
    private Servo Servo7;
    private Servo Servo8;
    private Follower follower;
    private Timer pathTimer, OpModeTimer;

    public enum PathState {
        //from start to end position
        //drive state
        //shoot state
        drive_start_shoot,
        shoot_preload,
        shoot,
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
        shoot_final,
        Default,


    }

    PathState pathState;
    private final Pose startPose = new Pose(20.45698166431594, 122.9562764456982, Math.toRadians(140));
    private final Pose shootPose = new Pose(43.20451339915375, 99.77433004231311, Math.toRadians(140));
    private final Pose preloadpose = new Pose(43.471086036671366, 83.90973201692523, Math.toRadians(180));
    private final Pose load = new Pose(10.497351953753556, 84.08969586205525, Math.toRadians(180));
    private final Pose shoot2 = new Pose(43.473906911142464, 99.81241184767278, Math.toRadians(140));
    private final Pose preload2 = new Pose(43.222849083215806, 59.35966149506348, Math.toRadians(180));
    private final Pose load2 = new Pose(10.563356443314351, 59.776292735254984, Math.toRadians(180));
    private final Pose shoot3 = new Pose(71.56800889966031, 119.61585649297761, Math.toRadians(140));
    private final Pose preload3 = new Pose(43.210155148095915, 35.78843441466854, Math.toRadians(180));
    private final Pose load3 = new Pose(10.463501460100522, 36.12579510915989, Math.toRadians(180));
    private final Pose shoot4 = new Pose(71.56800889966031, 119.61585649297761, Math.toRadians(140));
    private final Pose finalPos = new Pose(86.77927650529412, 123.67219452114661, Math.toRadians(140));
    private PathChain driveStartShoot;
    private PathChain shootPreload;
    private PathChain preloadLoad;
    private PathChain loadShoot1;
    private PathChain shootPreload2;
    private PathChain preloadLoad2;
    private PathChain LoadShoot2;
    private PathChain shootPreload3;
    private PathChain preloadLoad3;
    private PathChain LoadShoot3;
    private PathChain ShootFinal;

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootPreload = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preloadpose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preloadpose.getHeading())
                .build();
        preloadLoad = follower.pathBuilder()
                .addPath(new BezierLine(preloadpose, load))
                .setTangentHeadingInterpolation()
                .build();
        loadShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(load, shoot2))
                .setLinearHeadingInterpolation(load.getHeading(), shoot2.getHeading())
                .build();
        shootPreload2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, preload2))
                .setLinearHeadingInterpolation(shoot2.getHeading(), preload2.getHeading())
                .build();
        preloadLoad2 = follower.pathBuilder()
                .addPath(new BezierLine(preload2, load2))
                .setTangentHeadingInterpolation()
                .build();
        LoadShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(load2, shoot3))
                .setLinearHeadingInterpolation(load2.getHeading(), shoot3.getHeading())
                .build();
        shootPreload3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot3, preload3))
                .setLinearHeadingInterpolation(shoot3.getHeading(), preload3.getHeading())
                .build();
        preloadLoad3 = follower.pathBuilder()
                .addPath(new BezierLine(preload3, load3))
                .setTangentHeadingInterpolation()
                .build();
        LoadShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(load3, shoot4))
                .setLinearHeadingInterpolation(load3.getHeading(), shoot4.getHeading())
                .build();
        ShootFinal = follower.pathBuilder()
                .addPath(new BezierLine(shoot4, finalPos))
                .setLinearHeadingInterpolation(shoot4.getHeading(), finalPos.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch (pathState) {
            case drive_start_shoot:
                follower.setMaxPower(0.7);
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.shoot);
                break;

            case shoot:
                if (!follower.isBusy()) {
                    Potato1.setVelocity(1200);     //TODO ADD DELAY AND REDUCE SPEED
                    if (Math.abs(1200 - Potato1.getVelocity()) < 10) {
                        intake(1, 1);

                        setPathState(PathState.shoot_preload);
                    }
                }
                break;

            case shoot_preload:
                if(!follower.isBusy() && getRuntime() - stateStartTime > 3) {
                    intake(0, 0.5);
                    Potato1.setVelocity(0);
                    follower.followPath(shootPreload, true);
                    setPathState(PathState.preload_load);
                    telemetry.addLine("Done Path 1");
                    setPathState(PathState.preload_load);
                }
                break;

            case preload_load:
                if(!follower.isBusy()){
                    intake(1, 1);
                    follower.setMaxPower(0.5);
                    follower.followPath(preloadLoad, true);
                    setPathState(PathState.load_shoot);
                }
                break;

            case load_shoot:
                if(!follower.isBusy()){
                    intake(0, 0);
                    follower.setMaxPower(0.7);
                    follower.followPath(loadShoot1, true);
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
                if(!follower.isBusy() && getRuntime() - stateStartTime > 3){
                    intake(0, 0.5);
                    Potato1.setVelocity(0);
                    follower.followPath(shootPreload2, true);
                    setPathState(PathState.preload_load2);
                    telemetry.addLine("Done Path 2");
                    setPathState(PathState.preload_load2);
                }
                break;

            case preload_load2:
                if(!follower.isBusy()){
                    intake(1, 1);
                    follower.setMaxPower(0.5);
                    follower.followPath(preloadLoad2, true);
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
                if(!follower.isBusy()){
                    Potato1.setVelocity(1300);
                    if (Math.abs(1300 - Potato1.getVelocity()) < 10) {
                        intake(1, 1);
                        setPathState(PathState.shoot_preload3);
                    }
                }
                break;

            case shoot_preload3:
                if(!follower.isBusy() && getRuntime() - stateStartTime > 3){
                    intake(0, 0.5);
                    Potato1.setVelocity(0);
                    follower.followPath(shootPreload3, true);
                    setPathState(PathState.preload_load3);
                    telemetry.addLine("Done Path 3");
                    setPathState(PathState.preload_load3);
                }
                break;

            case preload_load3:
                if(!follower.isBusy()){
                    intake(1, 1);
                    follower.setMaxPower(0.5);
                    follower.followPath(preloadLoad3, true);
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
                        setPathState(PathState.shoot_final);
                    }
                }
                break;

            case shoot_final:
                if(!follower.isBusy() && getRuntime() - stateStartTime > 3){
                    intake(0,0);
                    Potato1.setVelocity(0);
                    follower.followPath(ShootFinal);
                    setPathState(PathState.Default);
                }
            break;

            case Default:
                telemetry.addLine("No state commanded");
                break;

        }


    }


    public void setPathState(PathState newState) {
        pathState = newState;
        stateStartTime = getRuntime();
    }

    @Override
    public void init() {
        pathState = PathState.drive_start_shoot;
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
        follower.setPose(startPose);

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