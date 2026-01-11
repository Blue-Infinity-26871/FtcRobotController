package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "3 Ball Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private int shootState;   // 0 = idle, 1 = spin-up, 2 = transfer, 3 = done

    // Shooter / transfer hardware
    private DcMotor M5;
    private DcMotor M6;       // transfer
    private DcMotor M7;       // shooter
    private DcMotor M8;
    private Servo S1;       // feeder servo


    public final Pose startPose = new Pose(0, 0, Math.toRadians(180));
    public final Pose scorePose = new Pose(7.5, 0, Math.toRadians(135));

    public final Pose pickup1Pose = new Pose(
            0, 24, Math.toRadians(0)
    );
    // Ball 2 (rightmost of row 2)
    // public final Pose pickup1PoseEnd = new Pose(
    //124.78706199460916, 84.03234501347708, Math.toRadians(0)
    //);

    // Ball 3 (rightmost of row 3)
    public final Pose pickup2Pose = new Pose(
            103.24528301886792, 59.579514824797855, Math.toRadians(0)
    );
    //public final Pose pickup2PoseEnd = new Pose(
    //       125.36927223719675, 59.38544474393531, Math.toRadians(0)
    //);
    public final Pose pickup3Pose = new Pose(
            107.1266846361186, 35.1266846361186, Math.toRadians(0)
    );
    //public final Pose pickup3PoseEnd = new Pose(
    //       125.36927223719675, 35.32075471698113, Math.toRadians(0)
    //);

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        // Preload to first score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        //leave1 = follower.pathBuilder()
        //       .addPath(new BezierLine(scorePose, leavePose))
        //     .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
        //   .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }
    @Override
    public void loop() {

        follower.update();
        autonomoutPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update

    }

    private void autonomoutPathUpdate() {
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower, setStartingPose(startPose);

    }
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop(){}
}

   //