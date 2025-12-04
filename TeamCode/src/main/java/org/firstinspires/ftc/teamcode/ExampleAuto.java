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
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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


    public final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    public final Pose scorePose = new Pose(7.5, 0, Math.toRadians(0));

    // Ball 1 (rightmost of row 1)
    public final Pose leavePose = new Pose(
            24, 0, Math.toRadians(0)
    );
    public final Pose pickup1Pose = new Pose(
            0, 24, Math.toRadians(0)
    );
    // Ball 2 (rightmost of row 2)
    public final Pose pickup1PoseEnd = new Pose(
            124.78706199460916, 84.03234501347708, Math.toRadians(0)
    );

    // Ball 3 (rightmost of row 3)
    public final Pose pickup2Pose = new Pose(
            103.24528301886792, 59.579514824797855, Math.toRadians(0)
    );
    public final Pose pickup2PoseEnd = new Pose(
            125.36927223719675, 59.38544474393531, Math.toRadians(0)
    );
    public final Pose pickup3Pose = new Pose(
            107.1266846361186, 35.1266846361186, Math.toRadians(0)
    );
    public final Pose pickup3PoseEnd = new Pose(
            125.36927223719675, 35.32075471698113, Math.toRadians(0)
    );

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1,leave1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        // Preload to first score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        leave1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, scorePose))
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
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                scorePreload.setBrakingStrength(0.2);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    if(pathTimer.getElapsedTimeSeconds() > 5) {

                        M7.setPower(0.92);
                        S1.setPosition(0.5);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 8) {
                        scorePreload.setBrakingStrength(0.2);
                        M8.setPower(-1.0);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 11) {
                        M8.setPower(0.0);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 13) {
                        scorePreload.setBrakingStrength(0.2);
                        M8.setPower(-1.0);
                        M5.setPower(1.0);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 16) {
                        M7.setPower(0.86);
                        M5.setPower(0.0);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 19) {
                        scorePreload.setBrakingStrength(0.2);
                        M8.setPower(-1.0);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 23) {
                        scorePreload.setBrakingStrength(0.2);
                        M8.setPower(0.0);
                        M7.setPower(0.0);
                        S1.setPosition(0.0);
                    }

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                 //   follower.followPath(grabPickup1,true);
                    if(pathTimer.getElapsedTimeSeconds() > 25) {
                        follower.followPath(leave1, true);
                        setPathState(2);
                    }

                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                  //  follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                 //   follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                  //  follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                   // follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                   // follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
/** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
@Override
public void loop() {

    // These loop the movements of the robot, these must be called continuously in order to work
    follower.update();
    autonomousPathUpdate();

    // Feedback to Driver Hub for debugging
    telemetry.addData("path state", pathState);
    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    telemetry.update();
}

/** This method is called once at the init of the OpMode. **/
@Override
public void init() {
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();


    follower = Constants.createFollower(hardwareMap);
    buildPaths();
    follower.setStartingPose(startPose);
    M5 = hardwareMap.get(DcMotor.class, "M5");
    M6 = hardwareMap.get(DcMotor.class, "M6");
    M7 = hardwareMap.get(DcMotor.class, "M7");
    M8 = hardwareMap.get(DcMotor.class, "M8");
    S1 = hardwareMap.get(Servo.class, "S1");
    M5.setDirection(DcMotor.Direction.REVERSE);
    M7.setDirection(DcMotor.Direction.REVERSE);
}

/** This method is called continuously after Init while waiting for "play". **/
@Override
public void init_loop() {}

/** This method is called once at the start of the OpMode.
 * It runs all the setup actions, including building paths and starting the path system **/
@Override
public void start() {
    opmodeTimer.resetTimer();
    setPathState(0);
}

/** We do not use this because everything should automatically disable **/
@Override
public void stop() {}
}
