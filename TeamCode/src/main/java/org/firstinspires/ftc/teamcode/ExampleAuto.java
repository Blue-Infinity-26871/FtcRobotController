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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "3 Ball Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    private int shootState;   // 0 = idle, 1 = spin-up, 2 = transfer, 3 = done

    // Shooter / transfer hardware
    private DcMotor M6;       // transfer
    private DcMotor M7;       // shooter
    private Servo   S1;       // feeder servo

    // --- 3-Ball Auto Poses ---

    public final Pose startPose = new Pose(79.7628, 137.0135, Math.toRadians(180));
=======
=======
=======

    public final Pose startPose = new Pose(79.7628, 137.0135, Math.toRadians(0));
>>>>>>> Stashed changes

    public final Pose startPose = new Pose(79.7628, 137.0135, Math.toRadians(0));
>>>>>>> Stashed changes

    public final Pose startPose = new Pose(79.7628, 137.0135, Math.toRadians(0));

>>>>>>> Stashed changes
    public final Pose scorePose = new Pose(98.58760107816713, 114.30727762803235, Math.toRadians(37));

    // Ball 1 (rightmost of row 1)
    public final Pose pickup1Pose = new Pose(
            103.43935309973047, 84.22641509433961, Math.toRadians(0)
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
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        // Preload to first score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

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
                // Drive from start to first scoring position
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                // FIRST SCORE AT scorePose:
                //  - when we arrive, spin up M7
                //  - after 5 sec, start M6 (transfer)
                //  - after feed time, move to grabPickup1
                if (!follower.isBusy()) {
                    handleFirstScore();
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    // After first pickup, go back to score
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    // Score second ball
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    // Grab third
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()) {
                    // Score third
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    // Grab fourth
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    // Done
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * Handles the first scoring sequence:
     *  - shootState 0: start spinning shooter (M7), start timer
     *  - shootState 1: after 5 sec, start transfer (M6) + open S1
     *  - shootState 2: after 1.5 sec feeding, stop everything and go to grabPickup1
     */
    private void handleFirstScore() {
        switch (shootState) {
            case 0:
                // Spin up shooter
                M7.setPower(1.0);       // same as TeleOp "full speed"
                S1.setPosition(0.0);    // keep gate closed while spinning up
                actionTimer.resetTimer();
                shootState = 1;
                break;

            case 1:
                // Wait 5 seconds before feeding
                if (actionTimer.getElapsedTimeSeconds() >= 5.0) {
                    // Start transfer + open gate
                    M6.setPower(1.0);
                    S1.setPosition(0.5); // same "open" as TeleOp when m7Power > 0.4
                    actionTimer.resetTimer();
                    shootState = 2;
                }
                break;

            case 2:
                // Let it feed for a bit, then move on
                if (actionTimer.getElapsedTimeSeconds() >= 1.5) { // tweak feed time as needed
                    // Stop shooter & transfer
                    M6.setPower(0.0);
                    M7.setPower(0.0);
                    S1.setPosition(0.0);

                    // Now go grab the first pickup, holding end point
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                    shootState = 3;  // finished this scoring sequence
                }
                break;

            case 3:
                // Do nothing – we've already started the next path
                break;
        }
    }

    /** Change path state and reset its timer **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("shoot state", shootState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Map shooter/transfer hardware (same names as TeleOp)
        M6 = hardwareMap.get(DcMotor.class, "M6");
        M7 = hardwareMap.get(DcMotor.class, "M7");
        S1 = hardwareMap.get(Servo.class, "S1");

        // Match TeleOp behavior
        M7.setDirection(DcMotor.Direction.REVERSE);  // same as TeleOp
        M6.setPower(0.0);
        M7.setPower(0.0);
        S1.setPosition(0.0);

        shootState = 0;

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        shootState = 0;
        setPathState(0);
    }

    @Override
<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    public void stop() {
        // Safety: make sure everything is off
        if (M6 != null) M6.setPower(0.0);
        if (M7 != null) M7.setPower(0.0);
        if (S1 != null) S1.setPosition(0.0);
    }
=======
    public void stop() {}
>>>>>>> Stashed changes
=======
    public void stop() {}
>>>>>>> Stashed changes
=======
    public void stop() {}
>>>>>>> Stashed changes
}