package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class SampleAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        // START POSITION_ENDPOSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD,

        DRIVE_SHOOT1POS_LINE1POS,
    }

    PathState pathState;

    private final Pose startPose = new Pose(117.88534658710252, 130.35451755137652, Math.toRadians(217));
    private final Pose shootPose1 = new Pose(98.58760107816713, 114.30727762803235, Math.toRadians(37));
    private final Pose linePose1 = new Pose(103.43935309973047, 84.22641509433961);
    // possible heading , Math.toRadians(0)
    private final Pose collectPose1 = new Pose(124.78706199460916, 84.03234501347708);
    // possible heading , Math.toRadians(0)
    private final Pose shootPose2 = new Pose(110.6199460916442, 110.03773584905659, Math.toRadians(44));
    private final Pose linePose2 = new Pose(103.43935309973047, 59.579514824797855);
    // possible heading , Math.toRadians(0)
    private final Pose collectPose2 = new Pose(125.36927223719675, 59.38544474393531);
    // possible heading , Math.toRadians(0)
    private PathChain driveStartPOSShoot1POS, driveShoot1POSLine1POS;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPOSShoot1POS = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();
        driveShoot1POSLine1POS = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, linePose1))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), linePose1.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
            follower.followPath(driveStartPOSShoot1POS, true);
            setPathState(PathState.SHOOT_PRELOAD); // reset the timer & make new state
            break;
            case SHOOT_PRELOAD:
                // check is follower done it's path?
                if (!follower.isBusy()) {
                    // TODO add logic to flywheel shooter
                    telemetry.addLine("Done State 1");
                    // transition to next state
                }
                break;
            case DRIVE_SHOOT1POS_LINE1POS:
                if (!follower.isBusy()) {
                    telemetry.addLine("Start State 2");
                }
            default:
                telemetry.addLine("No State Commanded");
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
        opModeTimer= new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);

    }


    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

    }

}
