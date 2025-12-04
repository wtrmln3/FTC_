package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Import draw methods from your Drawing class
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;

@Autonomous(name="Test Auto", group = "Examples")
public class TestAuto extends OpMode {

    public static Follower follower;

    private Timer opmodeTimer;
    private int pathState;

    // Poses
    private final Pose startPose    = new Pose(25.7, 129.6, Math.toRadians(143));
    private final Pose scorePose    = new Pose(49.8, 107.4, Math.toRadians(143));
    private final Pose pickup1Pose  = new Pose(24.6, 93.0, Math.toRadians(90));

    // Paths
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1;

    @Override
    public void init() {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Create follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build paths
        buildPaths();
        drawOnlyCurrent();
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    private void buildPaths() {
        // Start → Score
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Score → Pickup1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Pickup1 → Score
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Handle autonomous path logic
        autonomousPathUpdate();

        // Draw robot and paths safely by passing follower
        draw();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start → Score
                follower.followPath(scorePreload);
                pathState = 1;
                break;

            case 1: // Wait until path finished, then go to pickup
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    pathState = 2;
                }
                break;

            case 2: // Wait until pickup path finished, then return to score
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    pathState = 3;
                }
                break;

            case 3: // Finished all paths
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
    }

    @Override
    public void stop() {}
}
