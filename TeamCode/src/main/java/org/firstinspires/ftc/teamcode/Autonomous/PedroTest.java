package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.FollowPathAction;

public class PedroTest extends LinearOpMode {
    Follower follower;
    private Path scorePreload, park;
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));
    public void runOpMode(){

        while (opModeInInit()&&!isStopRequested()){
            follower.setStartingPose(new Pose(9, 111, Math.toRadians(270)));
        }
        waitForStart();
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new FollowPathAction(follower, scorePreload)
                        )
                )
        );
    }
}
