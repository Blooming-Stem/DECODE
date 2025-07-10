package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystem.FinalRobot;
import org.psilynx.psikit.Logger;
import org.psilynx.psikit.RLOGServer;
import org.psilynx.psikit.wpi.Pose2d;
import org.psilynx.psikit.wpi.Rotation2d;
import org.psilynx.psikit.wpi.Translation2d;

@Autonomous(name="monkeytest")
public class PedroTest extends LinearOpMode {

    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(8.173, 56.043, Point.CARTESIAN),
                            new Point(38.724, 68.303, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierCurve(
                            new Point(38.724, 68.303, Point.CARTESIAN),
                            new Point(0.195, 32.108, Point.CARTESIAN),
                            new Point(65.578, 33.276, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierCurve(
                            new Point(65.578, 33.276, Point.CARTESIAN),
                            new Point(79.589, 20.822, Point.CARTESIAN),
                            new Point(16.541, 29.578, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line4 = builder
            .addPath(
                    new BezierCurve(
                            new Point(16.541, 29.578, Point.CARTESIAN),
                            new Point(119.676, 15.373, Point.CARTESIAN),
                            new Point(16.151, 16.930, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line5 = builder
            .addPath(
                    new BezierCurve(
                            new Point(16.151, 16.930, Point.CARTESIAN),
                            new Point(138.357, 5.059, Point.CARTESIAN),
                            new Point(10.508, 9.341, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line6 = builder
            .addPath(
                    new BezierCurve(
                            new Point(10.508, 9.341, Point.CARTESIAN),
                            new Point(10.703, 51.373, Point.CARTESIAN),
                            new Point(38.919, 63.438, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line7 = builder
            .addPath(
                    new BezierCurve(
                            new Point(38.919, 63.438, Point.CARTESIAN),
                            new Point(10.119, 53.708, Point.CARTESIAN),
                            new Point(10.703, 23.157, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line8 = builder
            .addPath(
                    new BezierCurve(
                            new Point(10.703, 23.157, Point.CARTESIAN),
                            new Point(10.119, 57.211, Point.CARTESIAN),
                            new Point(38.335, 60.519, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line9 = builder
            .addPath(
                    new BezierCurve(
                            new Point(38.335, 60.519, Point.CARTESIAN),
                            new Point(5.059, 49.427, Point.CARTESIAN),
                            new Point(11.286, 22.962, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line10 = builder
            .addPath(
                    new BezierCurve(
                            new Point(11.286, 22.962, Point.CARTESIAN),
                            new Point(3.503, 45.924, Point.CARTESIAN),
                            new Point(38.724, 65.773, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line11 = builder
            .addPath(
                    new BezierCurve(
                            new Point(38.724, 65.773, Point.CARTESIAN),
                            new Point(5.059, 48.065, Point.CARTESIAN),
                            new Point(10.897, 23.157, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain line12 = builder
            .addPath(
                    new BezierCurve(
                            new Point(10.897, 23.157, Point.CARTESIAN),
                            new Point(10.897, 55.265, Point.CARTESIAN),
                            new Point(38.724, 71.416, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();
    private final Pose startPose = new Pose(8, 111, Math.toRadians(0));
    private Path startToCornerBasket,cornerToPickUp1,pickUp1ToCorner,cornerToPickUp2,pickUp2toCorner,cornerToPickUp3, pickUp3ToCorner;


    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));
    public void runOpMode(){
        FinalRobot robot = new FinalRobot(this);
        while(opModeInInit()&&!isStopRequested()) {



            robot.follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            robot.follower.setStartingPose(new Pose(8, 56, Math.toRadians(0)));
            robot.follower.setMaxPower(0.8);


            startToCornerBasket = new Path(
                    new BezierLine(
                            new Point(8.075, 111.925, Point.CARTESIAN),
                            new Point(20.458, 123.542, Point.CARTESIAN)
                    )
            );
            startToCornerBasket.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));

            cornerToPickUp1 = new Path(
                    new BezierLine(
                            new Point(20.458, 123.542, Point.CARTESIAN),
                            new Point(32.748, 121.346, Point.CARTESIAN)
                    )
            );
            cornerToPickUp1.setConstantHeadingInterpolation(Math.toRadians(0));

            pickUp1ToCorner = new Path(
                    new BezierLine(
                            new Point(32.748, 121.346, Point.CARTESIAN),
                            new Point(20.458, 123.542, Point.CARTESIAN)
                    )
            );
            pickUp1ToCorner.setConstantHeadingInterpolation(Math.toRadians(0));


            cornerToPickUp2 = new Path(
                    new BezierLine(
                            new Point(20.458, 123.542, Point.CARTESIAN),
                            new Point(32.972, 131.888, Point.CARTESIAN)
                    )
            );
            cornerToPickUp2.setConstantHeadingInterpolation(Math.toRadians(0));

            pickUp2toCorner = new Path(
                    new BezierLine(
                            new Point(32.972, 131.888, Point.CARTESIAN),
                            new Point(20.458, 123.542, Point.CARTESIAN)
                    )
            );
            pickUp2toCorner.setConstantHeadingInterpolation(Math.toRadians(0));
            robot.updateLoggerInit();





        }
        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        robot.updateLoggerBeginning(),


                        new SequentialAction(
                                    new FollowPathAction(robot.follower, line1,true),
    //new SleepAction(.1),
                                    new FollowPathAction(robot.follower, line2,true),
    //new SleepAction(.1),
                                    new FollowPathAction(robot.follower, line3,true),
    //new SleepAction(.1),
                                    new FollowPathAction(robot.follower, line4,true),
    //new SleepAction(.1),
            new FollowPathAction(robot.follower, line5,true),
    //new SleepAction(.1),
                    new FollowPathAction(robot.follower, line6,true),
    //new SleepAction(.1),
                    new FollowPathAction(robot.follower, line7,true),
    //new SleepAction(.1),
                    new FollowPathAction(robot.follower, line8,true),
    //new SleepAction(.1),
            new FollowPathAction(robot.follower, line9,true),
    //new SleepAction(.1),
                    new FollowPathAction(robot.follower, line10,true),
    //new SleepAction(.1),
                    new FollowPathAction(robot.follower, line11,true),
    //new SleepAction(.1),
                    new FollowPathAction(robot.follower, line12,true)
                        ),
                        robot.updateFolowerAction(),
                        robot.updateLogger(),
                        robot.updateLoggerEnd()
                )

//                )
       );

    }

}
