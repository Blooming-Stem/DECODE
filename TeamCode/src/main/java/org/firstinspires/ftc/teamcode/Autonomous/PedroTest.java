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
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;


@Autonomous(name="monkeytest")
public class PedroTest extends LinearOpMode {





    private final Pose startPose = new Pose(8, 111, Math.toRadians(0));
    private Path startToCornerBasket,cornerToPickUp1,pickUp1ToCorner,cornerToPickUp2,pickUp2toCorner,cornerToPickUp3, pickUp3ToCorner;
    private Path forwards;
    private Path backwards;
    public static PathChain line1 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(8, 56, Point.CARTESIAN),
                    new Point(32.299, 71.327, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line2 = new PathBuilder()
            .addPath(new BezierCurve(
                    new Point(32.299, 71.327, Point.CARTESIAN),
                    new Point(20.187, 53.383, Point.CARTESIAN),
                    new Point(32.523, 35.664, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line3 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(32.523, 35.664, Point.CARTESIAN),
                    new Point(48.449, 35.215, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line4 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(48.449, 35.215, Point.CARTESIAN),
                    new Point(48.000, 26.243, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line5 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(48.000, 26.243, Point.CARTESIAN),
                    new Point(24.224, 26.243, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line6 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(24.224, 26.243, Point.CARTESIAN),
                    new Point(48.000, 26.467, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line7 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(48.000, 26.467, Point.CARTESIAN),
                    new Point(48.000, 17.944, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line8 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(48.000, 17.944, Point.CARTESIAN),
                    new Point(23.327, 18.168, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line9 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(23.327, 18.168, Point.CARTESIAN),
                    new Point(48.000, 18.168, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line10 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(48.000, 18.168, Point.CARTESIAN),
                    new Point(47.776, 12.421, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line11 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(47.776, 12.421, Point.CARTESIAN),
                    new Point(17.495, 12.748, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

// Repeat the same pattern for line12 to line22

    public static PathChain line12 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(17.495, 8.748, Point.CARTESIAN),
                    new Point(28.262, 63.925, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line13 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(28.262, 63.925, Point.CARTESIAN),
                    new Point(36.336, 63.925, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line14 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(36.336, 63.925, Point.CARTESIAN),
                    new Point(22.430, 35.664, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line15 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(22.430, 35.664, Point.CARTESIAN),
                    new Point(11.664, 35.439, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line16 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(11.664, 35.439, Point.CARTESIAN),
                    new Point(36.785, 63.925, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line17 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(36.785, 63.925, Point.CARTESIAN),
                    new Point(22.430, 36.112, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line18 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(22.430, 36.112, Point.CARTESIAN),
                    new Point(11.888, 34.991, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line19 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(11.888, 34.991, Point.CARTESIAN),
                    new Point(36.785, 63.701, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line20 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(36.785, 63.701, Point.CARTESIAN),
                    new Point(22.206, 35.439, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line21 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(22.206, 35.439, Point.CARTESIAN),
                    new Point(11.664, 35.215, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();

    public static PathChain line22 = new PathBuilder()
            .addPath(new BezierLine(
                    new Point(11.664, 35.215, Point.CARTESIAN),
                    new Point(37.234, 63.701, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .build();




    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));
    public void runOpMode(){
        FinalRobot robot = new FinalRobot(this);

//        Path line2 = new Path(new BezierCurve(
//                new Point(38.804, 65.944, Point.CARTESIAN),
//                new Point(7.178, 18.393, Point.CARTESIAN),
//                new Point(43.065, 39.701, Point.CARTESIAN),
//                new Point(26.019, 24.000, Point.CARTESIAN),
//                new Point(67.738, 57.645, Point.CARTESIAN),
//                new Point(67.065, 18.393, Point.CARTESIAN),
//                new Point(69.981, 39.701, Point.CARTESIAN),
//                new Point(37.682, 27.589, Point.CARTESIAN),
//                new Point(68.187, 23.776, Point.CARTESIAN),
//                new Point(62.131, 19.290, Point.CARTESIAN),
//                new Point(69.757, 25.794, Point.CARTESIAN),
//                new Point(16.150, 21.981, Point.CARTESIAN)
//        ));
//        line2.setLinearHeadingInterpolation(0, 0);
//
//        Path line3 = new Path(new BezierCurve(
//                new Point(16.150, 21.981, Point.CARTESIAN),
//                new Point(53.832, 20.636, Point.CARTESIAN),
//                new Point(65.720, 27.813, Point.CARTESIAN),
//                new Point(52.486, 16.374, Point.CARTESIAN),
//                new Point(58.766, 30.056, Point.CARTESIAN),
//                new Point(54.729, 10.991, Point.CARTESIAN),
//                new Point(68.860, 13.009, Point.CARTESIAN),
//                new Point(42.393, 14.579, Point.CARTESIAN),
//                new Point(62.355, 14.804, Point.CARTESIAN),
//                new Point(15.925, 15.028, Point.CARTESIAN)
//        ));
//        line3.setLinearHeadingInterpolation(0, 0);

//        Path line1 = new Path(new BezierLine(new Point(6.056, 56.523, Point.CARTESIAN), new Point(32.299, 71.327, Point.CARTESIAN)));
//        line1.setLinearHeadingInterpolation(0, 0);
//
//        Path line2 = new Path(new BezierCurve(new Point(32.299, 71.327, Point.CARTESIAN), new Point(20.187, 53.383, Point.CARTESIAN), new Point(32.523, 35.664, Point.CARTESIAN)));
//        line2.setLinearHeadingInterpolation(0, 0);
//
//        Path line3 = new Path(new BezierLine(new Point(32.523, 35.664, Point.CARTESIAN), new Point(48.449, 35.215, Point.CARTESIAN)));
//        line3.setLinearHeadingInterpolation(0, 0);
//
//        Path line4 = new Path(new BezierLine(new Point(48.449, 35.215, Point.CARTESIAN), new Point(48.000, 26.243, Point.CARTESIAN)));
//        line4.setLinearHeadingInterpolation(0, 0);
//
//        Path line5 = new Path(new BezierLine(new Point(48.000, 26.243, Point.CARTESIAN), new Point(24.224, 26.243, Point.CARTESIAN)));
//        line5.setLinearHeadingInterpolation(0, 0);
//
//        Path line6 = new Path(new BezierLine(new Point(24.224, 26.243, Point.CARTESIAN), new Point(48.000, 26.467, Point.CARTESIAN)));
//        line6.setLinearHeadingInterpolation(0, 0);
//
//        Path line7 = new Path(new BezierLine(new Point(48.000, 26.467, Point.CARTESIAN), new Point(48.000, 17.944, Point.CARTESIAN)));
//        line7.setLinearHeadingInterpolation(0, 0);
//
//        Path line8 = new Path(new BezierLine(new Point(48.000, 17.944, Point.CARTESIAN), new Point(23.327, 18.168, Point.CARTESIAN)));
//        line8.setLinearHeadingInterpolation(0, 0);
//
//        Path line9 = new Path(new BezierLine(new Point(23.327, 18.168, Point.CARTESIAN), new Point(48.000, 18.168, Point.CARTESIAN)));
//        line9.setLinearHeadingInterpolation(0, 0);
//
//        Path line10 = new Path(new BezierLine(new Point(48.000, 18.168, Point.CARTESIAN), new Point(47.776, 9.421, Point.CARTESIAN)));
//        line10.setLinearHeadingInterpolation(0, 0);
//
//        Path line11 = new Path(new BezierLine(new Point(47.776, 9.421, Point.CARTESIAN), new Point(17.495, 8.748, Point.CARTESIAN)));
//        line11.setLinearHeadingInterpolation(0, 0);
//
//        Path line12 = new Path(new BezierLine(new Point(17.495, 8.748, Point.CARTESIAN), new Point(28.262, 63.925, Point.CARTESIAN)));
//        line12.setLinearHeadingInterpolation(0, 0);
//
//        Path line13 = new Path(new BezierLine(new Point(28.262, 63.925, Point.CARTESIAN), new Point(36.336, 63.925, Point.CARTESIAN)));
//        line13.setLinearHeadingInterpolation(0, 0);
//
//        Path line14 = new Path(new BezierLine(new Point(36.336, 63.925, Point.CARTESIAN), new Point(22.430, 35.664, Point.CARTESIAN)));
//        line14.setLinearHeadingInterpolation(0, 0);
//
//        Path line15 = new Path(new BezierLine(new Point(22.430, 35.664, Point.CARTESIAN), new Point(11.664, 35.439, Point.CARTESIAN)));
//        line15.setLinearHeadingInterpolation(0, 0);
//
//        Path line16 = new Path(new BezierLine(new Point(11.664, 35.439, Point.CARTESIAN), new Point(36.785, 63.925, Point.CARTESIAN)));
//        line16.setLinearHeadingInterpolation(0, 0);
//
//        Path line17 = new Path(new BezierLine(new Point(36.785, 63.925, Point.CARTESIAN), new Point(22.430, 36.112, Point.CARTESIAN)));
//        line17.setLinearHeadingInterpolation(0, 0);
//
//        Path line18 = new Path(new BezierLine(new Point(22.430, 36.112, Point.CARTESIAN), new Point(11.888, 34.991, Point.CARTESIAN)));
//        line18.setLinearHeadingInterpolation(0, 0);
//
//        Path line19 = new Path(new BezierLine(new Point(11.888, 34.991, Point.CARTESIAN), new Point(36.785, 63.701, Point.CARTESIAN)));
//        line19.setLinearHeadingInterpolation(0, 0);
//
//        Path line20 = new Path(new BezierLine(new Point(36.785, 63.701, Point.CARTESIAN), new Point(22.206, 35.439, Point.CARTESIAN)));
//        line20.setLinearHeadingInterpolation(0, 0);
//
//        Path line21 = new Path(new BezierLine(new Point(22.206, 35.439, Point.CARTESIAN), new Point(11.664, 35.215, Point.CARTESIAN)));
//        line21.setLinearHeadingInterpolation(0, 0);
//
//        Path line22 = new Path(new BezierLine(new Point(11.664, 35.215, Point.CARTESIAN), new Point(37.234, 63.701, Point.CARTESIAN)));
//        line22.setLinearHeadingInterpolation(0, 0);


        while(opModeInInit()&&!isStopRequested()) {



            robot.follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            robot.follower.setStartingPose(new Pose(8, 56, Math.toRadians(0)));
            //robot.follower.setMaxPower(0.8);



            robot.updateLoggerInit("monkeytest", "ConceptPSILogger");





        }
        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        robot.updateLoggerBeginning(),

                        new SequentialAction(

                                new FollowPathAction(robot.follower, line1, true),


                                new FollowPathAction(robot.follower, line2, true),

                                new FollowPathAction(robot.follower, line3, true),

//
                                new FollowPathAction(robot.follower, line4,true),


                                new FollowPathAction(robot.follower, line5,true),


                                new FollowPathAction(robot.follower, line6,true),


                                new FollowPathAction(robot.follower, line7,true),


                                new FollowPathAction(robot.follower, line8,true),


                                new FollowPathAction(robot.follower, line9,true),


                                new FollowPathAction(robot.follower, line10,true),


                                new FollowPathAction(robot.follower, line11,true),


                                new FollowPathAction(robot.follower, line12,true),

                                new FollowPathAction(robot.follower, line13,true),

                                new FollowPathAction(robot.follower, line14,true),

                                new FollowPathAction(robot.follower, line15,true),

                                new FollowPathAction(robot.follower, line16,true),

                                new FollowPathAction(robot.follower, line17,true),

                                new FollowPathAction(robot.follower, line18,true),

                                new FollowPathAction(robot.follower, line19,true),

                                new FollowPathAction(robot.follower, line20,true),

                                new FollowPathAction(robot.follower, line21,true),

                                new FollowPathAction(robot.follower, line22,true),
                                new InstantAction(()->{
                                    robot.shouldRunAction = false;
        })
                        ),

                        robot.updateLogger(),
                        robot.updateLoggerEnd()
                        //robot.sleepthing()
                )
        );
        //Logger.end();


    }

}
//public class GeneratedPaths {
//
//    public static PathBuilder builder = new PathBuilder();
//
//    public static PathChain line1 = builder
//            .addPath(
//                    new BezierLine(
//                            new Point(8.523, 56.523, Point.CARTESIAN),
//                            new Point(38.804, 65.944, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//
//    public static PathChain line2 = builder
//            .addPath(
//                    new BezierCurve(
//                            new Point(38.804, 65.944, Point.CARTESIAN),
//                            new Point(12.561, 31.850, Point.CARTESIAN),
//                            new Point(36.785, 37.009, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//
//    public static PathChain line3 = builder
//            .addPath(
//                    new BezierCurve(
//                            new Point(36.785, 37.009, Point.CARTESIAN),
//                            new Point(53.607, 38.579, Point.CARTESIAN),
//                            new Point(57.869, 27.589, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//
//    public static PathChain line4 = builder
//            .addPath(
//                    new BezierLine(
//                            new Point(57.869, 27.589, Point.CARTESIAN),
//                            new Point(18.617, 28.037, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//
//    public static PathChain line5 = builder
//            .addPath(
//                    new BezierCurve(
//                            new Point(18.617, 28.037, Point.CARTESIAN),
//                            new Point(55.402, 30.056, Point.CARTESIAN),
//                            new Point(58.318, 16.374, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//
//    public static PathChain line6 = builder
//            .addPath(
//                    new BezierLine(
//                            new Point(58.318, 16.374, Point.CARTESIAN),
//                            new Point(18.393, 16.822, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//
//    public static PathChain line7 = builder
//            .addPath(
//                    new BezierCurve(
//                            new Point(18.393, 16.822, Point.CARTESIAN),
//                            new Point(56.972, 18.617, Point.CARTESIAN),
//                            new Point(58.542, 8.523, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//
//    public static PathChain line8 = builder
//            .addPath(
//                    new BezierLine(
//                            new Point(58.542, 8.523, Point.CARTESIAN),
//                            new Point(18.393, 8.523, Point.CARTESIAN)
//                    )
//            )
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//}