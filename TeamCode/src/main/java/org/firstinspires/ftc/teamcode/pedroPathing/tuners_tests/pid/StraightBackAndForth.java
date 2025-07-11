package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystem.FinalRobot;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Straight Back And Forth", group = "PIDF Tuning")
public class StraightBackAndForth extends LinearOpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;


    private Path forwards;
    private Path backwards;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    public void runOpMode() {
        FinalRobot robot = new FinalRobot(this);
        while(opModeInInit()) {


            robot.follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            robot.follower.setStartingPose(new Pose(0, 0, 0));

            forwards = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(DISTANCE, 0, Point.CARTESIAN)));
            forwards.setLinearHeadingInterpolation(0,0);
            backwards = new Path(new BezierLine(new Point(DISTANCE, 0, Point.CARTESIAN), new Point(0, 0, Point.CARTESIAN)));
            backwards.setLinearHeadingInterpolation(0,0);

            robot.follower.followPath(forwards);

            telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                    + " inches forward. The robot will go forward and backward continuously"
                    + " along the path. Make sure you have enough room.");
            telemetryA.update();
            robot.updateLoggerInit();
        }
        waitForStart();


    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    while(opModeIsActive()&&!isStopRequested())

    {
        robot.updateLoggerBeginningFunction();

        robot.follower.update();
        robot.updateLoggerFunction();
        if (!robot.follower.isBusy()) {
            if (forward) {
                forward = false;
                robot.follower.followPath(backwards);
            } else {
                forward = true;
                robot.follower.followPath(forwards);
            }
        }

        telemetryA.addData("going forward", forward);

        robot.follower.telemetryDebug(telemetryA);
        robot.updateLoggerEndFunc();
    }
}
}
