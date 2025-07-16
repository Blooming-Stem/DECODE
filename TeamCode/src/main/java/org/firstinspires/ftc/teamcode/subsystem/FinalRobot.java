package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ServoHubConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;
import org.psilynx.psikit.core.wpi.Translation2d;


@Config

public class FinalRobot {
    LinearOpMode op;
    TelemetryPacket packet;
    FtcDashboard dash;
    public Servo left_shoulder;
    public Servo right_shoulder;
    public Servo right_spine;
    public Servo left_spine;
    public Servo elbow;
    public Servo wrist;
    public CRServo pullup1;
    public CRServo pullup2;

    public Servo shitter;
    public Servo finger;
    public DcMotorEx hang;
    public Follower follower;
    public DcMotorEx turret;
    public DcMotorEx slides1;
    public DcMotorEx slides2;
    public AnalogInput encoder;
    public boolean noextend = false;
    public double speci = 0;
    PIDFController pidf;
    PIDFController p;
    public static double kP = 0.007;
    public static double kI = 0.0001;
    public static double kD = 0.0001;
    public static double kF = 0;

    public static double kkkP = 0.01;
    public static double kkkI = 0.0001;
    public static double kkkD = 0.0002;
    public static double kkkF = 0;
    public static double setPoint = 0;
    public static double turretPoint = 0;
    public Telemetry telemetryThis;
    public double beforeUserStart;
    public double afterUserStart;
    public double beforeUserEnd;
    public boolean shouldRunAction = true;
    LynxModule myRevHub;

    LynxModule myExpansionHub;
    LynxGetADCCommand.Channel servoChannel;
    LynxGetADCCommand servoCommand;
    LynxGetADCResponse servoResponse;

    LynxGetADCCommand.Channel servoChannelEx;
    LynxGetADCCommand servoCommandEx;
    LynxGetADCCommand servoCommandHub;
    LynxGetADCResponse servoResponseEx;

    public FinalRobot(LinearOpMode opMode) {
        op = opMode;
        myRevHub = op.hardwareMap.get(LynxModule.class, "Control Hub");
        myExpansionHub = op.hardwareMap.get(LynxModule.class, "Expansion Hub 2");

        left_shoulder = op.hardwareMap.servo.get("leftshoulder");
        slides1 = op.hardwareMap.get(DcMotorEx.class, "slidesone");
        slides2 = op.hardwareMap.get(DcMotorEx.class,"slidestwo");
        hang = op.hardwareMap.get(DcMotorEx.class,"hang");
        right_shoulder = op.hardwareMap.servo.get("rightshoulder");
        left_spine = op.hardwareMap.servo.get("leftspine");
        right_spine = op.hardwareMap.servo.get("rightspine");
        elbow = op.hardwareMap.servo.get("elbow");
        elbow.scaleRange(0,1);
        wrist = op.hardwareMap.servo.get("wrist");
        finger = op.hardwareMap.servo.get("claw");
        turret = op.hardwareMap.get(DcMotorEx.class,"turret");
        encoder = op.hardwareMap.get(AnalogInput.class, "axon");
        pullup1 = op.hardwareMap.get(CRServo.class, "pullup1");
        pullup2 = op.hardwareMap.get(CRServo.class, "pullup2");
        shitter = op.hardwareMap.servo.get("shitter");
        pullup2.setDirection(CRServo.Direction.REVERSE);
        shitter.setPosition(0);
        pullup1.setPower(0);
        pullup2.setPower(0);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        turret.setDirection(DcMotorSimple.Direction.REVERSE);;
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides1.setDirection(DcMotorSimple.Direction.REVERSE);

        slides1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetHorizontal();
        //elbow.setPosition(0.5);
        resetArm();
        //resethang();
        closeClawInit();
        // resetTurret();


        pidf = new PIDFController(kP,kI,kD,kF);
        p = new PIDFController(kkkP, kkkI, kkkD, kkkF);
        packet = new TelemetryPacket();
        telemetryThis = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());
        dash = FtcDashboard.getInstance();

        telemetryThis.addLine("Ready");
        telemetryThis.update();

    }


    //reset horizontal slides
    public void resetHorizontal(){
        left_spine.setPosition(0.2);
        right_spine.setPosition(0.2);



    }
    //extend horizontal slides
    public void extendHorizontal(){
        right_shoulder.setPosition(0.27);
        left_shoulder.setPosition(0.27);
        elbow.setPosition(0.4);
        left_spine.setPosition(0.6);
        right_spine.setPosition(0.6);

    }
    public void extendHorizontalInspection(){
        right_shoulder.setPosition(0.27);
        left_shoulder.setPosition(0.27);
        elbow.setPosition(0.4);
        left_spine.setPosition(0.45);
        right_spine.setPosition(0.45);

    }
    // Turns turret 180 degrees
    public void spinTurret(){
        turretPoint = 1532;
//        turret.setTargetPosition(1532);
//        turret.setPower(1);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void spinTurretAuto(){
        turretPoint = 1149;
//        turret.setTargetPosition(1532);
//        turret.setPower(1);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void spinTurretAutoother(){
        turretPoint = -1215;
//        turret.setTargetPosition(1532);
//        turret.setPower(1);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void spinTurretThird(){
        turretPoint = 170;
//        turret.setTargetPosition(1532);
//        turret.setPower(1);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void spinTurretThirdOther(){
        turretPoint = -200;
//        turret.setTargetPosition(1532);
//        turret.setPower(1);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    // brings turret back to original position
    public void resetTurret(){
        turretPoint = 0;
//        turret.setTargetPosition(0);
//        turret.setPower(1);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    //extend vertical slides
    public void extendVert() {
        setPoint = 960;
    }
    public void wristMiddle(){
        wrist.setPosition(0.495);
    }
    public void extendPark(){ setPoint = 600;}
    public void extendHang(){ setPoint = 800;}
    public void extendVertHalf(){
        setPoint = 1000/2.75;
    }
    //    public void extendSpine(){
//        left_spine.setPosition(left_spine.getPosition()-0.1);
//        right_spine.setPosition(right_spine.getPosition()+0.1);
//    }
    // Brings slides above top bar and extends elbow
    public void startDepositSpecimen(){
        elbow.setPosition(0.2);
        setPoint = 2.75;
    }
    // brings slides down
    public void finishDepositSpecimen(){
        setPoint = 800/2.75;
    }
    // opens claw
    public void openClaw(){
        finger.setPosition(0.32);
    }
    public void openClawTele(){
        finger.setPosition(0.4);
    }
    // close claw
    public void closeClaw(){
        finger.setPosition(0.7);
    }
    public void closeClawTele(){
        finger.setPosition(0.78);
    }
    public void closeClawInit(){
        finger.setPosition(0.62);
    }
    //at the correct height for intaking sample
    public void intakeBlock(){
        right_shoulder.setPosition(0.31);
        left_shoulder.setPosition(0.31);
        elbow.setPosition(0.02);
        wristMiddle();
    }
    public void autoIntake(){
        right_shoulder.setPosition(0.347);
        left_shoulder.setPosition(0.347);
        elbow.setPosition(0.02);
        wristMiddle();
    }
    // rest position for arm
    // rest position for arm
    public void resetArm(){
        right_shoulder.setPosition(0.015);
        left_shoulder.setPosition(0.015);
        wristMiddle();
        elbow.setPosition(0.5);
        closeClaw();
    }
    public void resetArmAuto(){
        right_shoulder.setPosition(0.01);
        left_shoulder.setPosition(0.01);
        wristMiddle();
        elbow.setPosition(0.5);
        //closeClaw();
    }
    // brings arm forward at an angle
    public void basketDeposit(){
        left_shoulder.setPosition(0.135);
        right_shoulder.setPosition(0.135);
    }
    public void basketDepositTee(){
        left_shoulder.setPosition(0.1);
        right_shoulder.setPosition(0.1);
        elbow.setPosition(0.32);
    }
    public void basketDepositTefdafe(){
        left_shoulder.setPosition(0.1);
        right_shoulder.setPosition(0.1);
        elbow.setPosition(0.28);
    }
    public void basketDepositTefdafeLOL(){
        left_shoulder.setPosition(0.25);
        right_shoulder.setPosition(0.25);
        elbow.setPosition(0.1);
    }
    public void basketDepositMonkey(){
        left_shoulder.setPosition(0.05);
        right_shoulder.setPosition(0.05);
        elbow.setPosition(0.2);
    }
    public void autoTurret(double degrees){
        double ticks = 1532/180*degrees;
        turretPoint = ticks;
    }
    public void underBar(){
        right_shoulder.setPosition(0.3);
        left_shoulder.setPosition(0.3);
        elbow.setPosition(0.37);
    }
    // brings hang above low bar

    // brings slides to height of wall
    public void IntakeWallSpecimen(){
        right_shoulder.setPosition(0.275);
        left_shoulder.setPosition(0.275);
        elbow.setPosition(0.335);
        openClaw();
    }
    public void IntakeWallSpecimenTele(){
        right_shoulder.setPosition(0.29);
        left_shoulder.setPosition(0.29);
        elbow.setPosition(0.325);
        openClaw();
    }
    public void IntakeWallSpecimenAuto(){
        right_shoulder.setPosition(0.29);
        left_shoulder.setPosition(0.29);
        elbow.setPosition(0.325);

    }
    // brings arm down without extending horizontally
    public void RightBefore(){
        right_shoulder.setPosition(0.31);
        left_shoulder.setPosition(0.31);
        elbow.setPosition(0.02);
        wristMiddle();
    }
    // brings slides above low bar and extends arm
    public void teleopLowSpecimen(){
        setPoint = 365;
        speci = 365;
        right_shoulder.setPosition(0);
        left_shoulder.setPosition(0);
        elbow.setPosition(0.02);
        //wrist.setPosition(0.4);
        finger.setPosition(0.45);
    }
    public void autoPark(){
        setPoint = 365;
        right_shoulder.setPosition(0.1);
        left_shoulder.setPosition(0.1);

    }
    public void teleoplastLowSpecimen(){
        setPoint = 265;
        speci = 265;
        right_shoulder.setPosition(0);
        left_shoulder.setPosition(0);
        elbow.setPosition(0.02);
        //wrist.setPosition(0.4);
        finger.setPosition(0.45);
    }
    // brings slides down to low bar height
    public void FinishSpecimen(){



    }
    // opens claw and brings back horizontal slides
    public void ReleaseSpeciment(){
        finger.setPosition(0);
        resetHorizontal();
    }
    // brings slides above high bar and extends arm
    public void StartHighSpecimen() {
        speci = 930;
        setPoint = 930;
        right_shoulder.setPosition(0.32);
        left_shoulder.setPosition(0.32);
        elbow.setPosition(0.8);
        wristMiddle();
        finger.setPosition(0.45);
    }
    public void teleOpSpecimen() {
        speci = 450;
        speci = 450;
        setPoint = 1100/2.75;
        right_shoulder.setPosition(0.06);
        left_shoulder.setPosition(0.06);
        elbow.setPosition(0.06);
        wristMiddle();
    }
    public void lastteleOpSpecimen() {
        speci = 600/2.75;
        setPoint = 500/2.75;
        right_shoulder.setPosition(0.05);
        left_shoulder.setPosition(0.05);
        elbow.setPosition(0.02);
        wristMiddle();
        //finger.setPosition(0.4);
    }
    // brings slides down to height of high bar
    public void EndHighSpecimen() {
        setPoint = 870;

    }
    // Brings slides down to base height
    public void resetVert(){
        setPoint = 0;

    }
    // PIDF Slides Controller
    // copied from ftclib
    public void updateSlides(){
        pidf.setPIDF(kP, kI,kD,kF);
        double output = pidf.calculate(
                slides1.getCurrentPosition(), setPoint
        );
        op.telemetry.addData("Velocity", output);
        //op.telemetry.update();
        slides1.setPower(output);
        slides2.setPower(output);
    }
    public void updateTurret(){
        p.setPIDF(kkkP, kkkI,kkkD,kkkF);
        double toutput = p.calculate(
                turret.getCurrentPosition(), turretPoint
        );
        turret.setPower(toutput);
    }
    public void autoSpecimen(){
        right_shoulder.setPosition(0.35);
        left_shoulder.setPosition(0.35);
        elbow.setPosition(0.7);
//        right_spine.setPosition(0.09);
//        left_spine.setPosition(0.09);
        setPoint = 350;
    }
    public void autoSpeci(){
        right_shoulder.setPosition(0.35);
        left_shoulder.setPosition(0.35);
        elbow.setPosition(0.6);
//        right_spine.setPosition(0.09);
//        left_spine.setPosition(0.09);
        setPoint = 400;
    }
    public void extendhorizspecimen(){
        left_spine.setPosition(0.6);
        right_spine.setPosition(0.6);
    }


    public double ticks_to_inches(double ticks){
        return -57.7935*ticks*ticks+84.3296*ticks-18.0272;
    }
    public double inches_to_ticks(double inches){
        if(1-0.0785*inches<0){
            return 0.2;
        }
        return 0.7296-0.4694*Math.sqrt(1-0.0785*inches);


    }
    public double analog_to_ticks (double analog){
        return 0.7335*analog - 0.7323;
    }
    public void updateTelemetry(){
        dash.sendTelemetryPacket(packet);
    }


    public class UpdateSlides implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateSlides();
            return true;
        }
    }
    public Action updateSlidesAction(){
        return new UpdateSlides();
    }
    public class UpdateFollower implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(follower.isBusy()) {
                follower.update();
                follower.telemetryDebug(telemetryThis);
                follower.updatePose();
                follower.drawOnDashBoard();
            }
            return true;
        }
    }
    public Action updateFolowerAction(){
        return new UpdateFollower();
    }



    public class UpdateTurret implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateTurret();
            return true;
        }
    }
    public Action updateTurretAction(){
        return new UpdateTurret();
    }
    public void updateLoggerBeginningFunction(){
        beforeUserStart = Logger.getTimestamp();
        Logger.periodicBeforeUser();
        beforeUserEnd = Logger.getTimestamp();
    }


    public class UpdateLoggerBeginning implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateLoggerFunction();
            return shouldRunAction;
        }
    }
    public Action updateLoggerBeginning(){
        return new UpdateLoggerBeginning();
    }

    public void updateLoggerEndFunc(){
        afterUserStart = Logger.getTimestamp();
        Logger.periodicAfterUser(
                afterUserStart - beforeUserEnd,
                beforeUserEnd - beforeUserStart
        );
    }

    public class UpdateLoggerEnd implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateLoggerEndFunc();
            return shouldRunAction;
        }
    }
    public Action updateLoggerEnd(){
        return new UpdateLoggerEnd();
    }
    public void updateLoggerFunction(){
        Translation2d poseXY = new Translation2d((follower.getPose().getY()-72) * 0.0254, -(follower.getPose().getX()-72) * 0.0254);
        Rotation2d poseHeading = new Rotation2d(follower.getPose().getHeading()-Math.toRadians(90));
        Logger.recordOutput("2024-2025 Field", new Pose2d(poseXY, poseHeading));
        if(follower.isBusy()){
            Translation2d closestposeXY = new Translation2d((follower.getClosestPose().getY()-72) * 0.0254, -(follower.getClosestPose().getX()-72) * 0.0254);
            Rotation2d closestposeHeading = new Rotation2d(follower.getClosestPose().getHeading()-Math.toRadians(90));
            Logger.recordOutput("2024-2025 Field/Ghost", new Pose2d(closestposeXY, closestposeHeading));

        }
        telemetryThis.addData("Path", follower.getCurrentPathNumber());
        telemetryThis.update();
    }

    public class UpdateLogger implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            updateLoggerFunction();
            return shouldRunAction;
        }
    }
    public Action updateLogger(){
        return new UpdateLogger();
    }
    public class Sleep implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return shouldRunAction;
        }
    }
    public Action sleepthing(){
        return new Sleep();
    }
    public void updateLoggerInit(String opModeName, String value){

        RLOGServer server = new RLOGServer();
        RLOGWriter writer = new RLOGWriter("/sdcard/FIRST/userLogs", "finalascopetesting");
        Logger.addDataReceiver(server);
        Logger.addDataReceiver(writer);
        Logger.recordMetadata(opModeName, value);
        Logger.start();
        Logger.periodicAfterUser(0, 0);
        // Start logging! No more data receivers, replay sources, or metadata values may be added.
    }


    public double getServoBusCurrent()
    {
        servoChannel = LynxGetADCCommand.Channel.SERVO_CURRENT;
        servoCommand = new LynxGetADCCommand(myRevHub, servoChannel, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            servoResponse = servoCommand.sendReceive();
            return servoResponse.getValue() / 1000.0;    // return value in Amps
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
        }
        return 999;
    }
    public double getServoBusCurrentExpansion()
    {
        servoChannelEx = LynxGetADCCommand.Channel.SERVO_CURRENT;
        servoCommandEx = new LynxGetADCCommand(myExpansionHub, servoChannelEx, LynxGetADCCommand.Mode.ENGINEERING);
        try
        {
            servoResponseEx = servoCommandEx.sendReceive();
            return servoResponseEx.getValue() / 1000.0;    // return value in Amps
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
        }
        return 999;
    }


}