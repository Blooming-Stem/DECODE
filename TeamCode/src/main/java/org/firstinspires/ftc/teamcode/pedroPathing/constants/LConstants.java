package org.firstinspires.ftc.teamcode.pedroPathing.constants;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = .003;
        TwoWheelConstants.strafeTicksToInches = .003;
        TwoWheelConstants.forwardY = 6;
        TwoWheelConstants.strafeX = 7.0;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "omback";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "omfront";
        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }
}




