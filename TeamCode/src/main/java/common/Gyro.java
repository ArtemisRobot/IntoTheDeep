package common;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Gyro {

    public enum GyroType {IMU, NAVX}

    RevHubOrientationOnRobot.LogoFacingDirection defaultLogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  defaultUsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    GyroType gyroType;
    HardwareDevice device;
    IMU imu = null;
    AHRS navx = null;

    public Gyro (HardwareMap hardwareMap, String deviceName) {

        device = hardwareMap.get(deviceName);

        if (device instanceof IMU) {
            initIMU(defaultLogoDirection, defaultUsbDirection);

        } else if (device instanceof NavxMicroNavigationSensor) {
            initNavx();

        } else {
            throw new IllegalArgumentException(String.format("Unable to find a gyro with name \"%s\"", deviceName));
        }

        Logger.message("Created gyro %s (%s)", device.getDeviceName(), device.getClass().getSimpleName());
    }

    public Gyro (
            HardwareMap hardwareMap,
            String deviceName,
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection) {

        device = hardwareMap.get(deviceName);

        if (device instanceof IMU) {
            initIMU(logoDirection, usbDirection);

        } else if (device instanceof NavxMicroNavigationSensor) {
            initNavx();

        } else {
            throw new IllegalArgumentException(String.format("Unable to find a gyro with name \"%s\"", deviceName));
        }

        Logger.message("Created gyro %s (%s)", device.getDeviceName(), device.getClass().getSimpleName());
    }

    private void initIMU (
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection) {

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        gyroType = GyroType.IMU;
        imu = (IMU) device;
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    private void initNavx () {

        gyroType = GyroType.NAVX;
        navx = AHRS.getInstance((NavxMicroNavigationSensor) device, AHRS.DeviceDataType.kProcessedData);

        while ( navx.isCalibrating() ) {
            Thread.yield();
        }
        navx.zeroYaw();
    }

    public void resetYaw() {

        if (gyroType == GyroType.IMU)
            imu.resetYaw();
        else if (gyroType == GyroType.NAVX) {
            navx.zeroYaw();
        }
    }

    public double getYaw() {

        if (gyroType == GyroType.IMU) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            return orientation.getYaw(AngleUnit.DEGREES);

        } else if (gyroType == GyroType.NAVX) {
            return navx.getYaw();
        }

        return 0;
    }
}
