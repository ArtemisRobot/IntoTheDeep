package org.firstinspires.ftc.teamcode.pedroPathing.tuning;


public class Constants {
    // Tuning for test robot

    public static double xMovement = 74.7;                                  // ToDo Art: For Forward Velocity Tuner
    public static double yMovement = 58.00;                                 // ToDo Art: For Strafe Velocity Tuner

    public static double translationalPIDFCoefficientP = 0.1;
    public static double translationalPIDFCoefficientD = 0;
    public static double translationalPIDFFeedForward = 0.015;

    public static double headingPIDFCoefficientP = 0.4;
    public static double headingPIDFCoefficientD = 0;
    public static double headingPIDFFeedForward = 0.01;

    public static double drivePIDFCoefficientP = 0.025;
    public static double drivePIDFCoefficientD = 0.00001;
    public static double drivePIDFFeedForward = 0.01;

    public static double mass = 10.65942;                                   // ToDo Art: Weight of the robot in kilos
    public static double centripetalScaling = 0.0005;                       // ToDo Art: For CurvedBackAndForth
    public static double forwardZeroPowerAcceleration = -24.0;              // ToDo Art: For Forward Zero Power Acceleration Tuner
    public static double lateralZeroPowerAcceleration = -16.0;              // ToDo Art: For Lateral Zero Power Acceleration Tuner
    public static double zeroPowerAccelerationMultiplier = 5;               // ToDo Art:

}
