package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HeadingLockedMecanumDrive {
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private IMU imu;

    // Heading lock variables
    public static double headingKp = -0.005;  // Proportional constant for heading correction
    public static double headingSetpoint = 0;  // Desired heading
    private double headingError;         // The difference between current heading and desired heading

    public HeadingLockedMecanumDrive(HardwareMap hardwareMap, IMU imu) {
        this.imu = imu;

        // Initialize motors from hardware map
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor behaviors (zero power behavior and directions)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
    }

    // Method to drive the robot
    public void drive(double x, double y, double rx) {
        // Get current heading (yaw) from IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // When rotating manually, update the heading setpoint
        if (Math.abs(rx) > 0.05) {
            headingSetpoint = botHeading;  // Update setpoint when joystick is moving
        }

        // Calculate heading error and apply proportional control
        headingError = headingSetpoint - botHeading;
        double headingCorrection = headingKp * headingError;  // Proportional correction

        // Apply heading correction only when there is no manual rotation input (rx is near zero)
        if (Math.abs(rx) < 0.05) {
            rx = headingCorrection;  // Apply correction to rotation if user isn't manually rotating
        }

        // Calculate rotated movement direction (field-centric driving)
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing (adjust for motor imperfections)

        // Denominator ensures motor power is normalized
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        // Calculate the motor powers for mecanum drive (field-centric)
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    // Getter method for heading setpoint
    public double getHeadingSetpoint() {
        return headingSetpoint;
    }

    // Getter method for current heading (Yaw)
    public double getCurrentHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // Getter method for heading error
    public double getHeadingError() {
        return headingSetpoint - getCurrentHeading();
    }

    // Method to update Mecanum Drive (called in TeleOpMain)
    public void update(Gamepad gamepad, IMU imu) {
        double x = -gamepad.left_stick_x;  // Left joystick X axis
        double y = gamepad.left_stick_y;   // Left joystick Y axis
        double rx = -gamepad.right_stick_x; // Right joystick X axis for rotation

        // Call the drive method
        drive(x, y, rx);
    }
}
