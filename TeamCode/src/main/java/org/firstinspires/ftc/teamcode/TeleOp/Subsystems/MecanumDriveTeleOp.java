package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp
@Config
public class MecanumDriveTeleOp extends OpMode {

    // Motor declarations
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // IMU for heading control
    IMU imu;

    // Heading lock variables exposed to the FTC Dashboard
    public static double headingKp = -0.005; // Proportional constant for heading correction
    public static double headingSetpoint = 0;  // Desired heading
    private double headingError;         // The difference between current heading and desired heading

    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Initialize motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        // Set motor behaviors (zero power behavior and directions)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU for heading control
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        // Initialize dashboard for real-time adjustments
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        // Get joystick inputs
        double y = -gamepad1.left_stick_y;  // Y-axis is reversed in most controllers
        double x = gamepad1.left_stick_x;   // X-axis for strafing
        double rx = gamepad1.right_stick_x; // Rotation input

        // Reset IMU yaw if options button is pressed
        if (gamepad1.options) {
            imu.resetYaw();
        }

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

        // Telemetry for debugging and graphing on the dashboard
        telemetry.addData("Heading", botHeading); // Current heading
        telemetry.addData("Target Heading", headingSetpoint); // Desired target heading
        telemetry.addData("Heading Error", headingError); // Difference between target and current heading

        // Graphing the current and target heading for visualization
        dashboard.getTelemetry().addData("Current Heading", botHeading);
        dashboard.getTelemetry().addData("Target Heading", headingSetpoint);
        dashboard.getTelemetry().addData("Heading Error", headingError);

        telemetry.update();
    }
}
