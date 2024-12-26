package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.ViperSlideControl;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.Subsystems.HeadingLockedMecanumDrive;

@TeleOp
@Config
public class TeleOpMain extends OpMode {

    // Mecanum drive variables
    private HeadingLockedMecanumDrive mecanumDrive;

    // Viper Slide control variables
    private ViperSlideControl viperSlideControl;
    private DcMotorEx slidemotorLeft;
    private DcMotorEx slidemotorRight;

    // IMU for HeadingLockedMecanumDrive
    private IMU imu;

    // Dashboard for telemetry
    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Initialize Mecanum Drive motors as DcMotorEx
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Initialize Viper Slide motors
        slidemotorLeft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");
        slidemotorRight = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        viperSlideControl = new ViperSlideControl(slidemotorLeft, slidemotorRight);

        // Initialize IMU for HeadingLockedMecanumDrive (matching the standalone example)
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize IMU parameters
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);

        // Initialize Mecanum Drive with both HardwareMap and IMU
        mecanumDrive = new HeadingLockedMecanumDrive(hardwareMap, imu);

        // Set motor behaviors and directions for Mecanum Drive
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior for Mecanum Drive motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial motor configurations for Viper Slide
        slidemotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slidemotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidemotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize dashboard for telemetry
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        // Handle Mecanum Drive with IMU (Pass both Gamepad and IMU to update method)
        mecanumDrive.update(gamepad1, imu);

        // Handle Viper Slide Control
        viperSlideControl.update();

        // Set target positions for Viper Slide
        if (gamepad1.y) {
            viperSlideControl.setTarget(2950);
        }
        if (gamepad1.a) {
            viperSlideControl.setTarget(0);
        }
        if (gamepad1.x) {
            viperSlideControl.setTarget(900);
        }
        if (gamepad1.b) {
            viperSlideControl.setTarget(1400);
        }

        // Telemetry for Mecanum Drive (Heading)
        telemetry.addData("Heading", mecanumDrive.getCurrentHeading());
        telemetry.addData("Target Heading", mecanumDrive.getHeadingSetpoint());
        telemetry.addData("Heading Error", mecanumDrive.getHeadingError());

        // Telemetry for Viper Slide (Position and Target)
        telemetry.addData("Left Slide Position", viperSlideControl.getLeftPosition());
        telemetry.addData("Right Slide Position", viperSlideControl.getRightPosition());
        telemetry.addData("Target", viperSlideControl.getTarget());

        telemetry.update();
    }
}
