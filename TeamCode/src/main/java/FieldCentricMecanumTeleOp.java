import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ViperslidePIDF;


@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {


    private PIDController controller;


    public static double p = 0.005, i = 0, d = 0.0001;
    public static double f = -0.1;

    public static int target = 0;

    private final double ticks_in_degree = 384.5 / 180.0;

    private DcMotorEx slidemotorright;
    private DcMotorEx slidemotorleft;


    private Servo SlideR;
    private Servo SliderL;
    private Servo AlongeR;
    private Servo pince;
    private Servo bucket;


    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidemotorright = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        slidemotorleft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");


        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");

        //servo
        SlideR = hardwareMap.get(Servo.class, "SlideR");
        SliderL = hardwareMap.get(Servo.class, "SliderL");
        AlongeR = hardwareMap.get(Servo.class, "AlongeR");
        pince = hardwareMap.get(Servo.class, "pince");
        bucket = hardwareMap.get(Servo.class,"bucket");
        SlideR.setDirection(Servo.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        // J'ai changer LeftFront pour rightback a voir si ca marche

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
            //ceci est un test;

            if (gamepad1.dpad_up) {
                SlideR.setPosition(1);
                SliderL.setPosition(1);
                telemetry.update();
            } else {
                if (gamepad1.dpad_down) {
                    SlideR.setPosition(0.05);
                    SliderL.setPosition(0.05);
                    telemetry.update();
                }


            }
            if (gamepad2.a) {
                pince.setPosition(0.3);
                telemetry.update();
            } else {
                if (gamepad2.b) {
                    pince.setPosition(0.6);
                    telemetry.update();
                }
            }
            if (gamepad1.dpad_left) {
                AlongeR.setPosition(0.6);
                telemetry.update();
            } else {
                if (gamepad1.dpad_right) {
                    AlongeR.setPosition(0.4);
                    telemetry.update();
                }
            }
            if (gamepad1.left_bumper) {
                bucket.setPosition(1);
                telemetry.update();
            } else {
                if (gamepad1.right_bumper) {
                    bucket.setPosition(0.1);
                    telemetry.update();
                }
            }
            loop(); {
            if (gamepad1.y) {
                target = -2000;
                controller.setPID(p, i, d);
                int slidePos = slidemotorright.getCurrentPosition();
                double pid = controller.calculate(slidePos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                double power = pid + ff;

                slidemotorright.setPower(power);
                slidemotorleft.setPower(-power);
                telemetry.addData("pos", slidePos);
                telemetry.addData("target", target);
                telemetry.update();
            } else {
                controller.setPID(p, i, d);
                int slidePos = slidemotorright.getCurrentPosition();
                double pid = controller.calculate(slidePos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                double power = pid + ff;

                slidemotorright.setPower(power);
                slidemotorleft.setPower(-power);
                telemetry.addData("pos", slidePos);
                telemetry.addData("target", target);
                telemetry.update();

                if (gamepad1.a){
                    target = -50;
                }

                if (gamepad1.x){
                    target = -900;
                }

                if (gamepad1.b){
                    target = -1400;
                }
                }
            }



        }
        }
    }


