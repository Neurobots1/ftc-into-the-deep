import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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
    private Servo AllongeL;
    private Servo pince;
    private Servo bucket;
    private Servo pinceArriere;
    private Servo poignet;

    @Override
    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        boolean intakeToggle = true;
        boolean poignettoggle = true;
        boolean pinceToggle = false;

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

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servo
        SlideR = hardwareMap.get(Servo.class, "SlideR");
        SliderL = hardwareMap.get(Servo.class, "SliderL");
        AlongeR = hardwareMap.get(Servo.class, "AlongeR");
        pince = hardwareMap.get(Servo.class, "pince");
        bucket = hardwareMap.get(Servo.class, "bucket");
        AllongeL = hardwareMap.get(Servo.class, "AllongeL");
        pinceArriere = hardwareMap.get(Servo.class, "pinceArriere");
        poignet = hardwareMap.get(Servo.class, "poignet");
        SlideR.setDirection(Servo.Direction.REVERSE);
        AllongeL.setDirection(Servo.Direction.REVERSE);

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

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.resetYaw();
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();

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

            if (pinceToggle) {
                pince.setPosition(0.3);
                sleep(400);
                SlideR.setPosition(0.35);
                SliderL.setPosition(0.35);
                telemetry.update();
            }else{
                pince.setPosition(0.6);
            }

            if (gamepad1.dpad_right) {
                AlongeR.setPosition(1);
                AllongeL.setPosition(1);
                telemetry.update();
            } else {
                if (gamepad1.dpad_left) {
                    SliderL.setPosition(0.3);
                    SlideR.setPosition(0.3);
                    sleep(200);
                    AlongeR.setPosition(0.35);
                    AllongeL.setPosition(0.3);
                    telemetry.update();
                }
            }

            if (gamepad1.left_trigger>0.1){
                poignet.setPosition(0);
            } else{
                poignet.setPosition(1);
            }


            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                pinceToggle = !pinceToggle;
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                intakeToggle = !intakeToggle;
            }
            if (intakeToggle){
                pinceArriere.setPosition(0);
            } else{
                pinceArriere.setPosition(1);
            }
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            loop();
            {
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

                    if (gamepad1.a) {
                        target = -50;
                    }

                    if (gamepad1.y){
                        target = -2070;
                    }

                    if (gamepad1.b) {
                        target = -900;
                        pinceArriere.setPosition(1);
                    }

                    if (gamepad1.x) {
                        target = -1400;
                    }


                if (gamepad1.dpad_down) {
                    pince.setPosition(0.6);
                    sleep(200);
                    SlideR.setPosition(0.045);
                    SliderL.setPosition(0.045);
                    telemetry.update();
                } else {
                    if (gamepad1.dpad_up) {
                        if (slidemotorright.getCurrentPosition() > -200) {
                            poignet.setPosition(1);
                            sleep(300);
                            SlideR.setPosition(1);
                            SliderL.setPosition(1);
                            sleep(800);
                            pince.setPosition(0.6);
                            sleep(1000);
                            SlideR.setPosition(0.6);
                            SliderL.setPosition(0.6);
                            telemetry.update();
                        } else {
                            SliderL.setPosition(0.6);
                            SlideR.setPosition(0.6);
                        }


                    }
                }
                if (slidemotorright.getCurrentPosition() < -1950) {
                if (gamepad1.right_trigger>0.1) {
                        bucket.setPosition(0.1);
                    } else {
                        bucket.setPosition(1);
                    }
                } else {
                    bucket.setPosition(1);
                }
                //if (gamepad1.start){
                    //slidemotorright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               // }


            }
        }
    }
}


