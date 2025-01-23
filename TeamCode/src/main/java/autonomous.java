import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous
public class autonomous extends LinearOpMode {
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
    public void runOpMode(){

        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");

        slidemotorright = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        slidemotorleft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");

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

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);



















    }
}
