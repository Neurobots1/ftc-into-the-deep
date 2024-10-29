import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class ViperslidePIDF extends OpMode {
    private PIDController controller;
}

        public static double p = 0, i = 0, d = 0;
public static double f = 0;

public static int target = 0 ;

private final double tick_in_degree = 384.5 / 180.0;

private DcMotorEx slidemotorright;
private DcMotorEx slidemotorleft;

@Override
public void init() {
    controller = new PIDCOntroller(p, i , d);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    slidemotorright
}

@Override
public void loop() {
    controller.setPID(p, i, d);
    int slidePOS = slidemotorleft.getCurrentPosition();
    double pid = controller.calculate(armPos, target);
    double ff = Math.cos(Math.toRadians(target / tick_in_degree)) * f;

    double power = pid * ff;

    slidemotorleft.setPower(power);
    telemetry.addData("pos" , slidePOS);
    telemetry.addData("target" , target);
    telemetry.update();
}