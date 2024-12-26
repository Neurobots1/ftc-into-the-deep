package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp
public class LimitSwitchTest extends OpMode {

    private TouchSensor limitSwitch;

    @Override
    public void init() {
        // Initialize the limit switch from the hardware map
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        // Telemetry message to indicate initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Check the state of the limit switch
        boolean isPressed = limitSwitch.isPressed();

        // Telemetry to display the state of the limit switch
        telemetry.addData("Limit Switch Pressed", isPressed ? "YES" : "NO");
        telemetry.update();
    }
}
