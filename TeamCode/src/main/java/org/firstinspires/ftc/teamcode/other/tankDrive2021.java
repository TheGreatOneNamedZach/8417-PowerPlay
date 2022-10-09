package org.firstinspires.ftc.teamcode.other;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2021 Tank Drive", group="Other")
public class tankDrive2021 extends OpMode{
    private DcMotor motor1 = null;
    private ElapsedTime runtime = new ElapsedTime();
    double motorPower;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
    }

    public void loop() {

        telemetry.addData("Runtime", Math.round(getRuntime()));
        motorPower = gamepad1.left_stick_x;
        motor1.setPower(motorPower);
        telemetry.addData("Motor1", motorPower);
    }
}
