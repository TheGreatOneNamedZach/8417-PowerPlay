package org.firstinspires.ftc.teamcode.initialize;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.action.claw;
import org.firstinspires.ftc.teamcode.action.distanceSensor;
import org.firstinspires.ftc.teamcode.action.linearSlide;
import org.firstinspires.ftc.teamcode.action.mecanumDrive;
import org.firstinspires.ftc.teamcode.action.swivel;

/** This OpMode is a TeleOp.
 * It uses mecanum wheels to move around.
 * In addition, it can pick up cones with a claw and lift them with a linear slide.
 * The robot is driver controlled. It requires two gamepads. */
@TeleOp(name = "Mechanum Drive", group = "Main")
public class teleOp extends OpMode {
    // CONSTRUCT
    org.firstinspires.ftc.teamcode.action.mecanumDrive mecanumDrive = new mecanumDrive();
    org.firstinspires.ftc.teamcode.action.claw claw = new claw();
    org.firstinspires.ftc.teamcode.action.swivel swivel = new swivel();
    org.firstinspires.ftc.teamcode.action.linearSlide linearSlide = new linearSlide();
    org.firstinspires.ftc.teamcode.action.distanceSensor distanceSensor = new distanceSensor();
    // DECLARE NULL
    // DECLARE CUSTOM
    // METHODS
    /** Initializes the teleop. */
    @Override
    public void init() {
        mecanumDrive.init(this);
        claw.init(this);
        swivel.init(this);
        linearSlide.init(this);
        distanceSensor.init(this);
    }

    /** Runs one time when the teleop starts. */
    public void start() {
        swivel.goToFront();
        claw.close();
        mecanumDrive.runWithoutEncoder();
        distanceSensor.distanceSensorTurnToDegree(10);
        distanceSensor.distanceSensorTurnToDegree(0);
    }

    /** Loops until the stop button is pressed. */
    @Override
    public void loop() {
        mecanumDrive.setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        mecanumDrive.slowMode(gamepad1.right_bumper);

        linearSlide.setPower(-gamepad2.left_stick_y);
        linearSlide.goToJunctionsOnPress(gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_up);
        linearSlide.telemetryOutput();

        claw.toggleClawWithPress(gamepad2.right_bumper);
        swivel.toggleSwivelWithPress(gamepad1.left_bumper);
    }
}