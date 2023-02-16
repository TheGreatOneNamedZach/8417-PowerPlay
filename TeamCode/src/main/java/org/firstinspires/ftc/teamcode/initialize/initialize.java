package org.firstinspires.ftc.teamcode.initialize;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.action.claw;
import org.firstinspires.ftc.teamcode.action.colorSensor;
import org.firstinspires.ftc.teamcode.action.distanceSensor;
import org.firstinspires.ftc.teamcode.action.linearSlide;
import org.firstinspires.ftc.teamcode.action.mecanumDrive;
import org.firstinspires.ftc.teamcode.action.swivel;
import org.firstinspires.ftc.teamcode.action.webcam;

/** This class makes initializing parts of the robots slightly easier. It uses method chaining. */
public class initialize {
    // CONSTRUCT
    // DECLARE NULL
    private static OpMode opMode;
    private static Telemetry telemetry;
    // DECLARE CUSTOM
    // METHODS
    /** Initializes the initializer. */
    public initialize init(@NonNull OpMode opMode) {
        initialize.opMode = opMode;
        telemetry = opMode.telemetry;
        return null;
    }

    /** Initializes the mecanum drive. */
    public initialize mecanumDrive() {
        org.firstinspires.ftc.teamcode.action.mecanumDrive mecanumDrive = new mecanumDrive();
        mecanumDrive.init(opMode);
        return this;
    }

    /** Initializes the claw. */
    public initialize claw() {
        org.firstinspires.ftc.teamcode.action.claw claw = new claw();
        claw.init(opMode);
        return this;
    }

    /** Initializes the swivel. */
    public initialize swivel() {
        org.firstinspires.ftc.teamcode.action.swivel swivel = new swivel();
        swivel.init(opMode);
        return this;
    }

    /** Initializes the linear slide. */
    public initialize linearSlide() {
        org.firstinspires.ftc.teamcode.action.linearSlide linearSlide = new linearSlide();
        linearSlide.init(opMode);
        return this;
    }

    /** Initializes the distance sensor. */
    public initialize distanceSensor() {
        org.firstinspires.ftc.teamcode.action.distanceSensor distanceSensor = new distanceSensor();
        distanceSensor.init(opMode);
        return this;
    }

    /** Initializes the color sensor. */
    public initialize colorSensor() {
        org.firstinspires.ftc.teamcode.action.colorSensor colorSensor = new colorSensor();
        colorSensor.init(opMode);
        return this;
    }

    /** Initializes the webcam.
     * @param tensorFlow If you want to initialize TensorFlow, set this to true and don't forget to use teamColor.
     * @param vuforia If you want to initialize Vuforia, set this to true.
     * @param teamColor If you DO NOT want to pass in a team color, set this to null.
     * @apiNote "teamColor" specifies which model to use for object detection.
     * "Default" uses the default model.
     * "Blue" Uses the blue cone bias model.
     * "Red", "null", or any other value uses the model trained on just the image with no cone.
     * If no team color is specified when TensorFlow is initialized, the "Red" model is used.
     */
    public initialize webcam(Boolean tensorFlow, Boolean vuforia, String teamColor) {
        org.firstinspires.ftc.teamcode.action.webcam webcam = new webcam();
        webcam.init(opMode, tensorFlow, vuforia, teamColor);
        return this;
    }

    /** Outputs when it is completed initializing.
     * @apiNote If called, this should always be the last method called.
     */
    public void telemetryOutputWhenReady() {
        telemetry.addData("Status", "Ready!");
    }
}
