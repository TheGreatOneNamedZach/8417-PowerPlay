package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** This is an interface for this year's swivel joint on the claw. */
public class swivel {
    // CONSTRUCT
    private static final ElapsedTime delay = new ElapsedTime();
    // DECLARE NULL
    private static Servo swivel;
    private static Boolean isBackwards;
    // DECLARE CUSTOM
    private static double delayInSeconds = 0.75;
    private static final double backwardPositionInTicks = 0.62;
    private static final double forwardPositionInTicks = 0.51;

    // METHODS
    /** Initializes the swivel.
     * @param opMode If you are constructing from an Auto or TeleOp, type in "this" without the quotation marks.
     */
    public void init(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        swivel = hardwareMap.get(Servo.class, "Swivel");
    }

    /** Moves the swivel to the opposite direction of where it currently is.
     * This is useful during testing for visually indicating a change.
     * (e.g. If a color sensor sees red you can call toggleSwivel() to visually indicate that the color sensor sees red.)
     * @apiNote By default, the swivel as a delay of 0.75 seconds between movements. This can be changed.
     */
    public void toggleSwivel() {
        if (delay.time() >= delayInSeconds) { // If the delay has finished...
            if (isBackwards) { // Set the swivel the opposite position
                goToFront();
            } else {
                goToBack();
            }
            delay.reset(); // Reset the 0.75 second delay
        }
    }

    /** Moves the swivel to the opposite direction of where it currently is when a button is pressed.
     * @param isPressed The button being pressed.
     * @apiNote By default, the swivel as a delay of 0.75 seconds between movements. This can be changed.
     */
    public void toggleSwivelWithPress(@NonNull Boolean isPressed) {
        if (isPressed) { // If the button has been pressed...
            toggleSwivel();
        }
    }

    /** Moves the swivel to the front.
     * @apiNote This will bypass the delay feature.
     */
    public void goToFront() {
        swivel.setPosition(forwardPositionInTicks);
        isBackwards = false;
    }

    /** Moves the swivel to the back.
     * @apiNote This will bypass the delay feature.
     */
    public void goToBack() {
        swivel.setPosition(backwardPositionInTicks);
        isBackwards = true;
    }

    /** Changes the minimum time the swivel can move to a new position.
     * @param newDelay The delay in seconds.
     */
    public void setDelay(double newDelay) {
        delayInSeconds = newDelay;
    }
}
