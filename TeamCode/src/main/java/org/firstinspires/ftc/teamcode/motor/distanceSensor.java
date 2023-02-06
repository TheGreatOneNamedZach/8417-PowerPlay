package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.vision.mb1043sensor;

import java.util.Objects;

public class distanceSensor {
    private mb1043sensor mb1043sensor;
    private Servo distanceServo;
    private final double servoSpinAreaInDegrees = 180.00;
    private final double servoMaxRotation = 300.00;
    private static String actionInProgress = "none";
    private static int degreesToTravelPerScan = 0;
    private static long lastSysTime = -1;
    private static double roundingNumber = 10000.0;

    // This is the upper boundary.
    // This only matters if you have custom limits set.
    private final double maxBoundaryInTicks = servoSpinAreaInDegrees / servoMaxRotation;

    /** Initializes the distance sensor.
     * @param opMode If you are constructing from an Auto or TeleOp, type in "this" without the quotation marks.
     */
    public void init(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        mb1043sensor = hardwareMap.get(mb1043sensor.class, "Distance Sensor");
        distanceServo = hardwareMap.get(Servo.class, "Distance Servo");
    }

    /** Starts running the distance sensor until returnToStart() is called.
     * @param degreesToTravelPerScan Indicates how many degrees should be traveled after each scan. This should be negative to scan in the reverse direction.
     * @apiNote A continuous scan will be stopped if another type of scan is requested. However, that request will be carried out.
     */
    public void startScanning(int degreesToTravelPerScan) {
        actionInProgress = "contScan"; // Indicates to scan() that the user wants to start a continuous scan
        distanceSensor.degreesToTravelPerScan = degreesToTravelPerScan;
    }

    /** Stops rotating the distance sensor. When started back up, it will scan from its current position instead of resetting to its initial starting position.
     * @apiNote This is useful for when you can not stop calling scan() (or something else is calling scan()) but you need the sensor to stop rotating temporarily.
     */
    public void freeze() {
        actionInProgress = "none";
    }

    /** Scans its area then scans back to its starting position.
     * @param degreesToTravelPerScan Indicates how many degrees should be traveled after each scan. This should be negative to scan in the reverse direction.
     */
    public void sweepOnce(int degreesToTravelPerScan) {
        actionInProgress = "sweepOnce"; // Indicates to scan() that the user wants to start a single sweep
        distanceSensor.degreesToTravelPerScan = degreesToTravelPerScan;
    }

    /** Scans its area but does not go back to its starting position.
     * @param degreesToTravelPerScan Indicates how many degrees should be traveled after each scan.
     * @apiNote This will never travel towards the minimum boundary.
     */
    public void scanToMax(int degreesToTravelPerScan) {
        actionInProgress = "scanOnce"; // Indicates to scan() that the user wants to scan to the higher boundary
        distanceSensor.degreesToTravelPerScan = Math.abs(degreesToTravelPerScan);
    }

    /** Scans its area and ends in the starting position.
     * @param degreesToTravelPerScan Indicates how many degrees should be traveled after each scan.
     * @apiNote This will never travel towards the maximum boundary
     */
    public void scanToMin(int degreesToTravelPerScan) {
        actionInProgress = "scanOnce"; // Indicated to scan() that the user wants to scan to the lower boundary
        distanceSensor.degreesToTravelPerScan = Math.abs(degreesToTravelPerScan) * -1;
    }

    /** Returns the distance sensor to its starting position ASAP
     * @apiNote This does not return a distance.
     */
    public void returnToStart() {
        if(distanceServo.getPosition() == 0) {actionInProgress = "none";return;} // Ends early if already at the starting position
        distanceServo.setPosition(0); // Returns to start without scanning
    }

    /** Returns the distance without scanning. (This is scan() but does not rotate)
     * @apiNote Want to temperately stop rotating the sensor? Use freeze(). This is the same as freeze() if this method is called only when scan() is not.
     * @return Returns as a double[]. Returns the distance and current rotation (in degrees).
     */
    public double[] getDistanceWithoutScan() {
        return new double[] {mb1043sensor.getDistance(), (Math.round(distanceServo.getPosition() * roundingNumber) / roundingNumber)};
    }

    private void rotate(int degreesToTravelPerScan) {
        double currentPosition = distanceServo.getPosition();
        double nextPosition = currentPosition + degreesToServoPosition(degreesToTravelPerScan);
        distanceServo.setPosition(nextPosition);
    }

    @NonNull
    private Boolean delayNotFinished() {
        // If the time since the last ping is greater than the minimum delay then return true else return false
        if ((System.currentTimeMillis() - lastSysTime) > mb1043sensor.minDelayInMilliseconds) {
            lastSysTime = System.currentTimeMillis();
            return false;
        }
        return true;
    }

    private double degreesToServoPosition(int degrees) {
        // This assumes 0 degrees is the position 0.00
        try {
            // There is actually a lot of things that could go wrong
            // because of this, if something DOES go wrong, you will easily know it happened here.
            return (double)degrees / servoSpinAreaInDegrees;
        } catch (Exception e) {
            throw new RuntimeException("'degreesToServoPosition()' in the '" + getClass().getSimpleName() + "' class threw the following error:\n" + e);
        }
    }

    private double servoPositionToDegrees(double servoPosition) {
        return servoPosition * servoSpinAreaInDegrees;
    }

    public String getActionInProgress() {
        return actionInProgress;
    }

    /** Scans the area following the directions previously specified by the user.
     * If no directions were given previously, it will just scan in place.
     * If no position has been previously set, it will start assume it's position is 0.
     * @apiNote scan() was made to be called continuously in a loop() or while() statement.
     * @return Returns as a double[]. Returns the distance and current rotation (in degrees).
     */
    public double[] scan() {
        double distance = mb1043sensor.getDistance();
        double currentRotation = Math.round(servoPositionToDegrees(distanceServo.getPosition()) * roundingNumber) / roundingNumber;
        double[] disAndRot = {distance, currentRotation};
        if(delayNotFinished() || Objects.equals(actionInProgress, "none")) {
            return disAndRot;
        }
        /* Anything below this line will not run until:
        1.) The distance sensor receives the light/sound it sent out
        2.) The user actually tells the distance sensor how it should run
         */




        /* Stops rotating if rotating the sensor would cause it to leave the minimum or maximum boundaries
                In addition, if it is currently executing "sweepOnce", it will finish the second half
                In addition, if it is currently executing a "scanOnce", it will stop
           Else, it will reverse direction */
        if(
                (((distanceServo.getPosition() + degreesToServoPosition(degreesToTravelPerScan)) < 0) // If rotating would exit the lower boundary...
                        && //...and...
                        (degreesToTravelPerScan <= 0)) //...it is traveling towards the lower boundary
                || // OR
                (((distanceServo.getPosition() + degreesToServoPosition(degreesToTravelPerScan)) > maxBoundaryInTicks) // If rotating would exit the higher boundary...
                        && //...and...
                        (degreesToTravelPerScan >= 0)) //...it is traveling towards the higher boundary
        ) { // TL;DR, when it hits reaches a boundary, the following happens:



            if(
                    Objects.equals(actionInProgress, "sweepOnce") // If we are doing a single sweep...
                            && //...and...
                    (distanceServo.getPosition() + degreesToServoPosition(degreesToTravelPerScan)) > maxBoundaryInTicks //...the distance sensor is at the higher boundary (or can not rotate further)
            ) {
                /* Since we know half of a sweep has been completed,
                   and the sensor is at the higher boundary,
                   we can just call scanToMin() to complete the sweep */
                scanToMin(degreesToTravelPerScan);
                return disAndRot;
            }



            else if (
                    Objects.equals(actionInProgress, "sweepOnce") //But if we are doing a single sweep...
                            && //...and...
                    ((distanceServo.getPosition() + degreesToServoPosition(degreesToTravelPerScan)) < 0)) //...the distance sensor is at the lower boundary (or can not rotate further)
            {
                /* Since we know half of a sweep has been completed,
                   and the sensor is at the lower boundary,
                   we can just call scanToMax() to complete the sweep */
                scanToMax(degreesToTravelPerScan);
                return disAndRot;
            }



            if(
                    (Objects.equals(actionInProgress, "scanOnce")) // If we are only scanning once...
                            && //...and either:
                            (
                                    ((distanceServo.getPosition() + degreesToServoPosition(degreesToTravelPerScan)) > maxBoundaryInTicks) // A.) the distance sensor is at the higher boundary
                                    || //OR
                                    ((distanceServo.getPosition() + degreesToServoPosition(degreesToTravelPerScan)) < 0) // B.) The distance sensor is at the lower boundary
                            )
            ) {// then stop rotating and just scan in place
                actionInProgress = "none";
                return disAndRot;
            }

            // If none of those if statements happen it must be continuous rotation.
            degreesToTravelPerScan = degreesToTravelPerScan * -1; // So reverse directions to go back to the opposite boundary
            rotate(degreesToTravelPerScan);
            distance = mb1043sensor.getDistance();
            currentRotation = Math.round(servoPositionToDegrees(distanceServo.getPosition()) * roundingNumber) / roundingNumber;
            return new double[]{distance, currentRotation};

        }
        
        // If the sensor is not at a boundary...
        rotate(degreesToTravelPerScan);
        distance = mb1043sensor.getDistance();
        currentRotation = Math.round(servoPositionToDegrees(distanceServo.getPosition()) * roundingNumber) / roundingNumber;
        return new double[]{distance, currentRotation};
    }
}
