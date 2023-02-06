package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.vision.mb1043sensor;

import java.util.Objects;

public class distanceSensor {
    private Telemetry telemetry;
    private mb1043sensor mb1043sensor;
    private Servo distanceServo;
    private final double servoSpinAreaInDegrees = 180.00;
    private static String actionInProgress = "none";
    private static int degreesToTravelPerScan = 0;

    /** Initializes the distance sensor.
     * @param opMode If you are constructing from an Auto or TeleOp, type in "this" without the quotation marks.
     */
    public void init(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        mb1043sensor = hardwareMap.get(mb1043sensor.class, "Distance Sensor");
        distanceServo = hardwareMap.get(Servo.class, "Distance Servo");
    }

    /** Starts running the distance sensor until returnToStart() is called.
     * @param degreesToTravelPerScan Indicates how many degrees should be traveled after each scan.
     * @apiNote A continuous scan will be stopped if another type of scan is requested. However, that request will be carried out.
     */
    public void startScanning(int degreesToTravelPerScan) {
        actionInProgress = "contScan"; // Indicates to scan() that the user wants to start a continuous scan
        distanceSensor.degreesToTravelPerScan = degreesToTravelPerScan;
    }

    /** Scans its area then scans back to its starting position.
     * @param degreesToTravelPerScan Indicates how many degrees should be traveled after each scan.
     */
    public void sweepOnce(int degreesToTravelPerScan) {
        actionInProgress = "sweepOnce"; // Indicates to scan() that the user wants to start a single sweep
        distanceSensor.degreesToTravelPerScan = degreesToTravelPerScan;
    }

    /** Scans its area but does not go back to its starting position.
     * @param degreesToTravelPerScan Indicates how many degrees should be traveled after each scan.
     */
    public void scanToMax(int degreesToTravelPerScan) {
        actionInProgress = "scanOnce"; // Indicates to scan() that the user wants to scan to the higher boundary
        distanceSensor.degreesToTravelPerScan = degreesToTravelPerScan;
    }

    /** Scans its area and ends in the starting position.
     * @param degreesToTravelPerScan Indicates how many degrees should be traveled after each scan.
     * @apiNote degreesToTravelPerScan should be positive!
     */
    public void scanToMin(int degreesToTravelPerScan) {
        degreesToTravelPerScan = degreesToTravelPerScan * -1;
        actionInProgress = "scanOnce"; // Indicated to scan() that the user wants to scan to the lower boundary
        distanceSensor.degreesToTravelPerScan = degreesToTravelPerScan;
    }


    /** Returns the distance sensor to its starting position without returning a distance. */
    public void returnToStart() {
        if(distanceServo.getPosition() == 0) {actionInProgress = "none";return;} // Ends early if already at the starting position
        distanceServo.setPosition(0); // Returns to start without scanning
    }

    /** Returns the distance without scanning. (This is scan() but does not rotate)
     * @return Returns as a double[]. Returns the distance and current rotation (in degrees).
     */
    public double[] getDistanceWithoutScan() {
        return new double[] {mb1043sensor.getDistance(), distanceServo.getPosition()};
    }

    private void rotate(int degreesToTravelPerScan) {
        double currentPosition = distanceServo.getPosition();
        double nextPosition = currentPosition + degreesToServoPosition(degreesToTravelPerScan);
        distanceServo.setPosition(nextPosition);
    }

    private Boolean delayNotFinished() {
        // If the time since the last ping is greater than the minimum delay then return true else return false
        return !((System.currentTimeMillis() - mb1043sensor.getLastPingTime()) > mb1043sensor.minDelayInMilliseconds);
    }

    private double degreesToServoPosition(int degrees) {
        // This assumes 0 degrees is the position 0.00
        try {
            // There is actually a lot of things that could go wrong
            // with this so there is a try catch just in case. (e.g. divide by zero)
            return (double)degrees / servoSpinAreaInDegrees;
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            return 0;
        }
    }

    private double degreesToServoPosition(double degrees) { // Same as above but for a double
        try {
            return degrees / servoSpinAreaInDegrees;
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            return 0;
        }
    }

    private double servoPositionToDegrees(double servoPosition) {
        return servoPosition * servoSpinAreaInDegrees;
    }

    /** Scans the area following the directions previously specified by the user.
     * If no directions have previously been specified it will scan every degree continuously.
     * @apiNote scan() was made to be called continuously in a loop() or while() statement.
     * @return Returns as a double[]. Returns the distance and current rotation (in degrees).
     */
    public double[] scan() {
        double distance = mb1043sensor.getDistance();
        double currentRotation = servoPositionToDegrees(distanceServo.getPosition());
        double[] disAndRot = {distance, currentRotation};
        if(delayNotFinished()) {
            actionInProgress = "none";
            return disAndRot;
        } // Anything below this line will not run until the distance can be found again




        /* Stops rotating if rotating the sensor would cause it to leave the minimum or maximum boundaries
                In addition, if it is currently executing "sweepOnce", it will finish the second half
                In addition, if it is currently executing a "scanOnce", it will stop
           Else, it will reverse direction */
        if(
                (((distanceServo.getPosition() + degreesToServoPosition(degreesToTravelPerScan)) <= 0) // If rotating would exit the lower boundary...
                        && //...and...
                        (degreesToTravelPerScan <= 0)) //...it is traveling towards the lower boundary
                || // OR
                (((distanceServo.getPosition() + degreesToServoPosition(degreesToTravelPerScan)) >= degreesToServoPosition(servoSpinAreaInDegrees)) // If rotating would exit the higher boundary...
                        && //...and...
                        (degreesToTravelPerScan >= 0)) //...it is traveling towards the higher boundary
        ) { // TL;DR, when it hits reaches a boundary, the following happens:



            if(
                    Objects.equals(actionInProgress, "sweepOnce") // If we are doing a single sweep...
                            && //...and...
                    distanceServo.getPosition() == degreesToServoPosition(servoSpinAreaInDegrees) //...the distance sensor is at the higher boundary
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
                    distanceServo.getPosition() == 0) //...the distance sensor is at the lower boundary
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
                                    (distanceServo.getPosition() == degreesToServoPosition(servoSpinAreaInDegrees)) // A.) the distance sensor is at the higher boundary
                                    || //OR
                                    (distanceServo.getPosition() == 0) // B.) The distance sensor is at the lower boundary
                            )
            ) {// then stop rotating and just scan in place
                actionInProgress = "none";
                return disAndRot;
            }

            // If none of those if statements happen it must be continuous rotation.
            degreesToTravelPerScan = degreesToTravelPerScan * -1; // So reverse directions to go back to the opposite boundary
            rotate(degreesToTravelPerScan);
            distance = mb1043sensor.getDistance();
            currentRotation = servoPositionToDegrees(distanceServo.getPosition());
            return new double[]{distance, currentRotation};

        }
        
        // If the sensor is not at a boundary...
        rotate(degreesToTravelPerScan);
        distance = mb1043sensor.getDistance();
        currentRotation = servoPositionToDegrees(distanceServo.getPosition());
        return new double[]{distance, currentRotation};
    }
}
