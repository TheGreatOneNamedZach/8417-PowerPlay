package org.firstinspires.ftc.teamcode.initialize;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.action.imageDetectionTF;
import org.firstinspires.ftc.teamcode.action.mecanumDrive;

import java.util.List;
import java.util.Objects;

/**
 * This OpMode is an Autonomous.
 * It uses Vuforia and TensorFlow to detect a custom (or default) sleeve on the signal cone.
 * In addition, it detects the high junction and places a cone on it.
 * The robot moves on a timer. This autonomous is a backup to our RoadRunner autonomous.
 */
@Autonomous
public class auto extends OpMode {
    // CONSTRUCT
    imageDetectionTF detector = new imageDetectionTF(); // Image detector
    public ElapsedTime autoRuntime = new ElapsedTime(); // How long the autonomous has run for
    ElapsedTime actionRuntime = new ElapsedTime(); // How long the current action has run for
    // DECLARE NULL
    DcMotor elevator;
    Servo claw;
    DigitalChannel digitalTouch;
    org.firstinspires.ftc.teamcode.action.distanceSensor distanceSensor;
    org.firstinspires.ftc.teamcode.action.mecanumDrive mecanumDrive;
    Boolean teamSelected; // Has the primary driver selected an alliance?
    Boolean tFInitHasRun; // Has the TensorFlow initialise method run already?
    Boolean isPressed;
    String tempDuck; // Stores the name of any newly found image. This will be null when no NEW image is found
    // DECLARE CUSTOM
    int robotAction = 0; // Keeps track of which action the bot is currently doing
    double length = 1.0; // The time the action runs for\
    double[] distance = {-1, -1, -1, -1};
    String duck = "not found."; // Stores the name of the found image that has the highest confidence. This is the same as "tempDuck" but is never null
    public String teamColor = "Red"; // Which alliance we are currently on

    // METHODS
    /** Initializes the autonomous. */
    public void init(){
        telemetry.addData("", "Please wait...");

        claw = hardwareMap.get(Servo.class, "Claw");
        elevator = hardwareMap.get(DcMotor.class, "Elevator");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "Touch Sensor");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Gives information to access OpMode specific objects (e.g. motors, webcam, and telemetry)
        detector.init(this);
        distanceSensor.init(this);
        mecanumDrive.init(this);

        // Initialises Vuforia
        detector.initVuforia();

        mecanumDrive.setMaxSpeed(1.00); // Sets the maximum speed of the wheels. Slow-mode is enabled automatically so this is really 0.5
        mecanumDrive.runWithoutEncoder();
    }

    /* The primary driver will select which image detection model they want to use for the match
    This is determined by the alliance they are currently on
    In addition, the primary driver can choose to use the default model made by FTC instead of a custom sleeve */
    /** Loops until the start button is pressed. */
    public void init_loop(){
        if(gamepad1.x) { // If the blue button (X) is pressed, select the blue alliance
            teamColor = "Blue";
        } else if (gamepad1.b) { // If the red button (B) is pressed, select the red alliance
            teamColor = "Red";
        } else if (gamepad1.a) { // If the green button (A) is pressed, use the default model
            teamColor = "Default";
        } else if (gamepad1.y) { // If the Y button is pressed, the driver has selected an alliance
            teamSelected = true;
        }
        if(teamSelected) { // When the team has been selected...
            if(!tFInitHasRun) { // Initialise TensorFlow if it has not been started already
                detector.initTfod(teamColor); // Tell TensorFlow to use the model corresponding to the alliance
                tFInitHasRun = true;
            }

            tempDuck = detector.imageReturn(); // The variable "tempDuck" contains the latest detected image name (if any)
            if(tempDuck != null){ // If "tempDuck" has a detected image name, it is more recent than what is currently in "duck"
                duck = tempDuck; // Since we know the image name in "duck" is outdated, we update it to whatever image name is in "tempDuck"
            }

            // TELEMETRY
            telemetry.addData("Image", duck); // Displays the name of the latest image to the phone screen
            if(duck == null) { // If no image has been found, display "Loading..." on the phone screen
                telemetry.addData("Status", "Loading...");
            } else { // If an image has been found, tell the drivers that autonomous can now be run
                telemetry.addData("Status", "Ready! You can now run Autonomous.");
            }
            tFTelemetry(detector.tfod); // Outputs data about all detected images to the phone screen
        } else { // If the team has not been selected by the driver...
            if (Objects.equals(teamColor, "Blue")) { // Tell them which team is currently selected
                telemetry.addData("Team Alliance", "You are on the " + teamColor + " alliance.\nUse B to change to the red alliance.\nPress A to use no custom sleeve.");
            } else if (Objects.equals(teamColor, "Red")) {
                telemetry.addData("Team Alliance", "You are on the " + teamColor + " alliance.\nUse X to change to the blue alliance.\nPress A to use no custom sleeve.");
            } else {
                telemetry.addData("Team Alliance", "You are using the " + teamColor + " signal cone.\nUse X to change to the blue alliance.\nUse B to change to the red alliance.");
            }

            // TELEMETRY
            // Tells the primary driver how to confirm their selection
            // Don't add telemetry.update() or the webcam stops working properly. I have no clue why
            telemetry.addData("Status", "Press Y to confirm your team alliance.");
        }
    }

    /** Runs one time when the autonomous starts. */
    public void start() {
        autoRuntime.reset(); // Resets both timers
        actionRuntime.reset();
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        distanceSensor.distanceSensorTurnToDegree(10); // A servo always assumes it is at the starting position at the start (even if it is not)
        distanceSensor.returnToStart(); // Because of this we move it to not the start and back to the start

        claw.setPosition(.64);
    }

    /** Loops until the stop button is pressed. */
    @Override
    public void loop() {
        // TELEMETRY
        telemetry.addData("Time Elapsed For Autonomous", autoRuntime.seconds()); // Time since the autonomous has started
        telemetry.addData("Time Elapsed For Action", actionRuntime.time()); // Displays the current time since the action has started
        telemetry.addData("Robot Action", robotAction); // Displays the current action the robot is on
        telemetry.addData("Elev", elevator.getCurrentPosition());

        isPressed = !digitalTouch.getState();

        if(Objects.equals(duck, "Turtle") || Objects.equals(duck, "Bolt")) {

            if(robotAction <= 7) {
                goToPoleFromStart(false);
            } else if (robotAction <= 24) {
                putConeOnPole();
            } else if (robotAction == 25) {
                mecanumDrive(1.40, 0, -1, 0);
            }

            try {
                telemetry.addData("Distance", distance[0] + ", X " + distance[2] * 1 + ", Y " + distance[3]);
                telemetry.addData("Y", cmToSeconds(distance[3]) + " || " + Integer.signum((int)distance[3]));
                telemetry.addData("X", cmToSeconds(distance[2]) + " || " + Integer.signum((int)distance[2]));
            } catch (Exception ignored) {}




            /*
            // Parks in the left zone if the image is a turtle or lightning bolt
            if(robotAction == 0){ // This action is reserved for initialising motors before the robot moves (e.g. grabbing the preloaded cone)
                //mecanumDrive(length, 0, 0, 0);
                robotAction++;
            } else if(robotAction == 1){ // Moves forwards off of the wall
                mecanumDrive(.2, 1, 0, 0);
            } else if(robotAction == 2){ // Waits for 0.2 seconds
                mecanumDrive(.2, 0,0,0);
            } else if (robotAction == 3) { // Strafes left
                mecanumDrive(length, 0,  -1, 0);
            } else if(robotAction == 4){ // Waits for 0.2 seconds
                mecanumDrive(.2, 0,0,0);
            } else if (robotAction == 5) { // Moves forwards
                mecanumDrive(length,1, 0, 0);
            } else if(robotAction == 6){ // Waits 1 second
                mecanumDrive(length, 0,0,0);
            }*/
        }

        if (Objects.equals(duck, "Robot") || Objects.equals(duck, "Light") || Objects.equals(duck, null)) {
            // Parks in the middle zone if the image is a robot or light bulb
            // If no image is ever found, this will run
            // If the driver does not wait for the image detector to load, this will run
            if(robotAction == 0){ // This action is reserved
                //mecanumDrive(length, 0,0,0);
                robotAction++;
            } else if(robotAction == 1) { // Moves forward for 1.5 seconds
                mecanumDrive(length + .5, 1, 0, 0);
            } else if(robotAction == 2){ // Waits 0.2 seconds
                mecanumDrive(.2,0,0,0);
            }
        }

        if(Objects.equals(duck, "Handsaw") || Objects.equals(duck, "Panel")) {
            // Parks in the right zone if the image is the handsaw or solar panel
            if(robotAction == 0){ // This action is reserved
                //mecanumDrive(length,0,0,0);
                robotAction++;
            } else if(robotAction == 1) { // Moves forwards off the wall
                mecanumDrive(.1, 1, 0, 0);
            } else if(robotAction == 2){ // Waits 0.2 seconds
                mecanumDrive(.2,0,0,0);
            } else if(robotAction == 3) { // Strafes right
                mecanumDrive(length, 0, 1, 0);
            } else if(robotAction == 4){ // Waits 0.2 seconds
                mecanumDrive(.2,0,0,0);
            } else if(robotAction == 5) { // Moves forwards
                mecanumDrive(length, 1, 0, 0);
            } else if(robotAction == 6) { // Waits 0.2 seconds
                mecanumDrive(.2, 0, 0, 0);
            }
        }
    }

    private void goToPoleFromStart(Boolean terminalSide) {
        double xAxisPower = terminalSide ? -1 : 1;
        if(robotAction == 0) {
            mecanumDrive(.1, 1, 0, 0); // Off wall
        } else if (robotAction == 1) {
            waitThenGoToNextAction(.2);
        } else if (robotAction == 2) {
            mecanumDrive(1, 0, xAxisPower,0); // Strafe towards pole
        } else if (robotAction == 3) {
            waitThenGoToNextAction(0.2);
        } else if (robotAction == 4) {
            mecanumDrive(0.10, 0, 0, 0.5);
        } else if (robotAction == 5) {
            waitThenGoToNextAction(0.2);
        } else if (robotAction == 6) {
            mecanumDrive(0.15, 1, 0, 0);
        } else if (robotAction == 7) {
            distanceSensor.distanceSensorTurnToDegree(15);
            distanceSensor.startScanning(1);
            if (actionRuntime.time() >= .5) {
                robotAction++;
                actionRuntime.reset();
            }
        }
    }

    private void putConeOnPole() {
        if(distance[0] <= 60 || distance[0] >= 90) {
            distance = distanceSensor.scan();
            actionRuntime.reset(); // Remove this and it skips this action
        } else {
            distanceSensor.shutdown();
            if (robotAction == 8) {
                // Goes forwards
                // In addition, if the pole is behind it, it goes backwards. This should never happen but it could.
                mecanumDrive(cmToSeconds(distance[3] - 0.00), Integer.signum((int)distance[3]), 0, 0);
            } else if (robotAction == 9) {
                waitThenGoToNextAction(0.2);
            } else if (robotAction == 10) {
                mecanumDrive(0.05, 0, 0, 0.5); // Rot 0.1
            } else if (robotAction == 11) {
                waitThenGoToNextAction(0.2);
            } else if (robotAction == 12) {
                // Strafes
                // In addition, if the pole is to the left of it, it goes to the left
                mecanumDrive(cmToSeconds(distance[2] + 3.00), 0, Integer.signum((int)distance[2]), 0);
            } else if (robotAction == 13) {
                waitThenGoToNextAction(0.2);
            } else if (robotAction == 14) {
                setElevator(3100);
            } else if (robotAction == 15) {
                waitThenGoToNextAction(0.2);
            } else if (robotAction == 16) {
                mecanumDrive(0.2, 0.5, 0, 0);
            } else if (robotAction == 17) {
                waitThenGoToNextAction(0.2);
            } else if (robotAction == 18) {
                setElevator(2260);
            } else if (robotAction == 19) {
                waitThenGoToNextAction(0.2);
            } else if (robotAction == 20) {
                claw.setPosition(.42);
                robotAction++;
                actionRuntime.reset();
            } else if (robotAction == 21) {
                waitThenGoToNextAction(0.2);
            } else if (robotAction == 22) {
                mecanumDrive(0.2, 0.5, 0, 0);
            } else if (robotAction == 23) {
                waitThenGoToNextAction(0.2);
            } else if (robotAction == 24) {
                setElevator(0);
            }
        }
    }

    private double cmToSeconds(double cm) {
        // 1 field tile = 60 cm = 0.54545 seconds
        // 1.8333 field tiles = 110 cm = 1 second
        // 0.01667 field tiles = 1 cm = 0.0090909 seconds
        return Math.abs(cm * 0.0090909);
    }

    public void mecanumDrive(double secondsToRunFor, double yAxisPower, double xAxisPower, double rotationInPower) {
        mecanumDrive.setPower(xAxisPower, yAxisPower, rotationInPower);
        if(actionRuntime.time() >= secondsToRunFor){ // If this action runs longer than it should...
            robotAction++; // Go to the next action
            actionRuntime.reset(); // Reset the timer
            mecanumDrive.setPower(0, 0, 0);
        }
    }

    private void waitThenGoToNextAction(double secondsToWait) {
        if(actionRuntime.time() >= secondsToWait){ // If this action runs longer than it should...
            robotAction++; // Go to the next action
            actionRuntime.reset(); // Reset the timer
        }
    }

    private void tFTelemetry(TFObjectDetector tfod) { // Displays data about images detected to the phone screen
        if (tfod != null) { // If the image detector has started up...
            List<Recognition> recognitionsList = tfod.getRecognitions(); // Creates a list with every image detected
            if (recognitionsList != null) { // This will run if at least one image has been detected
                telemetry.addData("Number Of Images Detected", recognitionsList.size());

                for (Recognition recognition : recognitionsList) {
                    // This "for" statement will run for every image detected
                    double col = (recognition.getLeft() + recognition.getRight()) / 2; // Finds the "X" position of the image on the webcam stream
                    double row = (recognition.getTop() + recognition.getBottom()) / 2; // Finds the "Y" position of the image on the webcam stream
                    double width = Math.abs(recognition.getRight() - recognition.getLeft()); // Finds the width of the image on the webcam stream
                    double height = Math.abs(recognition.getTop() - recognition.getBottom()); // Finds the heights of the image on the webcam stream

                    // Adds a blank line between every image detected
                    telemetry.addData("", " ");
                    // Displays to the phone screen the name of the image and how confident that it is correct
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    // Displays to the phone screen the X and Y coordinates of the image. The coordinates are the image's position on the webcam stream
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    // Displays to the phone screen the size of the image. The size of the image output is the size on the webcam stream
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                }
            }
        }
    }

    private void setElevator(int ticks) {
        if(elevator.getCurrentPosition() >= ticks - 25 && ticks > 25 && elevator.getCurrentPosition() <= ticks + 25) {
            elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevator.setPower(0.025);
            robotAction++;
            actionRuntime.reset();
        } else if (ticks > 25){
            elevator.setTargetPosition(ticks);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(Math.min(0.6, (3600 - elevator.getCurrentPosition()) * 0.01));
        } else if (ticks == 0) {
            elevator.setTargetPosition(ticks);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator.setPower(Math.min(0.5, (-elevator.getCurrentPosition()) * 0.005));
            if(elevator.getCurrentPosition() <= 6) {
                elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator.setPower(0);
                robotAction++;
                actionRuntime.reset();
            }
        }
    }

    /** Stops the robot. */
    @Override
    public void stop() {
        detector.tfod.shutdown();
        super.stop(); // Stops the OpMode
    }
}