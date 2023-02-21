package org.firstinspires.ftc.teamcode.initialize;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.action.claw;
import org.firstinspires.ftc.teamcode.action.distanceSensor;
import org.firstinspires.ftc.teamcode.action.mecanumDrive;
import org.firstinspires.ftc.teamcode.action.swivel;
import org.firstinspires.ftc.teamcode.action.webcam;
import org.firstinspires.ftc.teamcode.other.redr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.other.redr.trajectorysequence.TrajectorySequence;

import java.util.Objects;

/** This OpMode is an Autonomous.
 * It uses Vuforia and TensorFlow to detect a custom (or default) sleeve on the signal cone.
 * In addition, it detects the high junction and places a cone on it.
 * The robot moves on a timer. This autonomous is a backup to our RoadRunner autonomous. */
@Autonomous(name = "RR Drive By Time", group = "A_ParkingOnly")
public class autoParkRR extends LinearOpMode {
    // CONSTRUCT
    public ElapsedTime autoRuntime = new ElapsedTime(); // How long the autonomous has run for
    ElapsedTime actionRuntime = new ElapsedTime(); // How long the current action has run for
    org.firstinspires.ftc.teamcode.action.distanceSensor distanceSensor = new distanceSensor();
    org.firstinspires.ftc.teamcode.action.mecanumDrive mecanumDrive = new mecanumDrive();
    org.firstinspires.ftc.teamcode.action.webcam webcam = new webcam();
    org.firstinspires.ftc.teamcode.action.claw claw = new claw();
    org.firstinspires.ftc.teamcode.action.swivel swivel = new swivel();
    // DECLARE NULL
    String tempDuck; // Stores the name of any newly found image. This will be null when no NEW image is found
    TrajectorySequence LeftParking;
    TrajectorySequence MiddleParking;
    TrajectorySequence RightParking;
    SampleMecanumDrive drive;
    // DECLARE CUSTOM
    String duck = "not found."; // Stores the name of the found image that has the highest confidence. This is the same as "tempDuck" but is never null
    public String teamColor = "Red"; // Which alliance we are currently on
    Boolean tFInitHasRun = false; // Has the TensorFlow initialise method run already?
    Boolean teamSelected = false; // Has the primary driver selected an alliance?
    Boolean autoFinished = false;
    // METHODS
    /* The primary driver will select which image detection model they want to use for the match
    This is determined by the alliance they are currently on
    In addition, the primary driver can choose to use the default model made by FTC instead of a custom sleeve */
    /** Loops until the start button is pressed. */
    public void runOpMode(){
        // INIT
        mecanumDrive.init(this);
        distanceSensor.init(this);
        claw.init(this);
        swivel.init(this);
        webcam.init(this, false, true, null);

        drive = new SampleMecanumDrive(hardwareMap);

        mecanumDrive.setMaxSpeed(1.00); // Sets the maximum speed of the wheels. Slow-mode is enabled automatically so this is really 0.5
        mecanumDrive.runWithoutEncoder();
        // INIT_LOOP
        while (opModeInInit()) {
            if (gamepad1.x) { // If the blue button (X) is pressed, select the blue alliance
                teamColor = "Blue";
            } else if (gamepad1.b) { // If the red button (B) is pressed, select the red alliance
                teamColor = "Red";
            } else if (gamepad1.a) { // If the green button (A) is pressed, use the default model
                teamColor = "Default";
            } else if (gamepad1.y) { // If the Y button is pressed, the driver has selected an alliance
                teamSelected = true;
            }
            if (teamSelected) { // When the team has been selected...
                if (!tFInitHasRun) { // Initialise TensorFlow if it has not been started already
                    webcam.init(this, true, false, teamColor);
                    tFInitHasRun = true;

                    LeftParking = drive.trajectorySequenceBuilder(new Pose2d(36.00, -65.00, Math.toRadians(90.00)))
                            .splineTo(new Vector2d(13.00, -54.00), Math.toRadians(90.00))
                            .splineTo(new Vector2d(13.00, -37.00), Math.toRadians(90.00))
                            .build();
                    MiddleParking = drive.trajectorySequenceBuilder(new Pose2d(36.00, -65.00, Math.toRadians(90.00)))
                            .splineTo(new Vector2d(36.00, -37.00), Math.toRadians(90.00))
                            .build();
                    RightParking = drive.trajectorySequenceBuilder(new Pose2d(36.00, -65.00, Math.toRadians(90.00)))
                            .splineTo(new Vector2d(61.00, -54.00), Math.toRadians(90.00))
                            .splineTo(new Vector2d(61.00, -37.00), Math.toRadians(90.00))
                            .build();
                    drive.setPoseEstimate(LeftParking.start());
                }

                tempDuck = webcam.tf_FindNewImages(); // The variable "tempDuck" contains the latest detected image name (if any)
                if (tempDuck != null) { // If "tempDuck" has a detected image name, it is more recent than what is currently in "duck"
                    duck = tempDuck; // Since we know the image name in "duck" is outdated, we update it to whatever image name is in "tempDuck"
                }

                // TELEMETRY
                telemetry.addData("Image", duck); // Displays the name of the latest image to the phone screen
                if (duck == null) { // If no image has been found, display "Loading..." on the phone screen
                    telemetry.addData("Status", "Loading...");
                } else { // If an image has been found, tell the drivers that autonomous can now be run
                    telemetry.addData("Status", "Ready! You can now run Autonomous.");
                }
                webcam.tf_OutputTelemetry(); // Outputs data about all detected images to the phone screen
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
                telemetry.addData("Status", "Press Y to confirm your team alliance and starting side.");
            }
            telemetry.update();
        }
        // START
        autoRuntime.reset(); // Resets both timers
        actionRuntime.reset();

        distanceSensor.distanceSensorTurnToDegree(10); // A servo always assumes it is at the starting position at the start (even if it is not)
        distanceSensor.returnToStart(); // Because of this we move it to not the start and back to the start

        claw.close();
        swivel.goToFront();
        waitForStart();

        while (opModeIsActive() && !autoFinished) {
            // LOOP
            telemetry.addData("Time Elapsed For Autonomous", autoRuntime.seconds()); // Time since the autonomous has started
            telemetry.addData("Time Elapsed For Action", actionRuntime.time()); // Displays the current time since the action has started

            if(Objects.equals(duck, "Turtle") || Objects.equals(duck, "Bolt") && !autoFinished) {
                drive.followTrajectorySequence(LeftParking);
                autoFinished = true;
            }

            if (Objects.equals(duck, "Robot") || Objects.equals(duck, "Light") || Objects.equals(duck, null)) {
                // Parks in the middle zone if the image is a robot or light bulb
                // If no image is ever found, this will run
                // If the driver does not wait for the image detector to load, this will run
                drive.followTrajectorySequence(MiddleParking);
                autoFinished = true;
            }

            if(Objects.equals(duck, "Handsaw") || Objects.equals(duck, "Panel")) {
                // Parks in the right zone if the image is the handsaw or solar panel
                drive.followTrajectorySequence(RightParking);
                autoFinished = true;
            }
            telemetry.update();
        }
    }
}