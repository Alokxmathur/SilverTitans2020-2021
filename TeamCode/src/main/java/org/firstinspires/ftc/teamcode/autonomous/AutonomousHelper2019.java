package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

public abstract class AutonomousHelper2019 extends OpMode {

    protected Match match;
    protected Robot robot;
    protected Alliance.Color allianceColor;
    protected double headingToFoundation;

    public static final long VUFORIA_SETTLING_TIME = 1500; //msecs for vuforia to see image
    public static final double STARTING_POSITION = Field.TILE_WIDTH + MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2;
    public static final double CAUTIOUS_SPEED = 0.6;
    public static final double FAST_SPEED = 1.0;
    public static final double START_TO_CLEAR_TAPE = 1.5*Field.TILE_WIDTH;
    public static final double RETRACTION_FROM_QUARRY = 0f*Field.MM_PER_INCH;

    protected double strafeToCenterOnFirstSkyStoneDistance, moveForwardToGatherFirstStone;
    protected boolean initialMovementDone, initialMovementQueued;
    protected boolean grabbedFirstSkyStone, queuedGrabFirstSkystone;
    protected boolean reachedBottomStone, queuedReachBottomStone;
    protected boolean grabbedSecondStone, queuedSecondStoneGrab;
    protected boolean returnedForSecondStone, queuedReturnForSecondStone;
    protected boolean positionedForSecondStone, queuedPositionForSecondStone;
    protected boolean firstStonedDelivered, queuedFirstDelivery;
    protected boolean firstTapeCleared, queuedFirstTapeClearance;
    protected boolean firstStonedPlaced, queuedFirstPlacement;
    protected boolean deliveredSecondStone, queuedSecondStoneDelivery;
    protected boolean foundationMoved, queuedFoundation;
    protected double lateralTravel;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(Alliance.Color allianceColor) {
        this.allianceColor = allianceColor;
        this.match = Match.getNewInstance();
        match.init();
        match.setAlliance(allianceColor);
        this.robot = match.getRobot();
        this.robot.init(hardwareMap, telemetry, match, allianceColor);
        headingToFoundation =  allianceColor == Alliance.Color.BLUE ? 90 : -90;

        AutoTransitioner.transitionOnStop(this, "Phoebe: Driver Controlled");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (robot.fullyInitialized()) {
            //keep looking for the sky stone position in the quarry
            match.setSkyStonePosition(robot.getSkyStoneLocation());
            //update driver station with sky stone position
            telemetry.addData("Alliance", allianceColor);
            telemetry.addData("Sky stone location", match.getSkyStonePosition());
            telemetry.addData("Status", "Ready to autonomous");
            telemetry.addData("Motors", robot.getMotorStatus());
            telemetry.addData("Picker", robot.getPickerArmStatus());
            telemetry.update();
        }
        else {
            telemetry.addData("status", "Waiting for vuForia to finish, please wait");
            telemetry.update();
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        match.setStart();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        this.robot.stop();
    }

    public Alliance.Color getAlliance() {
        return allianceColor;
    }

    /**
     * Turn on led lights, lower the foundation gripper and
     * get gripper to hover position and open it
     *
     * We do all of this in on the secondary thread so that we don't have to wait for them to
     * complete before reaching the bottom sky stone
     */
    protected void queueInitialOperations() {
        //queue following operations on tertiary thread
        //turn on flash
        robot.queueTertiaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
        //lower foundation gripper so camera can see
        robot.queueTertiaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.LOWER, "Lower foundation gripper"));

        //queue following operations on tertiary thread
        //get gripper to hover position
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Gripper to hover"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper"));
    }


    protected void flash() {
        //turn off flash
        robot.queueSecondaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_OFF, "Turn off flash"));
        //wait
        robot.queueSecondaryOperation(new WaitOperation(1000, "Flash"));
        //turn on flash
        robot.queueSecondaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
        //wait
        robot.queueSecondaryOperation(new WaitOperation(1000, "Flash"));
        //turn off flash
        robot.queueSecondaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_OFF, "Turn off flash"));
    }
}
