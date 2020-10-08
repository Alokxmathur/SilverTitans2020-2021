package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

public abstract class AutonomousHelperStonesPlacedFoundation extends AutonomousHelperStonesPlaced {
    double travelToMiddleOfFoundation = 1.5*Field.TILE_WIDTH - 6*Field.MM_PER_INCH;

    public void init(Alliance.Color allianceColor) {
        super.init(allianceColor);
    }

    @Override
    protected void queueFirstDelivery() {
        //queue raising of the foundation gripper on the tertiary thread
        robot.queueTertiaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.RAISE, "Raise foundation gripper"));

        //queue up secondary thread operation to get the gripper to placement position
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.PLACE, "Place"));

        //move another tile width and a half beyond the tape
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(travelToMiddleOfFoundation, headingToFoundation, FAST_SPEED, "Get to foundation"));
        //face the foundation
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Face foundation"));
        //approach the foundation
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(8*Field.MM_PER_INCH, headingToFoundation, CAUTIOUS_SPEED,"Approach foundation to grab it"));
        //face the foundation
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Realign after approaching foundation"));
        //grab the foundation
        robot.queuePrimaryOperation(new FoundationGripperOperation(
                FoundationGripperOperation.OperationType.LOWER,
                "Grab the foundation"));
    }

    @Override
    protected void queueFirstPlacement() {
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Get stone to drop level"));
        robot.queueSecondaryOperation(new WaitOperation(500, "Wait before placing stone"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Drop stone"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Hover to clear dropped stone"));

        //queue backward movement to drag foundation closer to building site
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(
                -36*Field.MM_PER_INCH, 0, CAUTIOUS_SPEED, "Drag foundation"));

        //release foundation by raising foundation gripper
        robot.queuePrimaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.RAISE, "Release foundation"));

        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Realign after dragging foundation"));
        //add an extra 2 inches for us to clear the foundation
        travelToMiddleOfFoundation += 2*Field.MM_PER_INCH;
        lateralTravel -= 2*Field.MM_PER_INCH;
        //Strafe to get away from the foundation
        robot.queuePrimaryOperation(
                new StrafeLeftForDistanceOperation(
                        allianceColor == Alliance.Color.BLUE ? -travelToMiddleOfFoundation : travelToMiddleOfFoundation,
                        FAST_SPEED,
                        "Get away from foundation"));
        //move up close to the bridge
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(16*Field.MM_PER_INCH, 0, CAUTIOUS_SPEED, "Near Middle"));
    }

    @Override
    protected void queueSecondStoneDelivery() {
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Level position"));
        //deliver second sky stone
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(
                lateralTravel + 12*Field.MM_PER_INCH,
                headingToFoundation, FAST_SPEED,
                "Move towards foundation"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.PLACE, "Place position"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Lower so stone does not fall off"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Drop"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Hover after dropping"));
    }

    @Override
    protected void queueUnderBridge() {
        //robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Align to navigate"));
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-Field.TILE_WIDTH, headingToFoundation, FAST_SPEED, "Navigate"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Level"));
        flash();
        flash();
        flash();
        flash();
    }

}
