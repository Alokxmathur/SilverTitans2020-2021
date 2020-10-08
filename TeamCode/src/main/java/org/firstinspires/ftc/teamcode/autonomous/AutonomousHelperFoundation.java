package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;

public abstract class AutonomousHelperFoundation extends AutonomousHelper {
    public static final double SPEED = .4;
    public static final double INITIAL_APPROACH_DISTANCE = 30 * Field.MM_PER_INCH;
    public static final double PULL_DISTANCE = 60*Field.MM_PER_INCH;
    public static final double BACKWARD_TO_NAVIGATE_DISTANCE = 1.25*Field.TILE_WIDTH;

    @Override
    public void start() {
        //Get the picker arm extended, then open the gripper and bring it back to the carry position
        //Do this on the secondary operation while approaching the foundation
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Hover"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CARRY, "Carry"));

        //queue forward movement towards foundation
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(INITIAL_APPROACH_DISTANCE, 0, SPEED, "Move towards foundation"));
        //lower foundation gripper
        robot.queuePrimaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.LOWER, "Grab foundation"));
        //queue backward movement towards building site
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-PULL_DISTANCE, headingToFoundation, SPEED, "Move towards wall and rotate"));
        //raise foundation gripper
        robot.queuePrimaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.RAISE, "Free the foundation"));

        robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Rotate back"));
        //Go back to navigate
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-BACKWARD_TO_NAVIGATE_DISTANCE, headingToFoundation, SPEED,"Back to navigate"));
    }

    @Override
    public void loop() {
        //we do nothing in the autonomous loop
        //The start method already would have queued up the operations needed
    }

}
