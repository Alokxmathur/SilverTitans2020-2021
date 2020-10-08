package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;

public abstract class AutonomousHelperNavigate extends AutonomousHelper {
    public static final double SPEED = .4;
    public static final double INITIAL_APPROACH_DISTANCE = 2 * Field.MM_PER_INCH;
    public static final double BACKWARD_TO_NAVIGATE_DISTANCE = 2*Field.TILE_WIDTH;

    @Override
    public void init(Alliance.Color allianceColor) {
        super.init(allianceColor);
    }
    @Override
    public void start() {
        queueInitialOperations();
    }
    @Override
    public void loop() {
    }

    /**
     * Move forward towards quarry while turning the flash on and getting the gripper to a position where it is close to capturing a stone
     * and wait to see a VuMark
     */
    @Override
    protected void queueInitialOperations() {
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Hover"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CARRY, "Carry"));
        //queue forward movement towards foundation
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(INITIAL_APPROACH_DISTANCE, 0, SPEED, "Move towards foundation"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Rotate back"));
        //Go back to navigate
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-BACKWARD_TO_NAVIGATE_DISTANCE, headingToFoundation, SPEED,"Back to navigate"));
    }
}
