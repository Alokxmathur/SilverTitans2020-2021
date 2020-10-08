package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitUntilVuMarkOperation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.game.Alliance.Color.BLUE;
import static org.firstinspires.ftc.teamcode.game.Alliance.Color.RED;

public abstract class AutonomousHelperStones extends AutonomousHelper2019 {

    public void init(Alliance.Color allianceColor) {
        super.init(allianceColor);
    }

    public void loop() {
        if (! initialMovementDone) {
            //initiate Phoebe
            if (! initialMovementQueued) {
                this.queueInitialOperations();
                initialMovementQueued = true;
            }
            initialMovementDone = robot.primaryOperationsCompleted();
        }
        else if (!reachedBottomStone) {
            if (!queuedReachBottomStone) {
                Match.log("Reaching bottom stone");
                queueReachBottomSkyStone();
                queuedReachBottomStone = true;
            }
            reachedBottomStone = robot.operationsCompleted();
        }
        else if (!grabbedFirstSkyStone) {
            if (!queuedGrabFirstSkystone) {
                Match.log("Grabbing first skystone");
                grabFirstSkyStone();
                queuedGrabFirstSkystone = true;
            }
            grabbedFirstSkyStone = robot.operationsCompleted();
        }
        else if (!firstTapeCleared) {
            if (!queuedFirstTapeClearance) {
                Match.log("Queueing first tape clearance");
                queuedFirstTapeClearance();
                queuedFirstTapeClearance = true;
            }
            firstTapeCleared = robot.operationsCompleted();
        }
        else if (!firstStonedDelivered) {
            if (!queuedFirstDelivery) {
                Match.log("Queueing first skystone delivery");
                this.queueFirstDelivery();
                queuedFirstDelivery = true;
            }
            firstStonedDelivered = robot.operationsCompleted();
        }
        else if (!returnedForSecondStone) {
            if (!queuedReturnForSecondStone) {
                Match.log("Queueing second skystone fetch");
                //go back for second skystone
                queueReturnForSecondStone();
                queuedReturnForSecondStone = true;
            }
            returnedForSecondStone = robot.operationsCompleted();
        } else if (!positionedForSecondStone) {
            if (!queuedPositionForSecondStone) {
                Match.log("Queueing second skystone position");
                //line up with second stone
                queuePositionForSecondStone();
                queuedPositionForSecondStone = true;
            }
            positionedForSecondStone = robot.operationsCompleted();
        }
        else if (!grabbedSecondStone) {
            if (!queuedSecondStoneGrab) {
                Match.log("Queueing second skystone grab");
                queueSecondStoneGrab();
                queuedSecondStoneGrab = true;
            }
            grabbedSecondStone = robot.operationsCompleted();
        }
        else if (!deliveredSecondStone) {
            if (!queuedSecondStoneDelivery) {
                Match.log("Queueing second skystone delivery");
                queueSecondStoneDelivery();
                queuedSecondStoneDelivery = true;
            }
            deliveredSecondStone = robot.operationsCompleted();
        }
        else if (!foundationMoved) {
            if (!queuedFoundation) {
                Match.log("Navigating under bridge");
                this.queueUnderBridge();
                queuedFoundation = true;
            }
            foundationMoved = robot.operationsCompleted();
        }
    }

    @Override
    /**
     * Turn on led lights, lower the foundation gripper on tertiary thread
     * Get gripper to hover position and open it on secondary thread
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

        //queue following operations on secondary thread
        //get gripper to hover position
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Gripper to hover"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Open gripper"));
    }

    /**
     * Reach the bottom sky stone, the one closer to the audience
     */
    protected void queueReachBottomSkyStone() {
        //move away from the wall towards the quarry, the distance moved should have the
        // gripper hovering right over the quarry
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(
                Field.QUARRY_FROM_WALL - Robot.GRIPPER_FORWARD_DISPLACEMENT - PickerArm.HOVER_EXTENSION - Robot.LENGTH/2, 0, CAUTIOUS_SPEED,"Closer to quarry"));

        //find which stone number we are going for
        int stoneNumberFromBottom = 0;
        if (match.getSkyStonePosition() == Field.SkyStonePosition.MIDDLE) {
            stoneNumberFromBottom = 1;
        }
        else if (match.getSkyStonePosition() == Field.SkyStonePosition.TOP) {
            stoneNumberFromBottom = 2;
        }
        //determine how far we have to strafe
        strafeToCenterOnFirstSkyStoneDistance = STARTING_POSITION -
                stoneNumberFromBottom*Field.STONE_LENGTH - Field.STONE_LENGTH/2
                + (allianceColor == BLUE ? Robot.GRIPPER_LEFT_DISPLACEMENT : -Robot.GRIPPER_LEFT_DISPLACEMENT);
        if (match.getSkyStonePosition() == Field.SkyStonePosition.BOTTOM) {
            //if we are reaching for the bottom stone, we should adjust how much we have to strafe because
            //the robot can only reach the wall and not center on the sky stone at the bottom
            strafeToCenterOnFirstSkyStoneDistance -= (MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2 - Field.STONE_LENGTH/2);
            //add an operation to strafe for time instead of distance
            robot.queuePrimaryOperation(new StrafeLeftForTimeOperation(1500, CAUTIOUS_SPEED, allianceColor == BLUE, "Reach the wall"));
        }
        else {
            //add operation to strafe to the bottom sky stone
            robot.queuePrimaryOperation(new StrafeLeftForDistanceWithHeadingOperation(
                    allianceColor == RED ? strafeToCenterOnFirstSkyStoneDistance : -strafeToCenterOnFirstSkyStoneDistance,
                    0,
                    CAUTIOUS_SPEED,
                    String.format("Strafe to %s sky stone, %.2f\"", match.getSkyStonePosition(), strafeToCenterOnFirstSkyStoneDistance / Field.MM_PER_INCH)));
            // realign after strafing
            robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Realign with quarry after strafe to bottom"));
        }
        robot.queuePrimaryOperation(new WaitUntilVuMarkOperation(VUFORIA_SETTLING_TIME, "Find sky using VuForia"));
    }

    /*
    Grab the first sky stone
    We use the distance measured by vuForia to fine tune the distance to the quarry
     */
    protected void grabFirstSkyStone() {
        Match.log(String.format(Locale.getDefault(), "First Sky stone distance=%.2f", -robot.getCurrentX()/Field.MM_PER_INCH));
        if (robot.findTarget() != null) {
            moveForwardToGatherFirstStone = -robot.getCurrentX()
                    - Robot.GRIPPER_FORWARD_DISPLACEMENT
                    + (Field.STONE_WIDTH / 2) - PickerArm.HOVER_EXTENSION;
        }
        else {
            Match.log("Oops- Not seeing first sky stone");
        }

        //move so that the hovering gripper is right over the sky stone
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(moveForwardToGatherFirstStone, 0, CAUTIOUS_SPEED, "Inch towards stone"));
        //wait for gripper to settle
        //robot.queuePrimaryOperation(new WaitOperation(600, "Wait for gripper to stabilize"));
        //lower shoulder onto sky stone
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_GRAB, "Shoulder Grab Position"));
        //grab sky stone
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Grab"));
        //lift the sky stone up
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Lift "));
        //retract from the quarry
        //robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-RETRACTION_FROM_QUARRY, 0, CAUTIOUS_SPEED, "Move away from stone"));
    }

    /**
     * Go under bridge and clear the tape
     */
    protected void queuedFirstTapeClearance() {
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LIFT, "Lift before leveling"));
        //queue leveling of the arm on the secondary thread
        robot.queueSecondaryOperation(new WaitOperation(1500, "Wait before leveling"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Level"));

        //move half the robot length away from the bottom wall so we can rotate without touching it
        robot.queuePrimaryOperation(new StrafeLeftForDistanceWithHeadingOperation(
                allianceColor == Alliance.Color.BLUE ? Robot.LENGTH/2 : -Robot.LENGTH/2,
                0,
                CAUTIOUS_SPEED,
                "Clear wall so we can rotate"));

        lateralTravel = START_TO_CLEAR_TAPE
                + strafeToCenterOnFirstSkyStoneDistance;
        //rotate to face the wall away from the audience to go under the bridge
        //robot.queuePrimaryOperation(new RotationOperation(robot.getBearing()-headingToFoundation, CAUTIOUS_SPEED, "Rotate towads bridge"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Bearing toward bridge"));
        //travel to clear the tape
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(lateralTravel, headingToFoundation, FAST_SPEED, "Clear tape"));
    }

    protected void queueFirstDelivery() {
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Drop"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Realign"));

        //calculate how much to return for second stone

        //add half the drive train width we moved away from wall to clear it when we picked up the bottom stone
        lateralTravel += MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2;
        //we subtract 3*stone length since we are going back to the sky-stone that is closer to the bridge
        lateralTravel -= 3*Field.STONE_LENGTH;
        //if we came from the bottom stone, we weren't centered on it so add the extra distance to travel back
        if (match.getSkyStonePosition() == Field.SkyStonePosition.BOTTOM) {
            lateralTravel += (MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2 - Field.STONE_LENGTH/2);
        }
        //return to second stone
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-lateralTravel, headingToFoundation, FAST_SPEED, "Travel back"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Face quarry"));
    }

    protected void queueReturnForSecondStone() {
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Hover"));
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(RETRACTION_FROM_QUARRY - moveForwardToGatherFirstStone, 0, CAUTIOUS_SPEED, "Closer to quarry"));
        robot.queuePrimaryOperation(new WaitUntilVuMarkOperation(VUFORIA_SETTLING_TIME, "Give vuforia time to see"));
    }

    protected void queuePositionForSecondStone() {
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Realign with quarry"));
        if (robot.findTarget() != null) {
            Match.log(String.format(Locale.getDefault(), "Second Sky stone distance=%.2f", -robot.getCurrentX()/Field.MM_PER_INCH));
            //strafe based on (relative to Skystone), the y position of robot
            //we subtract the distance the gripper center is offset to the left from the center of the robot
            double strafeToCenterOnSecondSkyStoneDistance = -robot.getCurrentY() - Robot.GRIPPER_LEFT_DISPLACEMENT;
            //we skip centering if we have to strafe less than 3 inches as this can be handled by the gripper design
            if (Math.abs(strafeToCenterOnSecondSkyStoneDistance) < 3*Field.MM_PER_INCH) {
                Match.log(String.format("Skipped strafing %.2f\"", strafeToCenterOnSecondSkyStoneDistance /Field.MM_PER_INCH));
                strafeToCenterOnSecondSkyStoneDistance = 0;
            }
            else {
                robot.queuePrimaryOperation(new StrafeLeftForDistanceWithHeadingOperation(strafeToCenterOnSecondSkyStoneDistance, 0,
                        CAUTIOUS_SPEED, String.format("Strafe to sky stone %.2f\"", strafeToCenterOnSecondSkyStoneDistance /Field.MM_PER_INCH)));
                lateralTravel += (allianceColor == Alliance.Color.BLUE ? -strafeToCenterOnSecondSkyStoneDistance : strafeToCenterOnSecondSkyStoneDistance);
            }
            //move so that the hovering gripper is right over the sky stone
            robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-robot.getCurrentX()
                    - Robot.GRIPPER_FORWARD_DISPLACEMENT
                    + (Field.STONE_WIDTH/2) - PickerArm.HOVER_EXTENSION, 0, CAUTIOUS_SPEED, "Inch towards stone"));
        }
        else {
            Match.log("Not seeing the second sky stone through vuforia");
            //move based on previous stone capture
            robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(moveForwardToGatherFirstStone, 0, CAUTIOUS_SPEED, "Inch towards stone"));
        }
    }

    protected void queueSecondStoneGrab() {
        //wait for gripper to settle
        robot.queuePrimaryOperation(new WaitOperation(600, "Wait for gripper to stabilize"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_GRAB, "Lower arm to grab"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Grab second stone"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LIFT, "Lift above other stones"));
    }

    protected void queueSecondStoneDelivery() {
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CARRY, "Carry position"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Face towards foundation"));
        //deliver second skystone
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(
                lateralTravel,
                headingToFoundation, FAST_SPEED,
                "Move towards foundation"));
    }

    protected void queueUnderBridge() {
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-8 *Field.MM_PER_INCH, headingToFoundation, FAST_SPEED, "Navigate"));
        flash();
        flash();
        flash();
        flash();
    }
}
