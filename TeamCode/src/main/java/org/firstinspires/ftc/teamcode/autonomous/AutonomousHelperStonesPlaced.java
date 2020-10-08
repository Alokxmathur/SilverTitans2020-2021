package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.GyroscopicBearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceWithHeadingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitUntilVuMarkOperation;

import java.util.Locale;

public abstract class AutonomousHelperStonesPlaced extends AutonomousHelperStones {

    public void init(Alliance.Color allianceColor) {
        super.init(allianceColor);
    }

    @Override
    public void loop() {
        if (!initialMovementDone) {
            //initiate Phoebe
            if (!initialMovementQueued) {
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
                Match.log("Queueing first sky stone delivery");
                this.queueFirstDelivery();
                queuedFirstDelivery = true;
            }
            firstStonedDelivered = robot.operationsCompleted();
        }
        else if (!firstStonedPlaced) {
            if (!queuedFirstPlacement) {
                Match.log("Placing first sky stone");
                this.queueFirstPlacement();
                queuedFirstPlacement = true;
            }
            firstStonedPlaced = robot.operationsCompleted();
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

    protected void queueFirstDelivery() {
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.PLACE, "Place"));
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(1.6*Field.TILE_WIDTH, headingToFoundation, FAST_SPEED, "Get to foundation"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Face foundation"));
        //move another tile width and a half
        lateralTravel += 1.5*Field.TILE_WIDTH;
    }

    protected void queueFirstPlacement() {
        //robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Level"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Drop"));
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Hover"));
    }

    protected void queueReturnForSecondStone() {
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CARRY, "Carry position"));

        robot.queueTertiaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.LOWER, "Lower gripper so camera can see"));

        robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Realign to go back"));

        //calculate how much to return for second stone

        //we add the amount we strafed to clear the wall after picking first stone,
        // also add a buffer of 9 inches for the arm to clear the bridge
        lateralTravel += (Robot.LENGTH/2 + 9*Field.MM_PER_INCH);

        //we subtract 3*stone length since we are going back to the sky-stone that is closer to the bridge
        lateralTravel -= 3*Field.STONE_LENGTH;
        //if we came from the bottom stone, we weren't centered on it so add the extra distance to travel back
        if (match.getSkyStonePosition() == Field.SkyStonePosition.BOTTOM) {
            lateralTravel += (MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2 - Field.STONE_LENGTH/2);
        }
        //return to second stone
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-lateralTravel, headingToFoundation, FAST_SPEED, "Travel back"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Hover"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Face quarry for second stone"));
        //robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-8*Field.MM_PER_INCH, 0, CAUTIOUS_SPEED, "Away from quarry to see"));
        robot.queuePrimaryOperation(new WaitUntilVuMarkOperation(VUFORIA_SETTLING_TIME, "Give vuforia time to see"));
    }

    protected void queuePositionForSecondStone() {
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
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_GRAB, "Lower arm to grab"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CLOSE_GRIPPER, "Grab second stone"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Lift second stone"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Face towards foundation"));
    }

    protected void queueSecondStoneDelivery() {
        robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Level to clear bridge"));
        //deliver second skystone
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(
                lateralTravel,
                headingToFoundation, FAST_SPEED,
                "Move towards foundation"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(0, "Face foundation"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.PLACE, "Get stone to place position"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.SHOULDER_LEVEL, "Level position"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OPEN_GRIPPER, "Drop"));
        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.HOVER, "Hover to clear drop"));
    }

    protected void queueUnderBridge() {

        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.CARRY, "Compact gripper"));
        robot.queuePrimaryOperation(new GyroscopicBearingOperation(headingToFoundation, "Realign"));
        robot.queuePrimaryOperation(new DriveForDistanceInDirectionOperation(-Field.TILE_WIDTH, headingToFoundation, FAST_SPEED, "Navigate"));
        flash();
        flash();
        flash();
        flash();
    }
}
