package frc.robot;

import java.util.ArrayList;

import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefBranch;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.ReefStalk;

public class ScoringChooser {
    
    public void setBranchScored(ReefBranch stalk) {
        stalk.scoredOn = true;
    }

    public boolean hasScoredFiveCoralOnLevel (int level) {
        // loops all of whatever level on each stalk and sees if its scored on and returns if 5 or more is scored
        int amountScored = 0;
        for (ReefStalk reefStalk : FieldElement.ALL_STALKS) {
            if (reefStalk.getBranch(level).scoredOn) {
                amountScored++;
            }
        }
        return amountScored >= 5;
    }

    public ReefBranch getHighestScoringAvailableBranch(ReefFace desiredFace, int priority) {
        // priorities goes from 1-3 so -1 to make it 0-2 for list
        // makes a list of priorities for the highest scoring branches and then loops through all of the stalks to see what has not been scored on
        ArrayList<ReefBranch> branchPriority = new ArrayList<>(); 
        priority--;
        int l3PlacementInList = 0;
        for (ReefBranch branch : desiredFace.getBranches()) {
            if (!branch.scoredOn) {
                int branchLevel =  branch.getLevel();
                // puts l4 at front of list, l2 at back of list, and when l4 gets placed it moves up the l3 placment in the list so priority goes l 4,3,2
                if (branchLevel == 4) {
                    branchPriority.add(0,branch);
                    l3PlacementInList++;
                } else if (branchLevel == 3) {
                    branchPriority.add(l3PlacementInList,branch);
                } else if (branchLevel == 2) {
                    branchPriority.add(branch);
                }
            }
        } 

        // if there is too many branches filled in the code than priority add a left L2 so it can return somthing
        if (branchPriority.size() - 1 < priority) {
            branchPriority.add(priority, desiredFace.getLeftStalk().getBranch(2));
        }

        return branchPriority.get(priority);
    }

    public ReefBranch getBranchForRP(ReefFace desiredFace, int[] rpLevelPriority, int priority) {
        ArrayList<ReefBranch> branchPriority = new ArrayList<>(); 
        priority--;
        // goes through each level priority and sees if the 5 it needs for the rp is filled and if not it addes all of the branch level to the priority
        for (int i = 0; i < rpLevelPriority.length; i++) {
            if (hasScoredFiveCoralOnLevel(rpLevelPriority[i])) {
                continue;
            }
            for (ReefBranch reefBranch : desiredFace.getBranches(rpLevelPriority[i])) {
                if (!reefBranch.scoredOn) {
                    branchPriority.add(reefBranch);
                }
            }
        }
        // if either got all of the l 4-2 for the rp of there is no places to score just call highest scoring thing
        if (priority > branchPriority.size() - 1) {
            return getHighestScoringAvailableBranch(desiredFace, priority);
        }

        return branchPriority.get(priority);
    }

    public ReefBranch getAutoSelectedBranch(ReefFace desiredFace, int priority, boolean goingForRP, int[] rpLevelPriority) {
        // priority is from 1 to whatever and goes down priority list based of how high number, rpLevel priority 2,3,4 add highest priority level first
        if (goingForRP) {
            return getBranchForRP(desiredFace, rpLevelPriority, priority);
        }
        return getHighestScoringAvailableBranch(desiredFace, priority);
    }
}
