package org.firstinspires.ftc.teamcode.actions;

/**
 * Abstract base class for path following update actions. Use this for tasks that require updates to be ran parallely to parrot pursuit
 */
public abstract class UpdateAction {
    /**
     * Called when the update action is triggered.
     */
    public abstract void onUpdate();
}
