
--up should be world up
--
function FlatOrientationClass:getStabilizedTransform(objectiveForward )
    local currentUp = self.PHelper.physics.orientation.up
    local currentRight = self.PHelper.physics.orientation.right
    local currentForward = self.PHelper.physics.orientation.forward

    local worldVertical = self.PHelper.physics.position.worldVertical
    local shipVerticalRelativeToSpeed = self.PHelper.physics.orientation.right:cross(self.PHelper.physics.velocity.world.vector)

    if (objectiveForward == nil) then
      objectiveForward = self.PHelper.physics.velocity.world.vector 
    end
    local objectiveUp = self.PHelper.physics.orientation.right:cross(forward)

    -- TODO replace getAngleDeg for :angle_between() *  constants.rad2deg
    if (worldVertical:len() > epsilon) -- worldVertical is null vector in space.
    then

        local targetRollUp = worldVertical:project_on_plane(currentForward):normalize_inplace()
        local rollDiffAngle =  getAngleDeg(targetRollUp,currentUp)
        self.rollPID:inject(rollDiffAngle)
        
        local targetPitchUp = objectiveUp:project_on_plane(currentRight):normalize_inplace()
        local pitchDiffAngle =  getAngleDeg(targetPitchUp,currentUp)
        self.pitchPID:inject(pitchDiffAngle)
        
        local targetYawForward = objectiveForward:project_on_plane(currentUp):normalize_inplace()
        local yawDiffAngle =  getAngleDeg(targetYawForward,currentForward )
        self.yawPID:inject(yawDiffAngle)
        
        return currentForward * self.rollPID:get() + currentRight * self.pitchPID:get() + currentUp * self.yawPID:get()

    else
        return Vector3.zero
    end
end

function getAngleDeg(oldDir, newDir)
    local axis = oldDir:cross(newDir)
    local axisLen = axis:len()
    local angle = 0
    axis = axis:normalize_inplace()
    if (axisLen > constants.epsilon)
    then
        angle = math.asin(utils.clamp(axisLen, 0, 1))
    else
        if oldDir:dot(newDir) < 0
        then --half turn
            angle = 180 * constants.deg2rad
        end
    end
    return angle * constants.rad2deg
end