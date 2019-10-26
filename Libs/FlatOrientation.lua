

FlatOrientationClass = {}
FlatOrientationClass.__index = FlatOrientationClass;


local function new(PHelper,core)
  local self = setmetatable({}, FlatOrientationClass)
  
  self.PHelper = PHelper
  self.core = core

  -- adjust parameters
  self.angleToleranceError = 0.0005 -- rads
  self.maxUpAcceleration = 10 --m/s^2
  self.maxDownAcceleration = self.maxUpAcceleration * -1 --m/s^2

  self.pitchRollSpeed = 5
  self.yawSpeed = 10
  self.pitchPID = pid.new(0.2, 0, 10)
  self.rollPID = pid.new(0.2, 0, 10)
  self.yawPID = pid.new(0.2, 0, 10)

  self.widgetInfo = {}

  
  return self
end

function FlatOrientationClass:widget()
   local widgetText = [[]];

  for k,v,item in pairs(self.widgetInfo) do
    widgetText = widgetText .. [[<div>]] ..k.. [[</div> <div><b>]] .. v .. [[</b></div>]]
  end
  return [[<div></div>]]
-- return [[ 
--           <div  style="width: 100%;background-color: #252f43;border-radius: 5px;font-size: 14px;"> VerticalPosition]] ..
--              widgetText ..
--          [[</div>
--       ]]
end   


function FlatOrientationClass:newStabilization(yaw)
    -- Axis
                local rollAmplitude = 30 --export
                local pitchAmplitude = 25 --export
                local yawSpeed = 1.5 --export
                local yawPower = 3 --export
                local brakeFactor = 1 --export
                local minBrakeAcceleration = 4 --export
                --local autoBrakeSpeed = 15 --export: auto brake when speed is below that value (in m/s), with no thrust

                  

                local worldVertical = vec3(self.core.getWorldVertical())
                local worldUp = vec3(self.core.getConstructWorldOrientationUp())
                local worldForward = vec3(self.core.getConstructWorldOrientationForward())
                local worldRight = vec3(self.core.getConstructWorldOrientationRight())
                local worldVelocity = vec3(self.core.getWorldVelocity())
                local worldVelocityDir = vec3(self.core.getWorldVelocity()):normalize()
                local worldAngularVelocity = vec3(self.core.getWorldAngularVelocity())
                local yawVelocity = worldAngularVelocity:dot(worldUp)

                -- Rotation
                local currentRoll = getRoll(worldVertical, worldForward, worldRight)
                local currentPitch = -math.asin(worldForward:dot(worldVertical)) * constants.rad2deg
                local targetRoll =   0 -- utils.clamp(rollInput * rollAmplitude, -rollAmplitude, rollAmplitude)
                local targetPitch =  0 --utils.clamp(pitchInput * pitchAmplitude, -pitchAmplitude, pitchAmplitude)
                self.rollPID:inject(targetRoll - currentRoll)
                self.pitchPID:inject(targetPitch - currentPitch)

                self.widgetInfo.YAW = yaw;

                local yawTargetVelocity = 0 -- -rollInput * yawSpeed
                local yawAcceleration = yawPower * (yawTargetVelocity - yawVelocity)
                 self.widgetInfo.YAW_ACC =yawAcceleration ;
                local angularAcceleration = self.rollPID:get() * worldForward + self.pitchPID:get() * worldRight + (yawAcceleration + yaw) * worldUp

                return angularAcceleration;
end



function FlatOrientationClass:composeAngularAcceleration()
    local targetUp = -1 * self.PHelper.physics.acceleration.gravity.vector
    local targetForward = self.PHelper.physics.velocity.tangential.vector
    local targetRight = targetForward:cross(targetUp)



    return vec3(0,0,0)
end


function FlatOrientationClass:getStabilizedTransformNotModified(yawModification )
    local targetUp = self.PHelper.physics.orientation.up
    local targetRight = self.PHelper.physics.orientation.right
    local targetForward = self.PHelper.physics.orientation.forward
    local worldVertical = self.PHelper.physics.position.worldVertical
    local angularAcceleration  = vec3();
  
    if (worldVertical:len() > epsilon) then -- worldVertical is null vector in space.
    
      -- stabilized roll

      local newUp = worldVertical:project_on_plane(targetForward):normalize_inplace()
      local stabAxis, stabAngleRad = getAxisAngleRad(targetUp, newUp, targetForward)
      self.rollPID:inject(stabAngleRad )
      local rollAngle = self.rollPID:get()
      targetUp = targetUp:rotate(rollAngle, stabAxis)
      targetRight = targetRight:rotate(rollAngle, stabAxis)
      targetForward = targetForward:rotate(rollAngle, stabAxis)

    
      -- stabilized pitch
      local newUp = worldVertical:project_on_plane(targetRight):normalize_inplace()
      stabAxis,stabAngleRad = getAxisAngleRad(targetUp, newUp, targetRight)
      self.pitchPID:inject(stabAngleRad )
      local pitchAngle = self.pitchPID:get()

      targetUp = targetUp:rotate(pitchAngle, stabAxis)
      targetRight = targetRight:rotate(pitchAngle, stabAxis)
      targetForward = targetForward:rotate(pitchAngle, stabAxis)



      -- self.widgetInfo.ROLL = rollAngle
      -- self.widgetInfo.PITCH = pitchAngle

      -- if (math.abs(pitchAngle) < 0.5 and math.abs(rollAngle) < 0.5) then
        local newForward =  targetForward:project_on_plane(targetUp):normalize_inplace()
        newForward =  newForward:rotate(yawModification * 1.2, self.PHelper.physics.orientation.up)
        stabAxis,stabAngleRad = getAxisAngleRad(targetForward, newForward, targetUp)
        self.yawPID:inject(stabAngleRad)
        local angle = self.yawPID:get()

        targetUp = targetUp:rotate(angle, stabAxis)
        targetRight = targetRight:rotate(angle, stabAxis)
        targetForward = targetForward:rotate(angle, stabAxis)
      -- else
        -- self.yawPID:reset()

      -- end

      angularAcceleration =  self.rollPID:get() * self.PHelper.physics.orientation.forward + self.pitchPID:get() * self.PHelper.physics.orientation.right + self.yawPID:get() *  self.PHelper.physics.orientation.up
    end

    return targetUp, targetRight, targetForward, angularAcceleration
end


function FlatOrientationClass:getStabilizedTransform(yawModification )
    local targetUp = self.PHelper.physics.orientation.up
    local targetRight = self.PHelper.physics.orientation.right
    local targetForward = self.PHelper.physics.orientation.forward
    local worldVertical = self.PHelper.physics.position.worldVertical
    local shipVerticalRelativeToSpeed = self.PHelper.physics.orientation.right:cross(self.PHelper.physics.velocity.world.vector)

    if (worldVertical:len() > epsilon) -- worldVertical is null vector in space.
    then
      -- stabilized roll

      local newUp = worldVertical:project_on_plane(targetForward):normalize_inplace()
      --newUp =  newUp:rotate(yawModification * 1.2, self.PHelper.physics.orientation.forward)
      local stabAxis, stabAngleRad = getAxisAngleRad(targetUp, newUp, targetForward)
      self.rollPID:inject(stabAngleRad )
      local rollAngle = self.rollPID:get()
      targetUp = targetUp:rotate(rollAngle, stabAxis)
      targetRight = targetRight:rotate(rollAngle, stabAxis)
      targetForward = targetForward:rotate(rollAngle, stabAxis)

    
      -- stabilized pitch
      local newUp = shipVerticalRelativeToSpeed:project_on_plane(targetRight):normalize_inplace()
      if (self.PHelper.physics.velocity.world.vector:len() < 10) then
        newUp = worldVertical:project_on_plane(targetRight):normalize_inplace()
      end
      stabAxis,stabAngleRad = getAxisAngleRad(targetUp, newUp, targetRight)
      self.pitchPID:inject(stabAngleRad )
      local pitchAngle = self.pitchPID:get()

      targetUp = targetUp:rotate(pitchAngle, stabAxis)
      targetRight = targetRight:rotate(pitchAngle, stabAxis)
      targetForward = targetForward:rotate(pitchAngle, stabAxis)



      -- self.widgetInfo.ROLL = rollAngle
      -- self.widgetInfo.PITCH = pitchAngle

      if (math.abs(pitchAngle) < 0.5 and math.abs(rollAngle) < 0.5 and  self.PHelper.physics.velocity.tangential.len > 10) then
        local newForward = self.PHelper.physics.velocity.tangential.vector:project_on_plane(targetUp):normalize_inplace()
        newForward =  newForward:rotate(yawModification * 1.2, self.PHelper.physics.orientation.up)
        stabAxis,stabAngleRad = getAxisAngleRad(targetForward, newForward, targetUp)
        self.yawPID:inject(stabAngleRad)
        local angle = self.yawPID:get()
         -- self.widgetInfo.YAW = angle
    

        targetUp = targetUp:rotate(angle, stabAxis)
        targetRight = targetRight:rotate(angle, stabAxis)
        targetForward = targetForward:rotate(angle, stabAxis)
      else
         self.yawPID:reset()
          -- self.widgetInfo.YAW = "NOT CALCULATED"
      end
    end

    -- targetUp = targetAngleUp
    -- targetRight = targetAngleRight
    -- targetForward = targetAngleForward
 
    return targetUp, targetRight, targetForward
end

function FlatOrientationClass:getStabilizedTransformVertical()
    local targetUp = self.PHelper.physics.orientation.up
    local targetRight = self.PHelper.physics.orientation.right
    local targetForward = self.PHelper.physics.orientation.forward
    local worldVertical = self.PHelper.physics.position.worldVertical

    if (worldVertical:len() > epsilon) -- worldVertical is null vector in space.
    then

      -- stabilized pitch
      local newForward = worldVertical:project_on_plane(targetRight):normalize_inplace()
      stabAxis,stabAngleRad = getAxisAngleRad(targetForward, newForward, targetRight)
      self.pitchPID:inject(stabAngleRad )
      local angle = self.pitchPID:get()
      targetUp = targetUp:rotate(angle, stabAxis)
      targetRight = targetRight:rotate(angle, stabAxis)
      targetForward = targetForward:rotate(angle, stabAxis)

      -- stabilized yaw
      local newForward = worldVertical:project_on_plane(targetUp):normalize_inplace()
      local stabAxis, stabAngleRad = getAxisAngleRad(targetForward, newForward, targetUp)
      self.rollPID:inject(stabAngleRad )
      local angle = self.rollPID:get()
      targetUp = targetUp:rotate(angle, stabAxis)
      targetRight = targetRight:rotate(angle, stabAxis)
      targetForward = targetForward:rotate(angle, stabAxis)
    
    end

    return targetUp, targetRight, targetForward
end

function FlatOrientationClass:composeControlledStabAngularAcceleration(isVertical,yawModification)
  -- compute the new target orientation, with possible auto correction
  local targetUp, targetRight, targetForward
  if yawModification == nil then yawModification = 0 end
  local res
  if (isVertical ~= nil and isVertical == true) then
    targetUp, targetRight, targetForward = self:getStabilizedTransformVertical()
  elseif isVertical == 0 then
    targetUp, targetRight, targetForward, angularAcceleration = self:getStabilizedTransformNotModified(yawModification)  
  else
    targetUp, targetRight, targetForward, angularAcceleration = self:getStabilizedTransform(yawModification)
  end
  



    local targetAxis, targetAngle = getAxisAngleRad(self.PHelper.physics.orientation.up, targetUp, targetRight)
    local rollPitchRotation = targetAngle * targetAxis

    local targetAxis, targetAngle = getAxisAngleRad(self.PHelper.physics.orientation.forward, targetForward, targetUp)
    local yawRotation = (targetAngle  )* targetAxis
     res = self.pitchRollSpeed * rollPitchRotation + self.yawSpeed * yawRotation


  return res
end




FlatOrientation = setmetatable(
    {
        new = new
    }, {
        __call = function(_, ...) return new(...) end
    }
)

return FlatOrientation