

AutoPilotClass = {}
AutoPilotClass.__index = AutoPilotClass;

--{physicsHelper=physicsHelper,verticalPosition=verticalPosition,flatOrientation= flatOrientation}
local function new(tools,unit)
  local self = setmetatable({}, AutoPilotClass)
  
  self.PH = tools.physicsHelper
  self.verticalPosition = tools.verticalPosition
  self.flatOrientation = tools.flatOrientation

  self.unit = unit

  self.active = true
  -- adjust parameters
  self.altitude = 0
  self.thrust = 0
  self.brakes = vec3(0,0,0)
  self.verticalMode = false
  self.aileronAcceleration = vec3(0,0,0)
  self.isLanding = false
  self.widgetInfo = {} 
  self.onTarget  = false 
  
  self.orientationPID = pid.new(1, 0, 1000)
  self.positionForwardPID = pid.new(1,0,100)
  self.positionRightPID = pid.new(1,0,100)

  self.hoverDistance = 30
  self.flightAltitude = 100

  self.screenAnimation = 0

  return self
end

function AutoPilotClass:widget()
  local widgetText = [[]];
  if self.active then
    self.screenAnimation = math.min(self.screenAnimation + 3,200)
  
    for k,v,item in pairs(self.widgetInfo) do
      widgetText = widgetText .. [[<div>]] ..k.. [[</div> <div><b>]] .. v .. [[</b></div>]]
    end
  else
      self.screenAnimation = math.min(self.screenAnimation - 3,0)
  end



  return [[
          <div style="
          width:28vw;
          left:36vw;
          bottom:0px;
          transform-origin: 0 100%;
          transform: rotate3d(1,0,0,30deg);
          position:absolute;
          background-color: ]].. (function() if self.active then return "#252f43" else return "rgba(50,50,50,0.5)" end end)() .. [[;
          border-radius: 5px;
          height:]] .. self.screenAnimation .. [[px;
          max-height:]] .. self.screenAnimation .. [[px;
          overflow:hidden;

          font-size: 14px;
          "> IA Module]] .. 
             widgetText ..
         [[</div>


      ]]
end 
          -- opacity:]].. (function() if active then return 1 else return 0.2 end end)() .. [[

     -- verticalPosition:setAltitude(autoPilot:getAltitude())
                 

     --                self.unit.setEngineCommand('brake', {vec3(autoPilot:getBrakeAcceleration()):unpack()} , nullvector)
     --                self.unit.setEngineCommand('vertical', {vec3(autoPilot:getAileronAcceleration() + physicsHelper.physics.orientation.up * Pitch * 100):unpack()}  , nullvector)
     --                self.unit.setEngineCommand('altitude', {vec3(physicsHelper.physics.orientation.up * Pitch * 100):unpack()} ,  vec3(0,0,0))
              
     --                if autoPilot:isVertical() then
     --                    self.unit.setEngineCommand('torque', vec3(0,0,0), {flatOrientation:composeControlledStabAngularAcceleration(true):unpack()} ) 
     --                    self.unit.setEngineCommand('horizontal', {verticalPosition:composeHorizontalAcceleration():unpack()} ,  vec3(0,0,0))                    
     --                elseif (physicsHelper.physics.velocity.tangential.len > 20) then
     --                    self.unit.setEngineCommand('torque', vec3(0,0,0), {flatOrientation:composeControlledStabAngularAcceleration():unpack()} ) 
     --                    self.unit.setEngineCommand('horizontal', {vec3(autoPilot:getThrust() * physicsHelper.physics.orientation.forward * physicsHelper.physics.acceleration.potential.forward):unpack()} ,  vec3(0,0,0)) 
     --                end

function AutoPilotClass:takeControl()
      if (self.currentState == nil) then
        self.currentState = self.STATE_PositioningHorizontal
      end


      local verticalDistance =  self.PH.physics.position.targetDistanceToPlanetCenter -self.PH.physics.position.distanceToPlanetCenter 
      local horizontalDistanceVector = vec3(self.PH.physics.position.target - self.PH.physics.position.world):project_on_plane(self.PH.physics.position.worldVertical)
 
      -- if (horizontalDistanceVector:len() >0.1 or self.PH.physics.velocity.tangential.len > 0.1) then
      --   return self:STATE_PositioningWhileVertical(verticalDistance,horizontalDistanceVector )
      -- else
      --   return self:STATE_Landing(verticalDistance,horizontalDistanceVector )  
      -- end

      self:currentState(verticalDistance,horizontalDistanceVector)
      

              
                    -- local horizontalDistance = vec3(self.PH.physics.position.target - self.PH.physics.position.world):project_on_plane(self.PH.physics.position.worldVertical):len()
                    -- local verticalDistance = vec3(self.PH.physics.position.target - self.PH.physics.position.world):dot(self.PH.physics.position.worldVertical)
                    -- self.widgetInfo.verticalDistance = verticalDistance
                    -- self.widgetInfo.thrust =  [[<div style="width:200px;height: 10px;background-color:grey;border:1px solid white"><div style="width:]] .. self.thrust * 200 .. [[px;height: 5px;background-color:orange"></div</div>]]
                    -- self.widgetInfo.brakes =  [[<div style="width:200px;height: 10px;background-color:grey;border:1px solid white"><div style="width:]] .. self.brakes:len() .. [[px;height: 5px;background-color:black"></div</div>]]
                        
                    --      if self.onTarget then 
                    --       self.widgetInfo.onTarget  = "TRUE" 
                    --     else
                    --       self.widgetInfo.onTarget  = "FALSE" 
                    --     end

                    -- if ( self.onTarget ) then
                    --     self.thrust = 0
                    --     self.verticalMode = true
                    --     self.aileronAcceleration = vec3(0,0,0)
                    --     self.brakes = vec3(0,0,0)
                    --     self.altitude = self.PH.physics.position.altitude -10

                    -- elseif ( horizontalDistance > 300) then
                    -- -----------------------------------Aproximation-------------------------------------------------------------
                    --   self.widgetInfo.state = [[<div style="width:40px;border:2px solid white;background-color:white">Aproximation</div>]]
                        
                    --     self.isLanding = false
                          
                    --     self.altitude = self.PH.physics.position.altitude + verticalDistance +50 
                    --     if((verticalDistance + 50) / horizontalDistance > 0.05) then
                    --         self.verticalMode = true
                    --         self.thrust = 0
                    --     else
                    --         self.verticalMode = false
                    --         self.thrust = 0.8
                    --         self.aileronAcceleration = physicsHelper.physics.orientation.right * turnFactor  * 100
                             
                            
                    --     end
                    --       self.brakes = vec3(0,0,0)
                    -- --------------------------------------------------------------------------------------------------
                    -- elseif horizontalDistance > 30 then
                    -- ---------------------------------Deceleration-----------------------------------------------------------------
                    --     self.widgetInfo.state = [[<div style="width:80px;border:2px solid grey;background-color:grey">Deceleration</div>]]
                    --     self.thrust = 0.1
                    --     self.isLanding = false
                          
                    --     self.altitude = self.PH.physics.position.altitude + verticalDistance - 5
                    --     self.aileronAcceleration = physicsHelper.physics.orientation.right * turnFactor  * 100
                    --     self.verticalMode = false

                    --     if self.PH.physics.velocity.tangential.len > 40 then
                    --       self.brakes = -0.2 * self.PH.physics.velocity.world.vector
                    --     else
                    --       self.brakes = vec3(0,0,0)
                    --     end
                    

                    -- --------------------------------------------------------------------------------------------------
                    -- elseif horizontalDistance > 3 and self.isLanding == false then
                    -- ---------------------------------Positioning------------------------------------------------------
                    --     self.widgetInfo.state = [[<div style="width:120px;border:2px solid purple;background-color:purple">Positioning</div>]]
                    --     self.thrust = 0.00
                    --     --  self.verticalMode = false
                    --     self.aileronAcceleration = vec3(0,0,0)
                    --     if self.PH.physics.velocity.tangential.len > 2 then
                    --       self.brakes = -1 * self.PH.physics.velocity.world.vector
                    --     else
                    --       self.brakes = vec3(0,0,0)
                    --     end
                    -- --------------------------------------------------------------------------------------------------
                    -- else
                    -- ---------------------------------vertical Landing-------------------------------------------------------
                    --     self.widgetInfo.state = [[<div style="width:160px;border:2px solid red;background-color:red">Vertival Landing</div>]]
                    --     if self.onTarget then 
                    --       self.widgetInfo.onTarget  = "TRUE" 
                    --     else
                    --       self.widgetInfo.onTarget  = "FALSE" 
                        
                    --     end

                    --     self.thrust = 0.00
                    --     self.verticalMode = true
                    --     self.aileronAcceleration = vec3(0,0,0)
                    --     self.isLanding = true
                    --     self.widgetInfo.tangentialSPD = self.PH.physics.velocity.tangential.len
                    --     if self.PH.physics.velocity.tangential.len > 0.05 then
                    --       self.brakes = -5  * self.PH.physics.velocity.world.vector
                          
                    --     else
                    --       self.verticalMode = true
                    --       if verticalDistance > 40 then
                    --         self.brakes = vec3(0,0,0)  
                    --       elseif verticalDistance > 10  then
                    --         self.brakes = -0.2 * self.PH.physics.velocity.world.vector
                    --       elseif (self.PH.physics.velocity.vertical.vector:len() > 0.5) then
                    --         self.brakes =  -5 * self.PH.physics.velocity.world.vector
                    --       else
                    --         self.onTarget = true
                    --       end
                    --     end
                    -- --------------------------------------------------------------------------------------------------
                    -- end
end 



function AutoPilotClass:STATE_PositioningHorizontal(verticalDistance, horizontalDistanceVector)
  self.widgetInfo.state = [[<div style="width:50%;border:2px solid white;background-color:#229ac8;position:absolute;top 0px;right:0px;text-align:right">PositioningWhileVertical</div>]]
  self.widgetInfo.restrictionState = round(horizontalDistanceVector:len()) .. [[<0.1  and ]] .. round(self.PH.physics.velocity.tangential.len)..[[ < 0.1]]  
  

  -- if (horizontalDistanceVector:len() <0.1  and self.PH.physics.velocity.tangential.len < 0.1) then
  --   self.currentState = self.STATE_Landing 
  --   return self:currentState(verticalDistance, horizontalDistanceVector)
  -- elseif math.abs(verticalDistance + self.hoverDistance ) > 2 then
  --   self.currentState = self.STATE_VerticalStabilization
  --   return self:currentState(verticalDistance, horizontalDistanceVector)
  -- end

  -- self.verticalPosition:setAltitude(self.PH.physics.position.altitude + verticalDistance + self.hoverDistance )

  self.positionForwardPID:inject(horizontalDistanceVector:dot(self.PH.physics.orientation.forward)) --
  self.positionRightPID:inject(horizontalDistanceVector:dot(self.PH.physics.orientation.right )) --
  
  self.widgetInfo.LATERAL = self.positionRightPID:get()
  self.widgetInfo.FRONTAL = self.positionForwardPID:get()

  -- local mainEnginesAcceleration = self.verticalPosition:composeHorizontalAcceleration()     
  local horizontalAceleration =   self.positionRightPID:get() * self.PH.physics.orientation.right +  self.positionForwardPID:get() * self.PH.physics.orientation.forward       
  
  

  -- if horizontalDistanceVector:normalize():dot(self.PH.physics.velocity.tangential.vector:normalize()) < 0.9 and self.PH.physics.velocity.tangential.len > 0.1 then
  --   self.unit.setEngineCommand('brake',{(-10 * self.PH.physics.velocity.tangential.vector):unpack()}, nullvector)
  -- else
  --   self.unit.setEngineCommand('brake',{(100 * horizontalDistanceVector:normalize()):unpack()}, nullvector)
  -- end
  
  self.unit.setEngineCommand('horizontal,brake', {horizontalAceleration:unpack()} ,  vec3(0,0,0))      
  -- self.unit.setEngineCommand('vertical', nullvector, nullvector)
  -- self.unit.setEngineCommand('altitude', nullvector , nullvector)
  -- self.unit.setEngineCommand('torque', vec3(0,0,0), {flatOrientation:composeControlledStabAngularAcceleration(true):unpack()} ) 
 
end

function AutoPilotClass:useAileronsToTurnTowardsTarget()
    self.orientationPID:inject(-1 * self.PH.physics.orientation.targetAngle)
    turnFactor = self.orientationPID:get()
    self.unit.setEngineCommand('vertical',  {vec3(self.PH.physics.orientation.right * turnFactor  * 100):unpack()}, nullvector)
end 

function AutoPilotClass:STATE_FastCarelessAproximation(verticalDistance, horizontalDistanceVector)
 self.widgetInfo.state = [[<div style="width:50%;border:2px solid white;background-color:red;position:absolute;top 0px;right:0px;text-align:right">FastCarelessAproximation</div>]]
 self.widgetInfo.restrictionState = round(horizontalDistanceVector:len())..[[ <20 ]]

 if horizontalDistanceVector:len() < 150 then
  self.currentState = self.STATE_Deceleraion
  return self:currentState(verticalDistance, horizontalDistanceVector)
 end
 self.widgetInfo.verticalDistance = verticalDistance
 self.verticalPosition:setAltitude(self.PH.physics.position.altitude + verticalDistance + self.flightAltitude )
 
    self.orientationPID:inject(-1 * self.PH.physics.orientation.targetAngle)
    turnFactor = self.orientationPID:get()
    

  self.unit.setEngineCommand('horizontal', {(self.PH.physics.acceleration.potential.forward * self.PH.physics.orientation.forward *2 ):unpack()} ,  vec3(0,0,0))      
  self.unit.setEngineCommand('brake',{vec3(0,0,0):unpack()} , nullvector)
  self.unit.setEngineCommand('vertical', {vec3( physicsHelper.physics.orientation.right * turnFactor  * 100):unpack()}, nullvector)
  self.unit.setEngineCommand('altitude',  {self.verticalPosition:composeVerticalAcceleration():unpack()}, nullvector)
  self.unit.setEngineCommand('torque', vec3(0,0,0), {self.flatOrientation:composeControlledStabAngularAcceleration(false):unpack()} ) 
 
end

function AutoPilotClass:STATE_ControledAproximation(erticalDistance, horizontalDistanceVector)

end

function AutoPilotClass:STATE_Deceleraion(verticalDistance, horizontalDistanceVector)
 self.widgetInfo.state = [[<div style="width:50%;border:2px solid white;background-color:purple;position:absolute;top 0px;right:0px;text-align:right">Deceleration</div>]]
 self.widgetInfo.restrictionState = round(self.PH.physics.velocity.tangential.len)..[[ <0.1 ]]

 if self.PH.physics.velocity.tangential.len < 0.1 then
  self.currentState = self.STATE_VerticalStabilization
  return self:currentState(verticalDistance, horizontalDistanceVector)
 end

 self.verticalPosition:setAltitude(self.PH.physics.position.altitude + verticalDistance +self.flightAltitude )
  

  self.unit.setEngineCommand('horizontal', {vec3(0,0,0):unpack()} ,  vec3(0,0,0))      
  self.unit.setEngineCommand('brake',{(self.PH.physics.velocity.world.vector * -10):unpack()} , nullvector)
  self.unit.setEngineCommand('vertical', {vec3(0,0,0):unpack()}, nullvector)
  self.unit.setEngineCommand('altitude',  {self.verticalPosition:composeVerticalAcceleration():unpack()}, nullvector)
  self.unit.setEngineCommand('torque', vec3(0,0,0), {self.flatOrientation:composeControlledStabAngularAcceleration(false):unpack()} ) 
 
end

function AutoPilotClass:STATE_VerticalStabilization(verticalDistance, horizontalDistanceVector)
  self.widgetInfo.state = [[<div style="width:50%;border:2px solid white;background-color:#6ec494;position:absolute;top 0px;right:0px;text-align:right">VerticalStabilization</div>]]
  self.widgetInfo.restrictionState = round(math.abs( verticalDistance +self.hoverDistance ))..[[ <0.5  and ]]..round(math.abs(self.PH.physics.velocity.vertical.magnitude)) .. [[ < 0.1 ]]

  if (math.abs( verticalDistance +self.hoverDistance ) <0.5  and math.abs(self.PH.physics.velocity.vertical.magnitude) < 0.1) then
    self.currentState = self.STATE_PositioningWhileVertical
    return self:currentState(verticalDistance, horizontalDistanceVector)
  end

  self.positionForwardPID:inject(horizontalDistanceVector:dot(self.PH.physics.orientation.up)) --
  self.positionRightPID:inject(horizontalDistanceVector:dot(self.PH.physics.orientation.right )) --
  
  self.widgetInfo.LATERAL = self.positionRightPID:get()
  self.widgetInfo.FRONTAL = self.positionForwardPID:get()

  self.verticalPosition:setAltitude(self.PH.physics.position.altitude + verticalDistance +self.hoverDistance )
  local mainEnginesAcceleration = self.verticalPosition:composeHorizontalAcceleration()      


 local horizontalAceleration =   self.positionRightPID:get() * self.PH.physics.orientation.right +  self.positionForwardPID:get() * self.PH.physics.orientation.up       
  if (horizontalAceleration:len() > horizontalAceleration:len() / 16) then --laterla acceleration bigger than the amount we can handle without going up
    horizontalAceleration = horizontalAceleration:normalize_inplace() *  horizontalAceleration:len() / 16
  end
  mainEnginesAcceleration = mainEnginesAcceleration + horizontalAceleration  


  local brakingIntensity = utils.clamp((50 - verticalDistance )/ 50, 0, 5)
  self.widgetInfo.brakingIntensity = brakingIntensity
  
  if (self.PH.physics.orientation.forward:dot(self.PH.physics.position.worldVertical) > 0.95) then
    self.unit.setEngineCommand('horizontal', {mainEnginesAcceleration:unpack()} ,  vec3(0,0,0))      
  else
    self.unit.setEngineCommand('horizontal', {vec3(0,0,0):unpack()} ,  vec3(0,0,0))      
  
  end
  
  self.unit.setEngineCommand('brake',{vec3(-1 * brakingIntensity * self.PH.physics.velocity.world.vector):unpack()} , nullvector)
  self.unit.setEngineCommand('vertical', nullvector, nullvector)
  self.unit.setEngineCommand('altitude', nullvector , nullvector)
  self.unit.setEngineCommand('torque', vec3(0,0,0), {flatOrientation:composeControlledStabAngularAcceleration(true):unpack()} ) 
 
end


function AutoPilotClass:STATE_PositioningWhileVertical(verticalDistance, horizontalDistanceVector)
  self.widgetInfo.state = [[<div style="width:50%;border:2px solid white;background-color:#229ac8;position:absolute;top 0px;right:0px;text-align:right">PositioningWhileVertical</div>]]
  self.widgetInfo.restrictionState = round(horizontalDistanceVector:len()) .. [[<0.1  and ]] .. round(self.PH.physics.velocity.tangential.len)..[[ < 0.1]]  
  
  if (horizontalDistanceVector:len() <0.1  and self.PH.physics.velocity.tangential.len < 0.1) then
    self.currentState = self.STATE_Landing 
    return self:currentState(verticalDistance, horizontalDistanceVector)
  elseif math.abs(verticalDistance + self.hoverDistance ) > 2 then
    self.currentState = self.STATE_VerticalStabilization
    return self:currentState(verticalDistance, horizontalDistanceVector)
  end

  self.verticalPosition:setAltitude(self.PH.physics.position.altitude + verticalDistance + self.hoverDistance )

  self.positionForwardPID:inject(horizontalDistanceVector:dot(self.PH.physics.orientation.up)) --
  self.positionRightPID:inject(horizontalDistanceVector:dot(self.PH.physics.orientation.right )) --
  
  self.widgetInfo.LATERAL = self.positionRightPID:get()
  self.widgetInfo.FRONTAL = self.positionForwardPID:get()

  local mainEnginesAcceleration = self.verticalPosition:composeHorizontalAcceleration()     
  local horizontalAceleration =   self.positionRightPID:get() * self.PH.physics.orientation.right +  self.positionForwardPID:get() * self.PH.physics.orientation.up       
  if (horizontalAceleration:len() > horizontalAceleration:len() / 16) then --laterla acceleration bigger than the amount we can handle without going up
    horizontalAceleration = horizontalAceleration:normalize_inplace() *  horizontalAceleration:len() / 16
  end
  mainEnginesAcceleration = mainEnginesAcceleration + horizontalAceleration  


  if horizontalDistanceVector:normalize():dot(self.PH.physics.velocity.tangential.vector:normalize()) < 0.9 and self.PH.physics.velocity.tangential.len > 0.1 then
    self.unit.setEngineCommand('brake',{(-10 * self.PH.physics.velocity.tangential.vector):unpack()}, nullvector)
  else
    self.unit.setEngineCommand('brake',{(100 * horizontalDistanceVector:normalize()):unpack()}, nullvector)
  end
  
  self.unit.setEngineCommand('horizontal', {mainEnginesAcceleration:unpack()} ,  vec3(0,0,0))      
  self.unit.setEngineCommand('vertical', nullvector, nullvector)
  self.unit.setEngineCommand('altitude', nullvector , nullvector)
  self.unit.setEngineCommand('torque', vec3(0,0,0), {flatOrientation:composeControlledStabAngularAcceleration(true):unpack()} ) 
 
end

-- Standing over the target with small error in the horizontal plane (less than 10 meters)
function AutoPilotClass:STATE_Landing(verticalDistance, horizontalDistanceVector)
 
  local landingDistance = 0
  self.widgetInfo.state = [[<div style="width:50%;border:2px solid white;background-color:6ac822;position:absolute;top 0px;right:0px;text-align:right">Landing</div>]]
  self.widgetInfo.restrictionState = round(math.abs(verticalDistance))..[[ < ]]..round(landingDistance).. [[ and ]] .. round(math.abs(self.PH.physics.velocity.vertical.magnitude)) .. [[ < 0.1]]  
 
  if (math.abs(verticalDistance) < landingDistance + 0.5  and math.abs(self.PH.physics.velocity.vertical.magnitude) < 0.1) then
    self.currentState = self.STATE_Landed
    return self:currentState(verticalDistance, horizontalDistanceVector)
  end
  --we want the ship to freefall until close enougth to the floor, that is taken care by the verticalposition module
  --we want also to stop the ship over the target, we will use 2 pids to achieve it nd use the inclined engines to hover to that situation
  self.verticalPosition:setAltitude(self.PH.physics.position.altitude + verticalDistance + landingDistance )
  
  local mainEnginesAcceleration = self.verticalPosition:composeHorizontalAcceleration()              
  self.unit.setEngineCommand('horizontal', {mainEnginesAcceleration:unpack()} ,  vec3(0,0,0))      

  -- the brakes should increase when distance is closer help faster slow down on the landing
  local brakingIntensity = utils.clamp( 5 - math.abs(verticalDistance) , 0, 10)
  self.widgetInfo.brakingIntensity = brakingIntensity
    
  self.unit.setEngineCommand('brake',{vec3(-1 * brakingIntensity * self.PH.physics.velocity.world.vector):unpack()} , nullvector)

  --ailerons require no force here
  self.unit.setEngineCommand('vertical', nullvector, nullvector)
 
  --altitude Aileron is irrelevant
  self.unit.setEngineCommand('altitude', nullvector , nullvector)

  --torque must kkep the ship vertical
  self.unit.setEngineCommand('torque', vec3(0,0,0), {flatOrientation:composeControlledStabAngularAcceleration(true):unpack()} ) 
 

end

function AutoPilotClass:STATE_Landed()
  self.widgetInfo.state = [[<div style="width:50%;border:2px solid white;background-color:green;position:absolute;top 0px;right:0px;text-align:right">Landed</div>]]
  self.unit.setEngineCommand('horizontal', nullvector ,nullvector)
  self.unit.setEngineCommand('brake',{vec3(-1 * 15 * self.PH.physics.velocity.world.vector):unpack()} , nullvector)
  self.unit.setEngineCommand('vertical', nullvector, nullvector)
  self.unit.setEngineCommand('altitude', nullvector , nullvector)
  --torque must kkep the ship vertical
  self.unit.setEngineCommand('torque', vec3(0,0,0), {flatOrientation:composeControlledStabAngularAcceleration(true):unpack()} ) 
end


function AutoPilotClass:setStatus(newStatus)
  self.active = newStatus == 1
end

function AutoPilotClass:isActive()
  return self.active
end

function AutoPilotClass:getAltitude()
  return self.altitude
end

function AutoPilotClass:getThrust()
  return self.thrust
end

function AutoPilotClass:getBrakeAcceleration()
  return self.brakes
end

function AutoPilotClass:getAileronAcceleration()
  return self.aileronAcceleration
end

function AutoPilotClass:isVertical()
  return self.verticalMode
end


AutoPilot = setmetatable(
    {
        new = new
    }, {
        __call = function(_, ...) return new(...) end
    }
)

return AutoPilot