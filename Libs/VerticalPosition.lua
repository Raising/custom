VerticalPositionClass = {}
VerticalPositionClass.__index = VerticalPositionClass;

function round(decimal,precision)
  if (precision == nil) then
    precision = 2
  end
  local decimalDisplacer = 10^precision

  return math.floor(decimal *decimalDisplacer) / decimalDisplacer
end


local function new(PHelper)
  local self = setmetatable({}, VerticalPositionClass)
  
  self.PHelper = PHelper
  self.altitudePID = pid.new(1, 0, 100)
  self.landingPID = pid.new(5,0,2000)
  self.pitchPID = pid.new(100,0,1000)

  -- adjust parameters
  self.altitudeToleranceError = 0.5
  self.maxUpAcceleration = 50 --m/s^2
  self.maxDownAcceleration = self.maxUpAcceleration * -1 --m/s^2

  -- self.baseAltitude = self.PHelper.physics.position.altitude
  self.targetAltitude =  self.PHelper.physics.position.altitude
  self.targetAltitudeStep = 20

  self.widgetInfo = {}
  self.widgetInfo.altitudeError =0
  self.widgetInfo.operation = "--"
  return self
end

function VerticalPositionClass:resetAltitude()
  self.targetAltitude =  self.PHelper.physics.position.altitude
end


function VerticalPositionClass:setAltitude(newAltitude)
  self.targetAltitude =  newAltitude
end

function VerticalPositionClass:widget()
   local widgetText = [[]];

    for k,v,item in pairs(self.widgetInfo) do
      widgetText = widgetText .. [[<div>]] ..k.. [[</div> <div><b>]] .. v .. [[</b></div>]]
    end
    return [[
              <div style="width: 100%;background-color: #252f43;border-radius: 5px;font-size: 14px;"> VerticalPosition]] ..
                 widgetText ..
             [[</div>
          ]]
    
  -- return [[
  --           <div style="width:15vw;
  --           transform-origin: 0 100%;
  --           transform: rotate3d(1,0,0,30deg);
  --           position:absolute;
  --           bottom:0px;left:0px;
  --           background-color: #252f43;
  --           border-radius: 5px;
  --           height:200px;
  --           font-size: 14px;"> VerticalPosition ]] .. self.landingPID:get() ..
  --             --target Line  
  --             [[<div style="position:absolute; bottom: 100px;right:60px;background:]] ..(function() if math.abs(self.widgetInfo.altitudeError) < 1 then return 'green' else return 'orange' end end)()  .. [[;height:2px;width:100px">]] .. [[</div>]] ..  
  --             [[<div style="position:absolute; bottom: 93px;right:2px;text-align:right;font-size:12px;height:14px;width:30%">]] .. round(self.targetAltitude,1) .. [[m</div>]] ..  
  --             -- ship Position
  --             [[<div style="position:absolute; bottom: ]].. round(98 -  self.widgetInfo.altitudeError / 20,0) .. [[px;right:90px;background:black;height:4px;width:20px;border-radius:0 2px 2px 0">]] .. [[</div>]] ..  
  --             [[<div style="position:absolute; bottom: ]].. round(93 -  self.widgetInfo.altitudeError / 20,0) .. [[px;right:120px;text-align:right;color:white;height:16px;width:50px;font-size:12px">]] .. round(self.widgetInfo.altitudeError).. [[m</div>]] ..  

  --              self.widgetInfo.operation ..
  --              -- widgetText ..
  --          [[</div>
  --       ]]
end  

function VerticalPositionClass:land(currentDistance)
    self.landingPID:inject(currentDistance)


    return self.PHelper.physics.orientation.up *  (self.landingPID:get() + self.PHelper.physics.acceleration.gravity.len)
end

function VerticalPositionClass:composeHorizontalAcceleration()
  return self.PHelper.physics.orientation.forward * self:optimalAceleration(true)
end


function VerticalPositionClass:composeVerticalPitchAcceleration(pitch)
  if pitch == 0 then
    local pitchStabilization = self:pitchAceleration(pitch)
    if pitchStabilization > 1 then self:setAltitude( self.PHelper.physics.position.altitude) end
    return self.PHelper.physics.orientation.up * (self:pitchAceleration(pitch) + self:optimalAceleration(false)) 
  else
    return self.PHelper.physics.orientation.up * self:pitchAceleration(pitch)
  end
end




function VerticalPositionClass:composeWorldVerticalAcceleration()
    self.altitudePID:inject(self:getVerticalDistanceError())
    self.widgetInfo.PIDacc = self.altitudePID:get() + self.PHelper.physics.acceleration.gravity.len
    return self.PHelper.physics.position.worldVertical:normalize() * ( self.altitudePID:get() + self.PHelper.physics.acceleration.gravity.len)
end

function VerticalPositionClass:composeVerticalAcceleration()
    return self.PHelper.physics.orientation.up * self:optimalAceleration(false)
end

function VerticalPositionClass:optimalAceleration(horizontal)
  if (math.abs(self.PHelper.physics.velocity.vertical.magnitude) < 1 and math.abs(self:getVerticalDistanceError()) <= self.altitudeToleranceError) then
    return self:stabilice()
  else
    return self:PIDcontroledAltitude()
 --   return self:fastestContorledAproach(horizontal)
  end
end


function VerticalPositionClass:pitchAceleration(pitch)
  local currentAngle = self.PHelper.physics.orientation.forward:angle_between(self.PHelper.physics.position.worldVertical) - (constants.deg2rad * 90)
    self.widgetInfo.CURRENTANGLE = currentAngle
    self.widgetInfo.ANGLE_ERROR = currentAngle + pitch
    self.widgetInfo.PITCHOUTPUT = self.pitchPID:get() 
  self.pitchPID:inject(currentAngle + pitch)
  return self.pitchPID:get() -- / self.PHelper.physics.orientation.up:dot(self.PHelper.physics.position.worldVertical:normalize())
  
end

function VerticalPositionClass:PIDcontroledAltitude()
    self.altitudePID:inject(self:getVerticalDistanceError())
    return self.altitudePID:get() / self.PHelper.physics.orientation.up:dot(self.PHelper.physics.position.worldVertical:normalize())
end


function VerticalPositionClass:stabilice()
  self.widgetInfo.stable = "true"

  return (self.PHelper.physics.acceleration.gravity.len) * (1 - self.PHelper.physics.velocity.vertical.magnitude )
end


function VerticalPositionClass:fastestContorledAproach(horizontal)
  self.widgetInfo.stable = "false"
  local distanceError = self:getVerticalDistanceError()

  -- get the required lineal deceleration 
  local timeRequiredForLinealDeceleration = math.abs( 2* (distanceError / (self.PHelper.physics.velocity.vertical.magnitude or epsilon)))
  local linealDecelerationRequired  = math.abs(self.PHelper.physics.velocity.vertical.magnitude / timeRequiredForLinealDeceleration)
  
  -- self.widgetInfo.stableAcc = (self.PHelper.physics.acceleration.centrifugal.len - self.PHelper.physics.acceleration.gravity.len) * (1 + self.PHelper.physics.velocity.vertical.magnitude)
  -- self.widgetInfo.speed = self.PHelper.physics.velocity.vertical.magnitude
  -- self.widgetInfo.timeDeceleration = timeRequiredForLinealDeceleration

  -- self.widgetInfo.radius = self.PHelper.physics.position.radius
  -- self.widgetInfo.centri = self.PHelper.physics.acceleration.centrifugal.len
  
  -- self.widgetInfo.decelerationRequired = linealDecelerationRequired
  -- accelerate at max thrust unless the lineal deceleration required is begger than the one we can achieve

  if (distanceError > 0 ) then
    if (self.PHelper.physics.velocity.vertical.magnitude < 0 or linealDecelerationRequired < self.PHelper.physics.acceleration.potential.down ) then
      self.widgetInfo.operation = '->____|_____'
      return self.maxUpAcceleration
    else
      self.widgetInfo.operation =  '___<-|______'
      return  self.PHelper.physics.velocity.vertical.magnitude  * -2
    end
  else
    local maxAcceleration =  self.PHelper.physics.acceleration.potential.up 
    if horizontal == true then 
      maxAcceleration  = self.PHelper.physics.acceleration.potential.forward 
     end 
    -- self.widgetInfo.maxAcc = maxAcceleration
    -- self.widgetInfo.VSPD = self.PHelper.physics.velocity.vertical.magnitude
  
    --  it is considered that the ship is able to accelerate at least 1 m/s2, since when the engines turn on the max acceleration is increased
    if (self.PHelper.physics.velocity.vertical.magnitude > 0 or linealDecelerationRequired < maxAcceleration ) then
      self.widgetInfo.operation =  '_____|____<-'
      return self.maxDownAcceleration
    else
      self.widgetInfo.operation =  '_____|->____'
      return self.PHelper.physics.velocity.vertical.magnitude * 2
    end
  end
end


function VerticalPositionClass:getVerticalDistanceError()
    self.widgetInfo.altitudeError = self.targetAltitude - self.PHelper.physics.position.altitude
    -- self.widgetInfo.targetAltitude = self.targetAltitude
    -- self.widgetInfo.currentAltitude = self.PHelper.physics.position.altitude + self.baseAltitude
    
    return self.targetAltitude  - self.PHelper.physics.position.altitude 
end


function VerticalPositionClass:increaseAltitude()
    self.targetAltitude = self.targetAltitude  + self.targetAltitudeStep
end

function VerticalPositionClass:setAltitude(altitude)
    self.targetAltitude = altitude
end


function VerticalPositionClass:decreaseAltitude()
    self.targetAltitude = self.targetAltitude - self.targetAltitudeStep
end

VerticalPosition = setmetatable(
    {
        new = new
    }, {
        __call = function(_, ...) return new(...) end
    }
)

return VerticalPosition