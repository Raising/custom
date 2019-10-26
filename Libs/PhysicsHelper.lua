mat4      = require("cpml/mat4")

-- #####################################################

PlanetsDatabase = {
 Alioth =  { position = vec3(0,0,0), name = "Alioth"},
 Jago = { position = vec3(-94134464.000, 12765536.000,-3634464.000),name = "Jago"},
 Lacobus = { position = vec3(98865536.000,-13534464.000, -934464.000), name = "Lacobus"},
 Symeon = { position = vec3(14165536.000, -85634464.000, -934464.000), name = "Simeon"},
}



-- ######################################################

PhysicsHelperClass = {}
PhysicsHelperClass.__index = PhysicsHelperClass;

-- engines = {vertical, aileronV, aileronH, amountVertical, amountAileronV, amountAileronH

local function new( core, control, mass, engines)
  local self = setmetatable({}, PhysicsHelperClass)

  self.control = control
  self.core = core
  self.mass = mass
  self.engines = engines
  self.widgetInfo = {}

  self.physics = nil
  self.currentPlanet = nil
  self.destination = vec3(0,0,0)

  self:setCurrentPlanet()
  self:SetOnPlanetDestination(PlanetsDatabase.Symeon.position)
  self:calculate()

  return self
end

function PhysicsHelperClass:worldToShip(worldPosition)
    local worldToShipTraslationTransform = mat4():translate(-1 * vec3(self.core.getConstructWorldPos()))
    local forward = self.physics.orientation.forward
    local up = self.physics.orientation.up
    local right = self.physics.orientation.right

    local worldToShipRotationTransform = mat4({right.x,forward.x,up.x,0,right.y,forward.y,up.y,0,right.z,forward.z,up.z,0,0,0,0,1})

    local shipPosition = worldToShipTraslationTransform * worldToShipRotationTransform * {worldPosition.x,worldPosition.y,worldPosition.z,1}
    return vec3(shipPosition[1],shipPosition[2],shipPosition[3])
end

function PhysicsHelperClass:widget()
   local widgetText = [[]];

   self.widgetInfo.CROSS_SECTION = round(self.core.getConstructCrossSection() * 16,1) .. [[ voxels]]
   self.widgetInfo.maxthurst = self.engines.aileronV.getMaxThrust() *  self.engines.amountAileronV
   self.widgetInfo.potentialAcc = self.physics.acceleration.potential.up

   
  for k,v,item in pairs(self.widgetInfo) do
    widgetText = widgetText .. [[<div>]] ..k.. [[</div> <div><b>]] .. v .. [[</b></div>]]
  end
   return [[<div></div>]]
-- return [[
--           <div style="width: 100%;background-color: #252f43;border-radius: 5px;font-size: 14px;"> VerticalPosition]] ..
--              widgetText ..
--          [[</div>
--       ]]
end  


function PhysicsHelperClass:worldToShipSpeed(worldPosition)
    local worldToShipTraslationTransform = mat4():translate(-1 * vec3(self.core.getConstructWorldPos()))
                
    local forward = vec3(self.physics.velocity.tangential.vector):normalize()
    local up = self.physics.orientation.up
    local right = forward:cross(up):normalize()
    up = right:cross(forward):normalize()

    local worldToShipRotationTransform = mat4({right.x,forward.x,up.x,0,right.y,forward.y,up.y,0,right.z,forward.z,up.z,0,0,0,0,1})
    local shipPosition = worldToShipTraslationTransform * worldToShipRotationTransform * {worldPosition.x,worldPosition.y,worldPosition.z,1}
    return vec3(shipPosition[1],shipPosition[2],shipPosition[3])
end

function PhysicsHelperClass:localToScreen(localPosition)
  local n = 1 -- near = 1 meter
  local f = 10000 -- far = 10000 meters
  local l = -0.5 -- near = -0.5 meter
  local r = 0.5 -- near = 0.5 meter
  local b = -0.3 -- near = -0.3 meter
  local t = 0.3 -- top = 0.3 meter


    local worldToShipTraslationTransform = mat4({
      2*n/(r-l),  0,          (r+l)/(r-l),    0,
      0,          2*n/(t-b),  (t+b)/(t-b),    0,
      0,          0,          -1*(f+n)/(f-n), -2*f*n/(f-n),
      0,          0,          -1,             0
      })

    
end


function PhysicsHelperClass:setCurrentPlanet()
  if (self.currentPlanet == nil) then
    local closestPlanet = nil 
    local bestDistance = -1

    for k, planet in pairs(PlanetsDatabase) do
      if (bestDistance == -1 or (vec3(self.core.getConstructWorldPos()) - planet.position):len() < bestDistance) then
        self.currentPlanet = planet
      end
    end
  end
end

-- this makes the spherical coordinates relative to the desired destination beeing Theta the angular distance and Phy the perpendicular
function PhysicsHelperClass:SetOnPlanetDestination( destination) -- in global coordinates
  self.destination = destination;
end

function PhysicsHelperClass:calculate()
 
  local worldGravity = vec3(self.core.getWorldGravity())
  local worldVertical = (-1 * worldGravity):normalize()

  local verticalVelocityMagnitude = vec3(self.core.getWorldVelocity()):dot(worldVertical) 
  local tangentialVelocityVector =  vec3(self.core.getWorldVelocity()):project_on_plane(worldVertical)

  local radius = (vec3(self.core.getConstructWorldPos()) - self.currentPlanet.position):len()
  local targetDistanceToPlanetCenter = (vec3(self.destination) - self.currentPlanet.position):len()
  local centrifugalAcceleration = utils.pow2(tangentialVelocityVector:len()) / radius;

  local angularVelocityMagnitude = tangentialVelocityVector:len() / radius


  -- planet relative magnitudes
  local planetRelativePosition = vec3(self.core.getConstructWorldPos()) - self.currentPlanet.position
  local planetNorth = self.destination - self.currentPlanet.position

  local destinationInTangentPlane = planetNorth:project_on_plane(worldVertical)

  -- local ThetaPositon = planetRelativePosition:angle_between(planetNorth)

  -- local proyectedNorthVector = planetNorth:project_on_plane(planetRelativePosition)

  local PhySpeed =   tangentialVelocityVector:normalize():cross(destinationInTangentPlane:normalize()):len()
  local ThetaSpeed = tangentialVelocityVector:normalize():dot(destinationInTangentPlane:normalize())
  
  local tagetDirectionAngle = 0
   local targetLocalPosition = self.destination
  if (self.physics ~= nil) then
  
     if tangentialVelocityVector:len() > 20 then
       targetLocalPosition  = self:worldToShipSpeed(self.destination)
     else
       targetLocalPosition  = self:worldToShip(self.destination)
     
     end

     if (targetLocalPosition.x ~= 0) then
        if targetLocalPosition.x < 0 then
            tagetDirectionAngle =  math.atan(targetLocalPosition.y/targetLocalPosition.x)/ math.pi * 180 +90
        else
             tagetDirectionAngle =  math.atan(targetLocalPosition.y/targetLocalPosition.x)/ math.pi * 180  -90
        end
    end
  end

  self.physics =  { 
    acceleration = {
      potential = {
        up = self:getEnginesMaxUpAcceleration(), --+ centrifugalAcceleration,
        forward = self:getEnginesMaxForwardAcceleration(), --+ centrifugalAcceleration,
        down = self:getEnginesMaxDownAcceleration() + worldGravity:len() - centrifugalAcceleration,
        lateral = self:getEnginesMaxLateralAcceleration()
      },
      centrifugal = {
        vector = centrifugalAcceleration * worldVertical,
        len = centrifugalAcceleration
      },
      gravity = {
        vector = worldGravity,
        len = worldGravity:len()
      }
    },
    velocity={
      world={
        vector = vec3(self.core.getWorldVelocity())
      },
      tangential = {
        vector = tangentialVelocityVector,
        len = tangentialVelocityVector:len(),
      },
      vertical = {
        vector = verticalVelocityMagnitude * worldVertical,
        magnitude = verticalVelocityMagnitude
      },
      spherical = {
        phy=  PhySpeed,
        theta= ThetaSpeed,
        magnitude= angularVelocityMagnitude
      }
    }, 
    position={
      altitude= self.core.getAltitude(),
      world = vec3(self.core.getConstructWorldPos()),
      target = self.destination,
      radius = radius,
      -- spherical = {
      --   Theta = ThetaPositon
      -- },
      atmosphere = self.control.getAtmosphereDensity(),
      worldVertical = worldVertical,
      distanceToPlanetCenter = radius,
      targetDistanceToPlanetCenter = targetDistanceToPlanetCenter,
      
      targetWorld = {
        altitude = 0, -- TODO

      },
      targetLocal = {
        vector = targetLocalPosition,
        len = targetLocalPosition:len(),

      }
    },

    orientation = {
        up =  vec3(self.core.getConstructWorldOrientationUp()):normalize(),
        forward = vec3(self.core.getConstructWorldOrientationForward()):normalize(),
        right = vec3(self.core.getConstructWorldOrientationRight()):normalize(),
        inplanetTarget = vec3(0,0,0),
        targetAngle = tagetDirectionAngle
    }


  }

end

function PhysicsHelperClass:getEnginesMaxUpAcceleration()
  local enginesVerticalThrust = 0
  if  (self.engines.amountVertical > 0) then 
    enginesVerticalThrust = self.engines.vertical.getMaxThrust() *  self.engines.amountVertical
  end

  local aileronVerticalThrust = 0
  if  (self.engines.amountAileronV > 0) then 
    aileronVerticalThrust = self.engines.aileronV.getMaxThrust() *  self.engines.amountAileronV
  end
  return (enginesVerticalThrust + aileronVerticalThrust ) /  self.mass;
end


function PhysicsHelperClass:getEnginesMaxForwardAcceleration()
  if  (self.engines.amountHorizontal == 0) then return 0 end
  return (self.engines.horizontal.getMaxThrust() *  self.engines.amountHorizontal ) /  self.mass;
end


function PhysicsHelperClass:getEnginesMaxDownAcceleration()
  if  (self.engines.amountAileronV == 0) then return 0 end
  return (self.engines.aileronV.getMaxThrust() *  self.engines.amountAileronV ) /  self.mass;
end

function PhysicsHelperClass:getEnginesMaxLateralAcceleration()
  if  (self.engines.amountAileronH == 0) then return 0 end
  return (self.engines.aileronH.getMaxThrust() *  self.engines.amountAileronH ) /  self.mass;
end

PhysicsHelper =  setmetatable(
    {
        new = new
    }, {
        __call = function(_, ...) return new(...) end
    }
)

return PhysicsHelper