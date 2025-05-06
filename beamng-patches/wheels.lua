-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.wheelRotators = {}
M.wheelRotatorIDs = {}
M.wheelRotatorCount = 0

M.wheels = {}
M.wheelIDs = {}
M.wheelCount = 0

M.rotators = {}
M.rotatorIDs = {}
M.rotatorCount = 0

M.wheelPower = 0
M.wheelTorque = 0

local max, min, abs = math.max, math.min, math.abs
local random = math.random
local pi = math.pi
local exp = math.exp

local kelvinToCelsius = -273.15
local celsiusToKelvin = 273.15
local brakeCoreWaterToSteamThresholdTemperature = 70

local initialWheelCountDec = -1
local initialRotatorCountDec = -1
local initialWheelRotatorCountDec = -1
local invWheelCount = 0
local speedoWheelCount = 0
local initialSpeedoWheelCount = 0
local invSpeedoWheelCount = 0
local ffiWheelCoreData

local minBrakeMass = 1
local brakeSmokeEfficiencyThreshold = 0.75
local wheelInfo = {}
local guiWheelInfo = {wheels = wheelInfo}
local axleBeamLookup = {}

local startPosition = nil
local state = "idle"
local targetSpeed = 200 / 3.6

local absBehavior = nil
local defaultABSBehavior = nil
local maxBrakeTorque = 0
local maxDoubleBrakeTorque = 0
local brakeABSCoefLimits = {left = 0, right = 0}
local brakeABSCoefLimitCache = {left = 0, right = 0}
local absPulse = 0
local absActive = false
local absActiveLastPulse = false
local warningLightsDelayTime = 0.15 --s
local warningLightsTimer = 0 --s

local updateVirtualAirspeedMethod = nop
local virtualAirspeed = 0
local lastVirtualAirspeed = 0
local airspeedMapTimer = 0
local airspeedMapTime = 0.05
local lastBrake = 0
local lastAccSign = 1

local airspeedBrakeThreshold = 0.2
local airspeedThrottleThreshold = 0.3
local airspeedYawThreshold = 1
local airspeedResetTimer = 0
local airspeedResetTime = 0.3
local airspeedResetSpeedThreshold = 1 / 0.7

local brakeThermalsEnabled
local padThermalEfficiencyData = {
  ["basic"] = {w1x1Coef = 0.018462, w2x1Coef = -0.013846, b1 = 3, b2 = 7, a = -0.988}, --w1x1Coef = w1 / (2 * x1), w2x1Coef = w2 / (2 * x1)
  ["premium"] = {w1x1Coef = 0.016, w2x1Coef = -0.012, b1 = 2.6, b2 = 7.3, a = -0.985},
  ["sport"] = {w1x1Coef = 0.019231, w2x1Coef = -0.0136, b1 = 2, b2 = 9.75, a = -0.993},
  ["semi-race"] = {w1x1Coef = 0.008462, w2x1Coef = -0.0117, b1 = 1.5, b2 = 10.75, a = -0.982},
  ["full-race"] = {w1x1Coef = 0.009231, w2x1Coef = -0.0115, b1 = 0.8, b2 = 12.5, a = -0.988},
  ["carbon-ceramic"] = {w1x1Coef = 0.016, w2x1Coef = -0.011, b1 = -0.8, b2 = 16, a = -0.996},
  ["godmode"] = {w1x1Coef = 0.007692, w2x1Coef = -0.007692, b1 = 10, b2 = 100, a = -1.001}
}

local virtualAirspeedMaps = {
  {acceleration = 0.1, invGainSum = 0, wheelCoef = 1, correctionCoef = 1.01}, -- stable, idle
  {acceleration = 0, invGainSum = 0, wheelCoef = 1, correctionCoef = 0.9}, -- heavyBrakingInit, idle
  {acceleration = 1, invGainSum = 0, wheelCoef = 0, correctionCoef = 1.00}, -- heavyBraking, braking
  {acceleration = 2, invGainSum = 0, wheelCoef = 0, correctionCoef = 1.01}, -- heavyAcceleration, acceleration
  {acceleration = 0.1, invGainSum = 0, wheelCoef = 1, correctionCoef = 1} -- heavyYaw, idle
}

local updateThermalsGFXMethod = nop

local function updateWheelAngles()
  if electrics ~= nil then
    local steeringSign = sign(electrics.values.steering_input)
    local wheelRotators = M.wheelRotators
    local FLWh, FRWh = wheelRotators[M.wheelRotatorIDs.FL], wheelRotators[M.wheelRotatorIDs.FR]
    local frontLeftWheelAngle = math.acos(obj:nodeVecPlanarCosRightForward(FLWh.node2, FLWh.node1)) * steeringSign
    local frontRightWheelAngle = math.acos(obj:nodeVecPlanarCosRightForward(FRWh.node1, FRWh.node2)) * steeringSign
    -- Take the average
    electrics.values.wheel_angle = (frontLeftWheelAngle + frontRightWheelAngle) * 0.5
  else
      electrics.values.wheel_angle = 0.0
  end
end
---------------

local function nodeCollision(p)
  local collisionNodeId = p.id1
  local node = v.data.nodes[collisionNodeId]
  if node then
    local wheelID = node.wheelID
    if wheelID then
      local wheelRot = M.wheelRotators[wheelID]
      if obj:inSameNodeCluster(collisionNodeId, wheelRot.node1) then
        wheelRot.lastTreadContactNode = collisionNodeId
      end
      if p.slipForce > 0 and p.materialID ~= 4 and p.materialID2 ~= 4 then
        sounds.bodyCollision(p)
      end
    elseif p.slipForce > 0 then
      sounds.bodyCollision(p)
    end
  end
end

local function beamBroke(id)
  local beamName = v.data.beams[id].name
  if not beamName or not axleBeamLookup[beamName] then
    return
  end

  for _, v in ipairs(axleBeamLookup[beamName]) do
    local wd = M.wheelRotators[v]
    if not wd.isBroken then
      wd.isBroken = true
      wd.propulsionTorque = 0
      wd.brakingTorque = 0
      wd.desiredBrakingTorque = 0
      wd.desiredMainBrakingTorque = 0
      wd.lastABSCoef = 0
      wd.angularVelocity = 0
      wd.angularVelocityBrakeCouple = 0
      obj:setWheelTorqueAndBrakeTorque(wd.cid, 0, 0)
      damageTracker.setDamage("wheels", wd.name, true)
      -- Brake damage
      damageTracker.setDamage("wheels", "brake" .. wd.name, true)
      if wd.rotatorType == "wheel" then
        M.wheelCount = M.wheelCount - 1
        invWheelCount = M.wheelCount > 0 and 1 / M.wheelCount or 0
        if wd.isSpeedo == 1 then
          speedoWheelCount = speedoWheelCount - 1
          invSpeedoWheelCount = speedoWheelCount > 0 and 1 / speedoWheelCount or 0
        end
      elseif wd.rotatorType == "rotator" then
        M.rotatorCount = M.rotatorCount - 1
      end
    end
  end
end

local function scaleBrakeTorque(coef)
  coef = coef or 0
  for i = 0, initialWheelCountDec do
    local wd = M.wheels[i]
    if wd.brakeTorque then
      wd.brakeTorque = wd.initialBrakeTorque * coef
    end
    if wd.parkingTorque then
      wd.parkingTorque = wd.initialParkingTorque * coef
    end
  end
end

local function calculateThermalEfficiency(temperature, config)
  local z1 = config.w1x1Coef * temperature + config.b1
  local z2 = config.w2x1Coef * temperature + config.b2

  local sigma1 = 1 / (1 + exp(-z1))
  local sigma2 = 1 / (1 + exp(-z2))
  local f = sigma1 + sigma2 + config.a

  return min(max(f, 0), 1)
end

local function updateThermalsGFX(dt)
  local tEnv = obj:getEnvTemperature() + kelvinToCelsius
  local airSpeed = electrics.values.airflowspeed
  local updateGUI = streams.willSend("wheelThermalData")
  local updateDamage = damageTracker.willSend()
  for i = 0, initialWheelRotatorCountDec do
    local wd = M.wheelRotators[i]
    local brakeSquealVolume, brakeSquealPitch, brakeSquealColor = 0, 0, 0
    if wd.enableBrakeThermals and not wd.isBroken then
      local isUnderWater = 1 + (obj:inWater(wd.node1) and wd.underwaterSurfaceCoolingCoef or 0)
      local isRotatingBrakeCouple = wd.obj:isRotatingBrakeCouple() --0,1 coef based on instability detection of the wheel AV inside the core
      local avBrakeCouple = wd.angularVelocityBrakeCouple * isRotatingBrakeCouple
      local absAVBrakeCouple = abs(avBrakeCouple)
      local wheelSpeed = absAVBrakeCouple * wd.radius
      local energyToBrakeSurface = wd.brakingTorque * absAVBrakeCouple

      local surfaceCoolingCoef = wd.brakeVentingCoef * wd.brakeTypeSurfaceCoolingCoef
      local coreCoolingCoef = wd.brakeVentingCoef * wd.brakeTypeCoreCoolingCoef

      local surfaceCooling = ((40 * wd.brakeTypeSurfaceCoolingCoef) + max(airSpeed, wheelSpeed) * surfaceCoolingCoef * 5.0) * isUnderWater
      local coreCooling = ((40 * wd.brakeTypeCoreCoolingCoef) + max(airSpeed * wd.airSpeedCoreCooling, wheelSpeed * wd.wheelSpeedCoreCooling) * coreCoolingCoef * 3.6) * isUnderWater

      local energyBrakeSurfaceToAir = (wd.brakeSurfaceTemperature - tEnv) * wd.brakeCoolingArea * surfaceCooling
      local tempSquared = square(wd.brakeSurfaceTemperature + celsiusToKelvin)
      local energyRadiationToAir = square(tempSquared) * wd.kRadiationToAir * wd.brakeCoolingArea
      local energyBrakeSurfaceToCore = (wd.brakeSurfaceTemperature - wd.brakeCoreTemperature) * wd.kSurfaceToCore * wd.brakeCoolingArea
      local energyBrakeCoreToAir = (wd.brakeCoreTemperature - tEnv) * coreCooling * wd.brakeCoolingArea
      local fireTemperature, fireDistance = fire.getClosestHotNodeTempDistance(wd.node1)
      local energyFireToDiskSurface = max((fireTemperature - wd.brakeSurfaceTemperature) * 20 * max(10 - fireDistance, 0), 0)

      wd.brakeSurfaceTemperature = max(wd.brakeSurfaceTemperature + ((energyToBrakeSurface + energyFireToDiskSurface) - (energyBrakeSurfaceToAir + energyRadiationToAir + energyBrakeSurfaceToCore)) * (dt * wd.brakeSurfaceEnergyCoef), tEnv)
      wd.brakeCoreTemperature = max(wd.brakeCoreTemperature + (energyBrakeSurfaceToCore - energyBrakeCoreToAir) * (dt * wd.brakeCoreEnergyCoef), tEnv)

      wd.isBrakeMolten = wd.brakeCoreTemperature > wd.brakeMeltingPoint or wd.isBrakeMolten

      local thermalEfficiency = wd.isBrakeMolten and 0 or calculateThermalEfficiency(wd.brakeSurfaceTemperature, wd.thermalEfficiencyConfig)
      local slopeSwitchBit = wd.isBrakeMolten and 0 or max(fsign(calculateThermalEfficiency(wd.brakeSurfaceTemperature + 1, wd.thermalEfficiencyConfig) - thermalEfficiency), 0)

      local relativeBrakingCoef = max(abs(wd.coreData.brakeTorqueApplied) - wd.frictionTorque, 0) * wd.invRelativeBrakingTorqueCoef
      local glazingInput = (slopeSwitchBit <= 0 and thermalEfficiency < 0.9) and ((1 - thermalEfficiency) * wd.padGlazingSusceptibility) or (-0.4 * relativeBrakingCoef * clamp((abs(wd.angularVelocityBrakeCouple) - 0.5), 0, 1))
      wd.padGlazingFactor = clamp(wd.padGlazingFactor + glazingInput * dt, 0, 1)

      wd.brakeThermalEfficiency = thermalEfficiency * linearScale(wd.padGlazingFactor, 0, 1.0, 1, 0.8)

      electrics.values.wheelThermals[wd.name].brakeSurfaceTemperature = wd.brakeSurfaceTemperature
      electrics.values.wheelThermals[wd.name].brakeCoreTemperature = wd.brakeCoreTemperature
      electrics.values.wheelThermals[wd.name].brakeThermalEfficiency = wd.brakeThermalEfficiency

      if slopeSwitchBit < 1 and thermalEfficiency <= brakeSmokeEfficiencyThreshold then
        wd.smokeParticleTick = wd.smokeParticleTick > 1 and 0 or wd.smokeParticleTick + dt * 50 * min((brakeSmokeEfficiencyThreshold - thermalEfficiency), 0.08)
        if wd.smokeParticleTick > 1 then
          local particleType = airSpeed < 10 and 48 or 49
          obj:addParticleByNodesRelative(wd.node1, wd.node2, 1 - random(1), particleType, 0, 1)
        end
      end

      if isUnderWater > 1 and wd.brakeSurfaceTemperature > brakeCoreWaterToSteamThresholdTemperature then
        wd.steamParticleTick = wd.steamParticleTick > 1 and 0 or wd.steamParticleTick + dt * 0.05 * (wd.brakeSurfaceTemperature - brakeCoreWaterToSteamThresholdTemperature)
        if wd.steamParticleTick > 1 then
          local particleType = airSpeed < 10 and 48 or 49
          obj:addParticleByNodesRelative(wd.node1, wd.node2, 1 - random(1), particleType, 0, 1)
        end
      end

      if updateDamage then
        damageTracker.setDamage("wheels", "brakeOverHeat" .. wd.name, (slopeSwitchBit < 1 and thermalEfficiency < 0.85) and wd.brakeThermalEfficiency or 0)
        if wd.isBrakeMolten then
          damageTracker.setDamage("wheels", "brake" .. wd.name, wd.isBrakeMolten)
        end
      end

      local actualBrakeTorque = abs(wd.coreData.brakeTorqueApplied) - wd.frictionTorque
      brakeSquealPitch = linearScale(wd.brakeMass, 1, 20, 1, 0)
      brakeSquealColor = 0.5 + linearScale(actualBrakeTorque, 0, wd.initialBrakeTorque * 0.5, -0.5, 0.5)

      local naturalSquealVolume = wd.squealCoefNatural * (1 - clamp(actualBrakeTorque * wd.invInitialBrakeTorque, 0, 0.8)) * clamp(absAVBrakeCouple / 10, 0, 1)
      local lowSpeedSquealVolume = wd.squealCoefLowSpeed * (1 - clamp(actualBrakeTorque / 1000, 0, 1)) * (1 - clamp(absAVBrakeCouple / 10, 0, 1))
      local glazingSquealVolume = wd.squealCoefGlazing * linearScale(wd.padGlazingFactor, 0, 0.1, 0, 1)

      local squealAVTorqueFadeOut = clamp(actualBrakeTorque * 0.05, 0, 1) * clamp((absAVBrakeCouple - 1), 0, 1) --only ever have any volume if both torque and av are > 0
      brakeSquealVolume = (wd.brakeSquealVolumeSmoother:getUncapped(max(naturalSquealVolume, lowSpeedSquealVolume, glazingSquealVolume) * squealAVTorqueFadeOut, dt))

      wd.brakeSquealLoop:setVolumePitch(brakeSquealVolume, brakeSquealPitch, brakeSquealColor, 1)

      if updateGUI then
        if wheelInfo[wd.name] then
          local wi = wheelInfo[wd.name]
          wi.energyToBrakeSurface = energyToBrakeSurface
          wi.brakeSurfaceTemperature = wd.brakeSurfaceTemperature
          wi.brakeCoreTemperature = wd.brakeCoreTemperature
          wi.surfaceCooling = surfaceCooling
          wi.coreCooling = coreCooling
          wi.energyBrakeSurfaceToAir = energyBrakeSurfaceToAir
          wi.energyBrakeSurfaceToCore = energyBrakeSurfaceToCore
          wi.energyBrakeCoreToAir = energyBrakeCoreToAir
          wi.energyRadiationToAir = energyRadiationToAir
          wi.finalBrakeEfficiency = wd.brakeThermalEfficiency
          wi.brakeThermalEfficiency = thermalEfficiency
          wi.padGlazingFactor = wd.padGlazingFactor
          wi.slopeSwitchBit = slopeSwitchBit
          wi.brakeType = wd.brakeType
          wi.padMaterial = wd.padMaterial
        else
          wheelInfo[wd.name] = {
            energyToBrakeSurface = energyToBrakeSurface,
            brakeSurfaceTemperature = wd.brakeSurfaceTemperature,
            brakeCoreTemperature = wd.brakeCoreTemperature,
            surfaceCooling = surfaceCooling,
            coreCooling = coreCooling,
            energyBrakeSurfaceToAir = energyBrakeSurfaceToAir,
            energyBrakeSurfaceToCore = energyBrakeSurfaceToCore,
            energyBrakeCoreToAir = energyBrakeCoreToAir,
            energyRadiationToAir = energyRadiationToAir,
            finalBrakeEfficiency = wd.brakeThermalEfficiency,
            brakeThermalEfficiency = thermalEfficiency,
            padGlazingFactor = wd.padGlazingFactor,
            slopeSwitchBit = slopeSwitchBit,
            brakeType = wd.brakeType,
            padMaterial = wd.padMaterial
          }
        end
      end
    elseif wd.enableBrakeThermals and wd.isBroken and updateGUI then
      wheelInfo[wd.name] = {
        energyToBrakeSurface = 0,
        brakeSurfaceTemperature = 0,
        brakeCoreTemperature = 0,
        surfaceCooling = 0,
        coreCooling = 0,
        energyBrakeSurfaceToAir = 0,
        energyBrakeSurfaceToCore = 0,
        energyBrakeCoreToAir = 0,
        energyRadiationToAir = 0,
        finalBrakeEfficiency = 0,
        brakeThermalEfficiency = 0,
        padGlazingFactor = 0,
        slopeSwitchBit = 0,
        brakeType = wd.brakeType,
        padMaterial = wd.padMaterial
      }
    end
    if wd.brakeSquealLoop then
      wd.brakeSquealLoop:setVolumePitch(brakeSquealVolume, brakeSquealPitch, brakeSquealColor, 1) --always update brake squeal, even if broken
    end
  end

  if updateGUI then
    gui.send("wheelThermalData", guiWheelInfo)
  end
end

local function updateWheelsGFX(dt)
  M.wheelTorque = 0
  M.wheelPower = 0

  for i = 0, initialWheelCountDec do
    local wd = M.wheels[i]
    wd.lastSlip, wd.lastSideSlip, wd.slipEnergy, wd.downForceRaw, wd.peakForce, wd.contactDepth, wd.contactMaterialID1, wd.contactMaterialID2 = wd.obj:getSlipVelEnergyDownPeakForceDepthMats()
    wd.downForce = wd.downForceSmoother:get(wd.downForceRaw, dt)
    if not wd.isBroken then
      M.wheelPower = M.wheelPower + wd.propulsionTorque * wd.angularVelocity
      M.wheelTorque = M.wheelTorque + wd.propulsionTorque * wd.wheelDir
    end

    wd.dynamicRadius = wd.dynamicRadiusSmoother:get(wd.lastTreadContactNode and obj:nodeLineSectionDistance(wd.lastTreadContactNode, wd.node1, wd.node2) or wd.radius, dt)
  end

  warningLightsTimer = warningLightsTimer + dt

  if warningLightsTimer >= warningLightsDelayTime then
    absPulse = absActive and bit.bxor(absPulse, 1) or 0
    warningLightsTimer = 0
  end

  local evals = electrics.values
  evals.abs = absPulse
  evals.absActive = absActive
  evals.odometer = (evals.odometer or 0) + evals.wheelspeed * dt
end

local function updateBrakingDistance(dt)
  if (input.brake or 0) < 0.2 then
    state = "idle"
  end

  local airspeed = electrics.values.airspeed or 0
  if state == "idle" then
    if (input.brake or 0) > 0.2 and airspeed > targetSpeed then
      state = "waiting"
    end
  elseif state == "waiting" then
    if airspeed <= targetSpeed then
      startPosition = obj:getPosition()
      state = "measuring"
      guihooks.message({txt = string.format("Measuring braking distance from %dkm/h...", targetSpeed * 3.6), context = {}}, 1, "vehicle.brakingdistance")
    end
  elseif state == "measuring" then
    if airspeed <= 1 then
      local endPosition = obj:getPosition()
      local distance = (startPosition - endPosition):length()
      local avgDeceleration = -(square(airspeed) - square(targetSpeed)) / (2 * distance)
      guihooks.message({txt = string.format("Brakingdistance from %dkm/h: %.2fm, G: %.2f", targetSpeed * 3.6, distance, avgDeceleration / -powertrain.currentGravity), context = {}}, 5, "vehicle.brakingdistance")
      startPosition = nil
      state = "idle"
    end
  end
end

local function updateGFX(dt)
  updateThermalsGFXMethod(dt)
  updateWheelsGFX(dt)
end

local function updateVirtualAirspeed(dt)
  local brake = electrics.values.brake
  local wheelspeed = electrics.values.wheelspeed
  local mapId = 1 -- stable

  if lastBrake == 0 and brake > 0 and airspeedMapTimer == 0 and virtualAirspeed > wheelspeed then
    mapId = 2 -- heavyBrakingInit
    airspeedMapTimer = airspeedMapTime
  end

  if (brake > airspeedBrakeThreshold or input.parkingbrake ~= 0) and airspeedMapTimer == 0 then
    mapId = 3 -- heavyBraking
    if lastVirtualAirspeed > wheelspeed * airspeedResetSpeedThreshold then
      airspeedResetTimer = airspeedResetTimer + dt
      if airspeedResetTimer > airspeedResetTime then
        lastVirtualAirspeed = (lastVirtualAirspeed + wheelspeed) * 0.5
        airspeedResetTimer = 0
      end
    end
  end

  if electrics.values.throttle > airspeedThrottleThreshold and brake < airspeedBrakeThreshold and airspeedMapTimer == 0 then
    mapId = 4 -- heavyAcceleration
  end

  local ffiSensors = sensors.ffiSensors

  if mapId ~= 1 and abs(ffiSensors.yawAngVel) > airspeedYawThreshold then
    mapId = 5 -- heavyYaw
  end
  local virtualAirspeedMap = virtualAirspeedMaps[mapId]

  lastBrake = brake
  airspeedMapTimer = max(airspeedMapTimer - dt, 0)

  local wheelSpeedSum = 0
  for i = 0, initialWheelCountDec do
    local wd = M.wheels[i]
    wheelSpeedSum = wheelSpeedSum + abs(wd.angularVelocity * wd.radius)
  end
  wheelSpeedSum = wheelSpeedSum * virtualAirspeedMap.wheelCoef

  local accSign = wheelspeed > 2 and fsign(electrics.values.avgWheelAV) or lastAccSign
  lastAccSign = accSign

  local accSpeed = (lastVirtualAirspeed - ffiSensors.sensorY * dt * accSign) * virtualAirspeedMap.acceleration
  lastVirtualAirspeed = (wheelSpeedSum + accSpeed) * virtualAirspeedMap.invGainSum
  virtualAirspeed = lastVirtualAirspeed * virtualAirspeedMap.correctionCoef
  electrics.values.virtualAirspeed = virtualAirspeed

  --  if streams.willSend("genericGraphAdvanced") then
  --    gui.send('genericGraphAdvanced', {
  --        virtualSpeed = { title = "Virtual Speed", color = getContrastColorStringRGB(7), unit = "km/h", value = virtualAirspeed * 3.6},
  --        realSpeed = { title = "Real Speed", color = getContrastColorStringRGB(1), unit = "km/h", value = obj:getGroundSpeed() * 3.6},
  --        wheelSpeed = { title = "Wheel Speed", color = getContrastColorStringRGB(8), unit = "km/h", value = wheelspeed * 3.6},
  --        yaw = { title = "Yaw Rate", color = getContrastColorStringRGB(5), unit = "km/h", value = abs(obj:getYawAngularVelocity()) * 10},
  --        mapIndex = { title = "Map Index", color = getContrastColorStringRGB(11), unit = "", value = (mapId-1) * 10},
  --      })
  --  end
end

local function updateABSCoef(wd, brake, invAirspeed, airspeed, airspeedCutOff, dt)
  if brake > 0 then
    wd.absTimer = wd.absTimer - dt
    if wd.absTimer <= 0 then
      local absDT = max(dt, wd.absTime) --if the ABS frequency is smaller than the physics step, we need to use the right dt here
      wd.absTimer = wd.absTimer + wd.absTime
      local slipRatio = min(max((airspeed - abs(wd.angularVelocityBrakeCouple * wd.radius * wd.wheelDir)) * invAirspeed, 0), 1)
      local slipRatioTarget = min(2 * invAirspeed + wd.slipRatioTarget, 1)
      local slipError = slipRatioTarget - slipRatio
      local slipErrorDerivative = (slipError - wd.lastSlipError) / absDT
      wd.slipErrorIntegral = max(min(wd.slipErrorIntegral + slipError * absDT, 1), -1)
      local ABSCoef = airspeedCutOff and min(max(slipError * 5 + wd.slipErrorIntegral * 0.3 + slipErrorDerivative * 0.05, 0), 1) or 1

      wd.absActive = brake > 0.1 and ABSCoef < 0.9
      absActive = wd.absActive or absActive
      absActiveLastPulse = absActive

      brakeABSCoefLimitCache[wd.oppositeWheelSide] = max(min(ABSCoef * 1.15, 1), brakeABSCoefLimitCache[wd.oppositeWheelSide])
      ABSCoef = min(ABSCoef, brakeABSCoefLimits[wd.ownWheelSide])

      wd.lastABSCoef = ABSCoef
      wd.lastSlipError = slipError
      return ABSCoef
    else
      brakeABSCoefLimitCache[wd.oppositeWheelSide] = brakeABSCoefLimits[wd.oppositeWheelSide]
      absActive = absActive or absActiveLastPulse
      return wd.lastABSCoef
    end
  else
    wd.slipErrorIntegral = 0
    wd.lastSlipError = 0
    wd.absActive = false
    absActiveLastPulse = false
    brakeABSCoefLimitCache[wd.oppositeWheelSide] = 1
    return 0
  end
end

local function updateBrakeABS(wd, brake, invAirspeed, airspeed, airspeedCutOff, dt)
  if brake > 0 and wd.brakeTorque > 0 then
    local brakeInputSplit = wd.brakeInputSplit
    local nonABSBrakingTorque = wd.brakeTorque * (min(brake, brakeInputSplit) + max(brake - brakeInputSplit, 0) * wd.brakeSplitCoef)
    local absCoef = updateABSCoef(wd, brake, invAirspeed, airspeed, airspeedCutOff, dt)
    local desiredBrakingTorque = nonABSBrakingTorque * absCoef
    return desiredBrakingTorque
  else
    return 0
  end
end

local function updateBrakeNoABS(wd, brake, invAirspeed, airspeed, airspeedCutOff, dt)
  local brakeInputSplit = wd.brakeInputSplit
  return wd.brakeTorque * (min(brake, brakeInputSplit) + max(brake - brakeInputSplit, 0) * wd.brakeSplitCoef)
end

local function updateWheelVelocities(dt)
  updateVirtualAirspeedMethod(dt)

  local airspeed = absBehavior == "arcade" and electrics.values.airspeed or virtualAirspeed
  --local airspeed = controller.getController("CMU").virtualSensors.virtual.speed or 0
  local invAirspeed = 1 / (airspeed + 1e-30)
  local airspeedCutOffSpeed = 5
  local airspeedCutOff = airspeed > airspeedCutOffSpeed

  local brake = electrics.values.brake or 0
  local parkingbrakeInput = input.parkingbrake or 0
  absActive = false

  brakeABSCoefLimitCache.left = 0
  brakeABSCoefLimitCache.right = 0

  local avgAV = 0
  local avgWheelSpeed = 0

  local wheels = M.wheels
  for i = 0, initialWheelCountDec do
    local wd = wheels[i]
    if not wd.isBroken then
      wd.lastAngularVelocity = wd.angularVelocity
      wd.lastAngularVelocityBrakeCouple = wd.angularVelocityBrakeCouple
      local wav = wd.coreData
      local wheelAV = wav.angularVelocity
      wd.angularVelocity = wheelAV
      wd.angularVelocityBrakeCouple = wav.angularVelocityBrakeCouple
      local wheelAVdir = wheelAV * wd.wheelDir
      local wheelSpeed = wheelAVdir * wd.dynamicRadius
      local isSpeedo = wd.isSpeedo
      avgAV = avgAV + wheelAVdir * isSpeedo
      avgWheelSpeed = avgWheelSpeed + wheelSpeed * isSpeedo
      wd.wheelSpeed = wheelSpeed
      -- composite brake (normal + parking)
      wd.desiredMainBrakingTorque = wd:updateBrake(brake * wd.defaultBrakeInputUsageCoef, invAirspeed, airspeed, airspeedCutOff, dt)
      wd.desiredBrakingTorque = max(wd.desiredMainBrakingTorque, wd.parkingTorque * parkingbrakeInput * wd.defaultBrakeInputUsageCoef)
    end
  end

  local rotators = M.rotators
  for i = 0, initialRotatorCountDec do
    local wd = rotators[i]
    if not wd.isBroken then
      local wav = wd.coreData
      wd.lastAngularVelocity = wd.angularVelocity
      wd.lastAngularVelocityBrakeCouple = wd.angularVelocityBrakeCouple
      local wheelAV = wav.angularVelocity
      wd.angularVelocity = wheelAV
      wd.angularVelocityBrakeCouple = wav.angularVelocityBrakeCouple

      local wheelAVdir = wheelAV * wd.wheelDir
      local wheelSpeed = wheelAVdir * wd.radius
      local isSpeedo = wd.isSpeedo
      avgAV = avgAV + wheelAVdir * isSpeedo
      avgWheelSpeed = avgWheelSpeed + wheelSpeed * isSpeedo
      wd.wheelSpeed = wheelSpeed

      -- composite brake (normal + parking)
      wd.desiredMainBrakingTorque = wd:updateBrake(brake * wd.defaultBrakeInputUsageCoef, invAirspeed, airspeed, airspeedCutOff, dt)
      wd.desiredBrakingTorque = max(wd.desiredMainBrakingTorque, wd.parkingTorque * parkingbrakeInput * wd.defaultBrakeInputUsageCoef)
    end
  end

  local evals = electrics.values
  evals.avgWheelAV = avgAV * invSpeedoWheelCount
  evals.wheelspeed = abs(avgWheelSpeed) * invSpeedoWheelCount

  brakeABSCoefLimits.left = brakeABSCoefLimitCache.left
  brakeABSCoefLimits.right = brakeABSCoefLimitCache.right
end

local function updateWheelTorques(dt)
  local torqueReactionCoefs = powertrain.torqueReactionCoefs
  local wheelRotators = M.wheelRotators
  for i = 0, initialWheelRotatorCountDec do
    local wd = wheelRotators[i]
    local brakingTorque = wd.brakePressureDelay:get(wd.desiredBrakingTorque) * wd.brakeThermalEfficiency
    wd.brakingTorque = brakingTorque
    local t = wd.coreData
    if wd.isBroken then
      t.propulsionTorque = 0
      t.brakingTorque = 0
      t.engineReactionTorque = 0
    else
      local propulsionTorque = wd.propulsionTorque
      t.propulsionTorque = propulsionTorque
      t.brakingTorque = brakingTorque + wd.frictionTorque
      t.engineReactionTorque = abs(propulsionTorque) * torqueReactionCoefs[wd.torsionReactorIdx]
    end
  end

  --updateBrakingDistance(dt)
  
  -- ALESSIO: I am not sure where I can add this function but at some point we need it. Probably, where it computes the new angles after steering input changes...
  updateWheelAngles()

end

local function updateWheelBrakeMethods()
  local needsVirtualAirspeed = false
  local hasABS = false
  for i = 0, M.wheelRotatorCount - 1 do
    local wd = M.wheelRotators[i]
    wd.updateBrake = wd.updateBrakeNoABS
    if (wd.hasABS and absBehavior ~= "off") or absBehavior == "arcade" then
      needsVirtualAirspeed = absBehavior == "realistic"
      hasABS = true
      wd.updateBrake = wd.updateBrakeABS
    end
    wd.absTime = absBehavior == "realistic" and 1 / wd.absFrequency or 0.01
  end

  electrics.values.hasABS = hasABS

  updateVirtualAirspeedMethod = needsVirtualAirspeed and updateVirtualAirspeed or nop
end

local function setABSBehavior(behavior)
  absBehavior = behavior
  updateWheelBrakeMethods()
end

local function getDefaultABSBehavior()
  if defaultABSBehavior == nil then
    defaultABSBehavior = settings.getValue("absBehavior") or "realistic"
  end
  return defaultABSBehavior
end

local function settingsChanged()
  defaultABSBehavior = settings.getValue("absBehavior") or "realistic"
end

local function resetABSBehavior()
  setABSBehavior(getDefaultABSBehavior())
end

local function toggleABSBehavior()
  local defaultABSBehavior = getDefaultABSBehavior()
  if defaultABSBehavior == "off" then
    defaultABSBehavior = "realistic"
  end
  setABSBehavior(absBehavior == "off" and defaultABSBehavior or "off")
  guihooks.message("ABS behavior: " .. absBehavior, 5, "vehicle.absBehavior")
end

local function setWheelRotatorType(wheelID, rotatorType)
  M.wheelRotators[wheelID].rotatorType = rotatorType
end

local function resetThermals()
  local tEnv = obj:getEnvTemperature() + kelvinToCelsius
  local startPreHeated = settings.getValue("startBrakeThermalsPreHeated")
  electrics.values.wheelThermals = {}

  if brakeThermalsEnabled then
    for _, wd in pairs(M.wheelRotators) do
      if wd.enableBrakeThermals then
        wd.brakeThermalEfficiency = 1
        wd.padGlazingFactor = 0
        wd.smokeParticleTick = 0
        wd.steamParticleTick = 0
        wd.isBrakeMolten = false

        local startTemp = tEnv
        if startPreHeated and (string.find(wd.padMaterial, "race") or string.find(wd.padMaterial, "carbon")) then
          local efficiency
          repeat
            startTemp = startTemp + 1
            efficiency = calculateThermalEfficiency(startTemp, wd.thermalEfficiencyConfig)
          until efficiency >= 0.95
          startTemp = startTemp + 50
        end

        wd.brakeSurfaceTemperature = startTemp
        wd.brakeCoreTemperature = startTemp

        electrics.values.wheelThermals[wd.name] = {}
        electrics.values.wheelThermals[wd.name].brakeSurfaceTemperature = wd.brakeSurfaceTemperature
        electrics.values.wheelThermals[wd.name].brakeCoreTemperature = wd.brakeCoreTemperature
        electrics.values.wheelThermals[wd.name].brakeThermalEfficiency = wd.brakeThermalEfficiency

        -- Reset brake damage values
        damageTracker.setDamage("wheels", "brake" .. wd.name, false)
        -- Reset brake overheating values
        damageTracker.setDamage("wheels", "brakeOverHeat" .. wd.name, 0)
      end
    end
  end
end

local function resetWheels()
  -- startPosition = nil
  -- state = "idle"

  M.wheelRotatorCount = initialWheelRotatorCountDec + 1

  electrics.values.avgWheelAV = 0
  electrics.values.wheelspeed = 0

  for i = 0, initialWheelRotatorCountDec do
    local wd = M.wheelRotators[i]
    wd.lastTorqueMode = 0
    wd.lastSlip = 0
    wd.lastSideSlip = 0
    wd.slipEnergy = 0
    wd.contactMaterialID1 = -1
    wd.contactMaterialID2 = -1
    wd.contactDepth = 0
    wd.downForceRaw = 0
    wd.peakForce = 0
    wd.downForceSmoother:reset()
    wd.downForce = 0
    wd.isTireDeflated = false
    wd.deflatedTireAngle = 0

    wd.slipErrorIntegral = 0
    wd.lastSlipError = 0
    wd.isBroken = false
    wd.absTimer = 0

    wd.propulsionTorque = 0
    wd.brakingTorque = 0
    wd.frictionTorque = 0
    wd.desiredBrakingTorque = 0
    wd.desiredMainBrakingTorque = 0
    wd.lastABSCoef = 0
    wd.angularVelocity = 0
    wd.angularVelocityBrakeCouple = 0

    wd.brakePressureDelay:reset()
    wd.brakeThermalEfficiency = 1

    wd.dynamicRadius = wd.radius
    if wd.dynamicRadiusSmoother then
      wd.dynamicRadiusSmoother:set(wd.radius)
    end

    --make sure to reset brake torques to initial values in case they were altered
    if wd.initialBrakeTorque then
      wd.brakeTorque = wd.initialBrakeTorque
    end
    if wd.initialParkingTorque then
      wd.parkingTorque = wd.initialParkingTorque
    end
  end

  setABSBehavior(absBehavior)

  brakeABSCoefLimits.left = 1
  brakeABSCoefLimits.right = 1

  
end

local function reset()
  resetWheels()
  resetThermals()
end

local function initSounds()
  for i = 0, initialWheelRotatorCountDec do
    local wd = M.wheelRotators[i]
    wd.flatTireSound = wd.flatTireSound or "event:>Surfaces>Flat_Tire"
    if wd.brakeTorque or wd.parkingTorque then
      wd.brakeSquealLoop = sounds.createSoundObj(wd.brakeSquealLoop or "event:>Vehicle>Failures>failure_brakes_normal", "AudioDefaultLoop3D", "", wd.node1)
      wd.brakeSquealVolumeSmoother = newTemporalSmoothing(100, 4)
    end
  end
end

local function initThermals()
  M.updateThermalsGFX = nop

  local tEnv = obj:getEnvTemperature() + kelvinToCelsius
  brakeThermalsEnabled = false

  local startPreHeated = settings.getValue("startBrakeThermalsPreHeated")
  electrics.values.wheelThermals = {}

  for _, wd in pairs(M.wheelRotators) do
    if wd.enableBrakeThermals then
      brakeThermalsEnabled = true
      wd.brakeMass = max(wd.brakeMass or 10, minBrakeMass)
      wd.brakeDiameter = wd.brakeDiameter or 0.35

      wd.brakeType = wd.brakeType or "vented-disc"
      if wd.brakeType == "vented-disc" then
        wd.brakeCoolingArea = pi * wd.brakeDiameter * wd.brakeDiameter / 2 * 0.7
        wd.brakeTypeSurfaceCoolingCoef = 1
        wd.brakeTypeCoreCoolingCoef = 1
        wd.wheelSpeedCoreCooling = 1
        wd.airSpeedCoreCooling = 0.5
      elseif wd.brakeType == "carbon-ceramic-vented-disc" then
        wd.brakeCoolingArea = pi * wd.brakeDiameter * wd.brakeDiameter / 2 * 0.7
        wd.brakeTypeSurfaceCoolingCoef = 1
        wd.brakeTypeCoreCoolingCoef = 1
        wd.wheelSpeedCoreCooling = 1
        wd.airSpeedCoreCooling = 0.6
      elseif wd.brakeType == "disc" then
        wd.brakeCoolingArea = pi * wd.brakeDiameter * wd.brakeDiameter / 2 * 0.7
        wd.brakeTypeSurfaceCoolingCoef = 1
        wd.brakeTypeCoreCoolingCoef = 0.01
        wd.wheelSpeedCoreCooling = 0
        wd.airSpeedCoreCooling = 0.01
      elseif wd.brakeType == "drum" then
        --perimeter * width + some side
        --wd.brakeCoolingArea = math.pi * wd.brakeDiameter * wd.brakeDiameter * 0.22
        wd.brakeCoolingArea = pi * (wd.brakeDiameter * (wd.brakeDiameter * 0.22) + wd.brakeDiameter * wd.brakeDiameter / 4 * 0.25)
        wd.brakeTypeSurfaceCoolingCoef = 0.25
        wd.brakeTypeCoreCoolingCoef = 1 --because brake drum "core" is in primary airflow
        wd.wheelSpeedCoreCooling = 1
        wd.airSpeedCoreCooling = 1
      else
        log("E", "wheels.initThermals", "Found unknown brake type: " .. wd.brakeType .. ", disabling brake thermals...")
        brakeThermalsEnabled = false
        break
      end

      wd.rotorMaterial = wd.rotorMaterial or "steel"
      if wd.rotorMaterial == "steel" then
        wd.brakeSpecHeat = 450
        wd.kSurfaceToCore = 55 / 0.01
        wd.kRadiationToAir = 0.0000000567 * 0.75
        wd.brakeMeltingPoint = 1500
      elseif (wd.rotorMaterial == "aluminum" or wd.rotorMaterial == "aluminium") then
        wd.brakeSpecHeat = 910
        wd.kSurfaceToCore = 150 / 0.01 --reduce K a bit from textbook value because aluminum brake still needs steel friction lining
        wd.kRadiationToAir = 0.0000000567 * 0.5
        wd.brakeMeltingPoint = 660
      elseif wd.rotorMaterial == "carbon-ceramic" then
        wd.brakeSpecHeat = 820
        wd.kSurfaceToCore = 22 / 0.01
        wd.kRadiationToAir = 0.000000115 * 0.9
        wd.brakeMeltingPoint = 1800
      else
        log("E", "wheels.initThermals", "Found unknown rotor material: " .. wd.rotorMaterial .. ", disabling brake thermals...")
        brakeThermalsEnabled = false
        break
      end

      wd.padMaterial = wd.padMaterial or "basic"
      wd.padGlazingSusceptibility = wd.padGlazingSusceptibility or 1.0
      wd.thermalEfficiencyConfig = padThermalEfficiencyData[wd.padMaterial] or padThermalEfficiencyData["basic"]

      wd.brakeVentingCoef = wd.brakeVentingCoef or 1
      wd.underwaterSurfaceCoolingCoef = wd.underwaterSurfaceCoolingCoef or 20

      wd.brakeSurfaceEnergyCoef = 1 / (wd.brakeMass * 0.15 * wd.brakeSpecHeat)
      wd.brakeCoreEnergyCoef = 1 / (wd.brakeMass * 0.85 * wd.brakeSpecHeat)
      wd.invRelativeBrakingTorqueCoef = 1 / wd.brakeTorque
      wd.brakeThermalEfficiency = 1
      wd.padGlazingFactor = 0
      wd.smokeParticleTick = 0
      wd.steamParticleTick = 0
      wd.isBrakeMolten = false

      local startTemp = tEnv
      if startPreHeated and (string.find(wd.padMaterial, "race") or string.find(wd.padMaterial, "carbon")) then
        local efficiency
        repeat
          startTemp = startTemp + 1
          efficiency = calculateThermalEfficiency(startTemp, wd.thermalEfficiencyConfig)
        until efficiency >= 0.95
        startTemp = startTemp + 50
      end

      wd.brakeSurfaceTemperature = startTemp
      wd.brakeCoreTemperature = startTemp

      electrics.values.wheelThermals[wd.name] = {}
      electrics.values.wheelThermals[wd.name].brakeSurfaceTemperature = wd.brakeSurfaceTemperature
      electrics.values.wheelThermals[wd.name].brakeCoreTemperature = wd.brakeCoreTemperature
      electrics.values.wheelThermals[wd.name].brakeThermalEfficiency = wd.brakeThermalEfficiency

      -- Initialising brake damage values
      damageTracker.setDamage("wheels", "brake" .. wd.name, false)
      -- Initialising brake overheating values
      damageTracker.setDamage("wheels", "brakeOverHeat" .. wd.name, 0)
    end
  end

  if brakeThermalsEnabled then
    updateThermalsGFXMethod = updateThermalsGFX
  end
end

ffi.cdef [[
  void bng_applyTorqueAxisCouple(void *obj, float torque, int axisn1, int axisn2, int node);
]]

local function initWheels()
  -- startPosition = nil
  -- state = "idle"
  electrics.values.avgWheelAV = 0
  electrics.values.wheelspeed = 0

  local brakesFound = false
  maxBrakeTorque = 0
  maxDoubleBrakeTorque = 0
  axleBeamLookup = {}

  M.wheelRotators = {}
  M.wheelRotatorIDs = {}
  M.wheelRotatorCount = 0
  M.wheels = {}

  M.treadNodeLookup = {}

  local maxWheelCid = 0
  ffiWheelCoreData = ffi.cast("struct{float propulsionTorque; float brakingTorque; float engineReactionTorque; float angularVelocity; float angularVelocityBrakeCouple; float totalTorqueApplied; float brakeTorqueApplied;}*", obj:getWheelsFFI())

  local count = tableSizeC(v.data.wheels or {})
  for i = 0, count - 1, 1 do
    local wd = v.data.wheels[i]
    local wobj = obj:getWheel(wd.wheelID)

    if wobj then
      wobj:setBrakeSpring(max(wd.brakeTorque or 0, wd.parkingTorque or 0, 1) * (wd.brakeSpring or 10))
      brakesFound = brakesFound or (wd.brakeTorque ~= nil)
      maxBrakeTorque = max(maxBrakeTorque, wd.brakeTorque or 0)
      maxDoubleBrakeTorque = max(maxBrakeTorque * 2, maxDoubleBrakeTorque)
      M.wheelRotatorCount = M.wheelRotatorCount + 1
      M.wheelRotatorIDs[wd.name] = wd.cid
      maxWheelCid = max(maxWheelCid, wd.cid)
      local wheel = {
        rotatorType = wd.rotatorType or "wheel",
        wheelSection = wd.wheelSection,
        name = wd.name,
        wheelID = wd.wheelID,
        wheelDir = wd.wheelDir,
        hasTire = wd.hasTire or wd.hasTire == nil,
        hubRadius = wd.hubRadiusSimple or wd.hubRadius or 0, --hubRadiusSimple is used by simplified traffic wheels to convey their actual hubRadius. We can't use the normal hubRadius because these tires PURELY consist of "hub", they do not have a tire.
        radius = (wd.hasTire or wd.hasTire == nil) and wd.radius or (wd.hubRadius or wd.radius), --use radius if there is a tire, if not use hub radius, if there is no hubradius, it might be a rotator, use normal radius then again...
        tireWidth = max(wd.tireWidth or 0, wd.hubWidth or 0),
        treadCoef = wd.treadCoef or 1,
        softnessCoef = wd.softnessCoef or 0.6,
        node1 = wd.node1,
        node2 = wd.node2,
        rayCount = wd.numRays,
        pressureGroup = wd.pressureGroup,
        isPropulsed = false, --powertrain.lua sets this to true for actually propulsed wheels
        brakeMass = wd.brakeMass,
        padMaterial = wd.padMaterial,
        padGlazingSusceptibility = wd.padGlazingSusceptibility,
        enableBrakeThermals = wd.enableBrakeThermals,
        brakeVentingCoef = wd.brakeVentingCoef,
        brakeType = wd.brakeType,
        brakeDiameter = wd.brakeDiameter,
        rotorMaterial = wd.rotorMaterial,
        nodes = wd.nodes,
        treadNodes = wd.treadNodes,
        torsionReactor = {name = "", outputTorque1 = 0},
        torsionReactorIdx = 1,
        lastTorqueMode = 0,
        wheelSpeed = 0,
        lastSlip = 0,
        lastSideSlip = 0,
        slipEnergy = 0,
        contactMaterialID1 = -1,
        contactMaterialID2 = -1,
        contactDepth = 0,
        downForceRaw = 0,
        peakForce = 0,
        isTireDeflated = false,
        deflatedTireAngle = 0,
        downForceSmoother = newTemporalSmoothingNonLinear(5),
        downForce = 0,
        obj = wobj,
        cid = wd.cid,
        slipErrorIntegral = 0,
        lastSlipError = 0,
        slipRatioTarget = wd.absSlipRatioTarget or 0.18,
        isBroken = false,
        isSpeedo = wd.speedo and 1 or (wd.speedo == nil and 1 or 0),
        hasABS = wd.enableABSactuator or wd.enableABS or false,
        absTimer = 0,
        absFrequency = wd.absHz or 100,
        absTime = 1 / (wd.absHz or 100),
        brakeTorque = wd.brakeTorque or 0,
        parkingTorque = wd.parkingTorque or 0,
        initialParkingTorque = wd.parkingTorque or 0,
        propulsionTorque = 0,
        brakingTorque = 0,
        frictionTorque = 0,
        desiredBrakingTorque = 0,
        desiredMainBrakingTorque = 0,
        lastABSCoef = 0,
        angularVelocity = 0,
        angularVelocityBrakeCouple = 0,
        lastAngularVelocity = 0,
        lastAngularVelocityBrakeCouple = 0,
        coreData = ffiWheelCoreData[wd.cid],
        brakeInputSplit = clamp(wd.brakeInputSplit or 1, 0, 1),
        brakeSplitCoef = clamp(wd.brakeSplitCoef or 1, 0, 1),
        brakePressureDelay = newLinearSmoothing(physicsDt, (wd.brakeTorque or 0) / ((wd.brakePressureInDelay or 0.05) + 1e-30), (wd.brakeTorque or 0) / ((wd.brakePressureOutDelay or 0.1) + 1e-30)),
        brakeThermalEfficiency = 1,
        squealCoefNatural = wd.squealCoefNatural or 0,
        squealCoefLowSpeed = wd.squealCoefLowSpeed or 0,
        squealCoefGlazing = wd.squealCoefGlazing or 1,
        tireSoundVolumeCoef = wd.tireSoundVolumeCoef or 1
      }
      wheel.initialBrakeTorque = wheel.brakeTorque
      wheel.invInitialBrakeTorque = 1 / wheel.brakeTorque
      wheel.useDefaultBrakeInput = wd.useDefaultBrakeInput == nil and (wheel.rotatorType == "wheel" and true or false) or wd.useDefaultBrakeInput
      wheel.defaultBrakeInputUsageCoef = wheel.useDefaultBrakeInput and 1 or 0

      if wheel.pressureGroup and v.data.pressureGroups then
        wheel.pressureGroupId = v.data.pressureGroups[wheel.pressureGroup]
      end

      wheel.inertia = 0
      local node1Pos = vec3(v.data.nodes[wheel.node1].pos)
      local node2Pos = vec3(v.data.nodes[wheel.node2].pos)
      for _, n in pairs(wheel.nodes) do
        local wheelNode = v.data.nodes[n]
        local distanceToAxis = vec3(wheelNode.pos):distanceToLine(node1Pos, node2Pos)
        wheel.inertia = wheel.inertia + wheelNode.nodeWeight * distanceToAxis * distanceToAxis
      end

      M.wheelRotators[wd.cid] = wheel
      if wd.axleBeams then
        for _, name in pairs(wd.axleBeams) do
          if not axleBeamLookup[name] then
            axleBeamLookup[name] = {}
          end
          table.insert(axleBeamLookup[name], wd.wheelID)
        end
      end
      M.setWheelBrakeUpdate(wheel.name, updateBrakeNoABS, updateBrakeABS)
      --insert as wheels as well (temporary) for better backwards compat (controller init right after wheels init, no second stage init done yet)
      M.wheels[wd.cid] = wheel
    else
      log("W", "wheels.initWheels", 'Wheel "' .. wd.name .. '" could not be added to drivetrain')
    end
  end

  local absMode = settings.getValue("absBehavior") or "realistic"
  setABSBehavior(absMode)

  if M.wheelRotatorCount == 0 then
    M.updateGFX = nop
    M.updateWheelTorques = nop
    M.updateWheelVelocities = nop
  end

  brakeABSCoefLimits = {}
  brakeABSCoefLimits.left = 1
  brakeABSCoefLimits.right = 1

end

local function init()
  initWheels()
  initThermals()
end

local function resetSecondStage()
  M.wheelCount = initialWheelCountDec + 1
  M.rotatorCount = initialRotatorCountDec + 1
  M.wheelPower = 0

  invWheelCount = M.wheelCount > 0 and 1 / M.wheelCount or 0
  speedoWheelCount = initialSpeedoWheelCount
  invSpeedoWheelCount = speedoWheelCount > 0 and 1 / speedoWheelCount or 0

  for i = 0, initialWheelCountDec do
    local wd = M.wheels[i]
    damageTracker.setDamage("wheels", wd.name, false)
    damageTracker.setDamage("wheels", "tire" .. wd.name, false)
  end

  airspeedMapTimer = 0
  lastBrake = 0
  lastVirtualAirspeed = 0
  lastAccSign = 1
  airspeedResetTimer = 0
end

local function initSecondStage()
  if not v.data.refNodes or not v.data.nodes then
    return
  end

  M.wheels = {}
  M.wheelIDs = {}
  M.wheelCount = 0
  invWheelCount = 0
  invSpeedoWheelCount = 0
  speedoWheelCount = 0
  M.wheelPower = 0

  M.rotators = {}
  M.rotatorIDs = {}
  M.rotatorCount = 0

  local avgWheelPos = vec3(0, 0, 0)
  for _, rotator in pairs(M.wheelRotators) do
    if rotator.brakeTorque > 0 then
      local wheelNodePos = v.data.nodes[rotator.node1].pos --find the wheel position
      avgWheelPos = avgWheelPos + wheelNodePos --sum up all positions
    end
    if rotator.isSpeedo == 1 then
      speedoWheelCount = speedoWheelCount + 1
    end
    if rotator.rotatorType == "wheel" then
      M.wheels[M.wheelCount] = rotator
      M.wheelCount = M.wheelCount + 1
      M.wheelIDs[rotator.name] = rotator.wheelID
    elseif rotator.rotatorType == "rotator" then
      M.rotators[M.rotatorCount] = rotator
      M.rotatorCount = M.rotatorCount + 1
      M.rotatorIDs[rotator.name] = rotator.wheelID
    end
  end

  initialWheelRotatorCountDec = M.wheelRotatorCount - 1
  initialWheelCountDec = M.wheelCount - 1
  initialRotatorCountDec = M.rotatorCount - 1
  invWheelCount = M.wheelCount > 0 and 1 / M.wheelCount or 0
  invSpeedoWheelCount = speedoWheelCount > 0 and 1 / speedoWheelCount or 0
  initialSpeedoWheelCount = speedoWheelCount
  avgWheelPos = avgWheelPos * invWheelCount --make the average of all positions

  local vectorForward = vec3(v.data.nodes[v.data.refNodes[0].ref].pos) - vec3(v.data.nodes[v.data.refNodes[0].back].pos) --vector facing forward
  local vectorUp = vec3(v.data.nodes[v.data.refNodes[0].up].pos) - vec3(v.data.nodes[v.data.refNodes[0].ref].pos)
  local vectorRight = vectorForward:cross(vectorUp) --vector facing to the right

  for _, rotator in pairs(M.wheelRotators) do
    local wheelNodePos = vec3(v.data.nodes[rotator.node1].pos) --find the wheel position
    local wheelVector = wheelNodePos - avgWheelPos --create a vector from our "center" to the wheel
    local dotLeft = vectorRight:dot(wheelVector) --calculate dot product of said vector and left vector

    if dotLeft >= 0 then
      rotator.ownWheelSide = "right"
      rotator.oppositeWheelSide = "left"
    else
      rotator.ownWheelSide = "left"
      rotator.oppositeWheelSide = "right"
    end
  end

  for i = 0, initialWheelCountDec do
    local wd = M.wheels[i]

    wd.tireVolume = 0
    if wd.hasTire then
      local hubArea = pi * wd.hubRadius * wd.hubRadius
      local overallArea = pi * wd.radius * wd.radius
      local tireArea = overallArea - hubArea
      wd.tireVolume = tireArea * wd.tireWidth
    end

    wd.dynamicRadiusSmoother = newTemporalSmoothingNonLinear(0.5, 0.5)
    wd.dynamicRadiusSmoother:set(wd.radius)
    wd.dynamicRadius = wd.radius

    damageTracker.setDamage("wheels", wd.name, false)
    damageTracker.setDamage("wheels", "tire" .. wd.name, false)
  end

  for _, v in ipairs(virtualAirspeedMaps) do
    local gainSum = (v.acceleration + M.wheelCount * v.wheelCoef)
    v.invGainSum = gainSum > 0 and 1 / gainSum or 0
  end

  airspeedMapTimer = 0
  lastBrake = 0
  lastVirtualAirspeed = 0
  lastAccSign = 1
  airspeedResetTimer = 0

  --dump(M.wheels)
  --dump(M.rotators)
end

local function setWheelBrakeUpdate(wheelName, brakeUpdateMethodNoABS, brakeUpdateMethodABS)
  local wheelRotatorId = M.wheelRotatorIDs[wheelName]
  if wheelRotatorId then
    local wd = M.wheelRotators[wheelRotatorId]
    wd.updateBrakeNoABS = brakeUpdateMethodNoABS
    wd.updateBrakeABS = brakeUpdateMethodABS
  end
  updateWheelBrakeMethods()
end

local function isPhysicsStepUsed()
  return M.wheelRotatorCount > 0
end

M.init = init
M.reset = reset
M.settingsChanged = settingsChanged
M.initSecondStage = initSecondStage
M.initSounds = initSounds
M.resetSecondStage = resetSecondStage
M.setABSBehavior = setABSBehavior
M.toggleABSBehavior = toggleABSBehavior -- used by .Tech
M.resetABSBehavior = resetABSBehavior
M.beamBroke = beamBroke
M.updateGFX = updateGFX
M.updateWheelTorques = updateWheelTorques
M.updateWheelVelocities = updateWheelVelocities
M.setWheelRotatorType = setWheelRotatorType
M.nodeCollision = nodeCollision
M.scaleBrakeTorque = scaleBrakeTorque
M.isPhysicsStepUsed = isPhysicsStepUsed
M.setWheelBrakeUpdate = setWheelBrakeUpdate

M.updateABSCoef = updateABSCoef

return M
