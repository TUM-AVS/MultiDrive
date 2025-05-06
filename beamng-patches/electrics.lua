-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local ceil = math.ceil
local min = math.min
local max = math.max

M.values = {
  throttle = 0,
  brake = 0,
  steering = 0,
  clutch = 0,
  wheelspeed = 0,
  odometer = 0,
  avgWheelAV = 0,
  airspeed = 0,
  horn = false,
  rpmspin = 0,
  rpm = 0,
  boost = 0,
  boostMax = 0
}

M.disabledState = {}

local smoothers = {
  wheelspeed = newExponentialSmoothing(10),
  gear_A = newExponentialSmoothing(10),
  --gear_M = newExponentialSmoothing(10),
  rpm = newExponentialSmoothing(10),
  lights = newExponentialSmoothing(10),
  fuel = newExponentialSmoothing(100),
  oiltemp = newExponentialSmoothing(100),
  watertemp = newExponentialSmoothing(100),
  turnsignal = newExponentialSmoothing(10),
  airspeed = newExponentialSmoothing(10),
  airflowspeed = newExponentialSmoothing(10),
  altitude = newExponentialSmoothing(10)
}

local rpmSmoother = newTemporalSigmoidSmoothing(50000, 75000, 50000, 75000, 0)

local lightsState = 0
local lightsSavedState = 0

local signalRightState = false
local signalLeftState = false
local signalWarnState = false
local lightbarState = 0
local hornState = false

local fogLightsState = false

local blinkPulse = false
local blinkTimerThreshold = 0.4
local blinkTimer = 0

-- sounds
local hornSound1
local hornSound2
local sirenSound
local indStartSnd
local indStopSnd
local indLoopSnd1
local indLoopSnd2
local lightOn
local lightOff
local hasSteered = false -- used to see whether right/left-turn has been finished

-- set to nop in the beginning - this avoids conflict with the warn signal
local automatic_indicator_stop = nop
local generateBlinkPulse = nop

local ignitionLevelSanitization
local previousIgnitionLevel
local updateElectricsWithIgnitionLevel  --used to alter the electrics output based on ignition state, made to support multiple different countries/regulations
local ignitionHoldingStarter = false
local ignitionHoldingStarterTimer = 0
local ignitionWasStartingEngine = false
local ignitionMessageLookup = {
  [0] = "ui.common.vehicleOff",
  [1] = "ui.common.vehicleAccessoryOn",
  [2] = "ui.common.vehicleOn"
}

local function generateBlinkPulseFun(dt)
  blinkTimer = blinkTimer + dt
  if blinkTimer > blinkTimerThreshold then
    if blinkPulse then
      indLoopSnd1 = indLoopSnd1 or sounds.createSoundscapeSound("indLoop1")
      sounds.playSoundSkipAI(indLoopSnd1)
    else
      indLoopSnd2 = indLoopSnd2 or sounds.createSoundscapeSound("indLoop2")
      sounds.playSoundSkipAI(indLoopSnd2)
    end
    blinkPulse = not blinkPulse
    blinkTimer = 0
  end
end

local function updateSignals()
  generateBlinkPulse = (signalLeftState or signalRightState) and generateBlinkPulseFun or nop
end

-- stops automatically indicator if turn has been finished or if wheel is steered in opposite direction
local function manage_automatic_indicator_stop()
  local controlPoint = 100
  local steering = M.values.steering
  if steering == nil then
    return
  end

  --check whether user has steered in the desired direction
  if signalLeftState and steering > controlPoint then
    hasSteered = true
  elseif signalRightState and steering < -controlPoint then
    hasSteered = true
  end

  --if the wheel has returned to the neutral position, turn indicator off
  if signalLeftState and hasSteered and steering <= 0 then
    signalLeftState = false
    hasSteered = false
    sounds.playSoundSkipAI(indStopSnd)
    automatic_indicator_stop = nop
  elseif signalRightState and hasSteered and steering >= 0 then
    signalRightState = false
    hasSteered = false
    sounds.playSoundSkipAI(indStopSnd)
    automatic_indicator_stop = nop
  end

  updateSignals()
end

local function stop_turn_signal()
  if not signalWarnState then
    if signalLeftState or signalRightState then
      signalLeftState = false
      signalRightState = false
      hasSteered = false
      sounds.playSoundSkipAI(indStopSnd)
      automatic_indicator_stop = nop
    end

    updateSignals()
  end
end

-- user input functions
local function toggle_left_signal()
  if not signalWarnState then
    signalLeftState = not signalLeftState
  else
    signalLeftState = true
  end
  if signalLeftState then
    signalRightState = false
    signalWarnState = false
    indStartSnd = indStartSnd or sounds.createSoundscapeSound("indicatorStart")
    sounds.playSoundSkipAI(indStartSnd)
    automatic_indicator_stop = manage_automatic_indicator_stop
    indStopSnd = indStopSnd or sounds.createSoundscapeSound("indicatorStop")
  end
  if not signalLeftState then
    sounds.playSoundSkipAI(indStopSnd)
    automatic_indicator_stop = nop
    hasSteered = false
  end

  updateSignals()
end

local function toggle_right_signal()
  if not signalWarnState then
    signalRightState = not signalRightState
  else
    signalRightState = true
  end
  if signalRightState then
    signalLeftState = false
    signalWarnState = false
    indStartSnd = indStartSnd or sounds.createSoundscapeSound("indicatorStart")
    sounds.playSoundSkipAI(indStartSnd)
    automatic_indicator_stop = manage_automatic_indicator_stop
    indStopSnd = indStopSnd or sounds.createSoundscapeSound("indicatorStop")
  end
  if not signalRightState then
    automatic_indicator_stop = nop
    sounds.playSoundSkipAI(indStopSnd)
    hasSteered = false
  end

  updateSignals()
end

local function toggleSound(val, snd)
  if not snd then
    return
  end
  if val then
    obj:setVolume(snd, 1)
    obj:playSFX(snd)
  else
    obj:stopSFX(snd)
  end
end

local function updateIgnitionStarter(dt)
  if ignitionHoldingStarter then
    ignitionHoldingStarterTimer = ignitionHoldingStarterTimer + dt
    if ignitionHoldingStarterTimer > 0.5 then
      if not ignitionWasStartingEngine then
        ignitionWasStartingEngine = true
        M.setIgnitionLevel(3)
      end
    else
      ignitionWasStartingEngine = false
    end
  else
    ignitionHoldingStarterTimer = 0
  end
end

local function updateGFX(dt)
  updateIgnitionStarter(dt)
  generateBlinkPulse(dt)

  local vals = M.values
  -- the primary source values

  automatic_indicator_stop()

  vals.accXSmooth = sensors.gx2
  vals.accYSmooth = sensors.gy2
  vals.accZSmooth = sensors.gz2

  vals.odometer = partCondition.getRootPartOdometerValue()
  vals.trip = partCondition.getRootPartTripValue()

  vals.brakelights = nil
  vals.nop = 0 --nop electrics for not yet working things
  vals.parkingbrake = vals.parkingbrake_input
  vals.parkingbrakelight = vals.parkingbrake > 0
  vals.lights = lightsState
  vals.lights_state = lightsState
  if signalWarnState then
    vals.turnsignal = 0
  elseif signalRightState then
    vals.turnsignal = 1
  elseif signalLeftState then
    vals.turnsignal = -1
  else
    vals.turnsignal = 0
  end

  vals.airspeed = obj:getGroundSpeed()
  vals.airflowspeed = obj:getAirflowSpeed()
  vals.altitude = obj:getAltitude()
  vals.parking = 0 -- TODO: input.parkinglights
  vals.reverse = (vals.gearIndex or 0) < 0

  -- and then the derived values
  vals.signal_L = vals.signal_left_input == 1 and blinkPulse
  vals.signal_R = vals.signal_right_input == 1 and blinkPulse

  vals.hazard = (signalWarnState and blinkPulse)
  vals.hazard_enabled = signalWarnState
  vals.lightbar = lightbarState
  vals.lowpressure = (beamstate.lowpressure)
  vals.oil = (vals.oiltemp or 0) >= 130
  vals.lowhighbeam = (lightsState == 1 or lightsState == 2)
  vals.lowbeam = (lightsState == 1)
  vals.highbeam = (lightsState == 2)
  vals.fog = fogLightsState
  vals.horn = hornState

  --mixed values for american style indicators/lights
  vals.lowhighbeam_signal_R = vals.signal_right_input == 1 and (blinkPulse and 1 or 0) or ceil(vals.lowhighbeam and 1 or 0)
  vals.lowhighbeam_signal_L = vals.signal_left_input == 1 and (blinkPulse and 1 or 0) or ceil(vals.lowhighbeam and 1 or 0)
  --wigwag lights
  --desired behavior: highbeam is controlled by normal highbeam values if lightbar is OFF
  --if it's on, only the wigwag signal has control over the highbeam
  local lightbarActive = vals.lightbar > 0
  local wigwagRActive = vals.wigwag_R == 1
  local wigwagLActive = vals.wigwag_L == 1
  local highbeamActive = vals.highbeam
  vals.highbeam_wigwag_R = ((highbeamActive and not lightbarActive) or (wigwagRActive)) and 1 or 0
  vals.highbeam_wigwag_L = ((highbeamActive and not lightbarActive) or (wigwagLActive)) and 1 or 0
  vals.reverse_wigwag_R = vals.wigwag_R == 1 or ceil(vals.reverse and 1 or 0)
  vals.reverse_wigwag_L = vals.wigwag_L == 1 or ceil(vals.reverse and 1 or 0)

  local rpm = vals.rpm
  vals.rpmTacho = rpmSmoother:get(rpm, dt)
  vals.rpmspin = (vals.rpmspin + dt * rpm * 6) % 360 --make sure to convert properly between the units here

  vals.signal_right_input = (signalRightState)
  vals.signal_left_input = (signalLeftState)

  vals.boost = (vals.turboBoost or 0) + (vals.superchargerBoost or 0)
  vals.boostMax = max((vals.turboBoostMax or 0), (vals.superchargerBoostMax or 0))

  -- inject imported electrics events first time, this needs to happen twice overall so that code between gfx first step and gfx second step can see these updated electrics
  beamstate.updateRemoteElectrics(true)

  for f, v in pairs(vals) do
    if M.disabledState[f] ~= nil then
      vals[f] = nil
    else
      if type(v) == "boolean" then
        vals[f] = vals[f] and 1 or 0
      end
    end
  end

  for f, s in pairs(smoothers) do
    if vals[f] ~= nil then
      vals[f] = s:get(vals[f])
    end
  end
end

local function updateGFXSecondStep(dt)
  local values = M.values
  --make sure to update the brakelights value based on the brake value
  values.brakelights = values.brakelights or ceil(values.brake)
  values.brakelight_signal_R = values.signal_right_input == 1 and (blinkPulse and 1 or 0) or ceil(values.brakelights or values.brake or 0)
  values.brakelight_signal_L = values.signal_left_input == 1 and (blinkPulse and 1 or 0) or ceil(values.brakelights or values.brake or 0)

  -- inject imported electrics events second time, this should be the last step before update electrics by ignition state so that we can override everything if needed
  beamstate.updateRemoteElectrics(false)

  updateElectricsWithIgnitionLevel()
end

local function updateElectricsWithIgnitionLevelEuropean()
  local values = M.values
  if values.ignitionLevel == 0 then
    --adjustments in "off" mode
    values.brakelights = 0
    values.parkingbrakelight = 0
    values.brakelight_signal_R = 0
    values.brakelight_signal_L = 0

    values.parking = 0
    values.reverse = 0

    values.lights = 0
    values.lights_state = 0

    values.turnsignal = 0
    values.signal_L = 0
    values.signal_R = 0
    values.hazard = 0
    values.hazard_enabled = 0
    values.lightbar = 0
    lightbarState = 0
    values.lowpressure = 0
    values.oil = 0
    values.lowhighbeam = 0
    values.lowbeam = 0
    values.highbeam = 0
    values.fog = 0
    values.horn = 0
    values.lowhighbeam_signal_R = 0
    values.lowhighbeam_signal_L = 0
    values.highbeam_wigwag_R = 0
    values.highbeam_wigwag_L = 0
    values.reverse_wigwag_R = 0
    values.reverse_wigwag_L = 0
    values.signal_right_input = 0
    values.signal_left_input = 0

    signalLeftState = false
    signalRightState = false
    signalWarnState = false

    automatic_indicator_stop = nop
    generateBlinkPulse = nop

    values.fuel = 0
    values.lowfuel = 0

    values.abs = 0
    values.esc = 0
    values.tcs = 0

    values.oiltemp = 0
    values.watertemp = 0
    values.checkengine = 0
    values.ignition = 0
    values.running = 0
    values.engineRunning = 0
  elseif values.ignitionLevel == 1 then
    --adjustments in "accessory" mode
  else
    --adjustments in "on" mode
  end
end

local function sanitizeIgnitionLevel(desiredIgnitionLevel)
  --return the _actual_ ignition level for a desired one so that certain levels can be "forbidden"
  return ignitionLevelSanitization[desiredIgnitionLevel] or 0
end

local function setAllowedIgnitionLevels(allowedIgnitionLevels)
  if #allowedIgnitionLevels <= 0 then --make sure that we always have at least one allowed ignition level
    allowedIgnitionLevels = {0}
  --print warning
  end
  local allowedIgnitionLevelsLookup = {}
  for _, level in ipairs(allowedIgnitionLevels) do
    allowedIgnitionLevelsLookup[level] = true
  end

  ignitionLevelSanitization = {[0] = 0, [1] = 1, [2] = 2, [3] = 3}

  for i = 0, 3, 1 do --for every possible ignition level
    if not allowedIgnitionLevelsLookup[i] then --check if it's currently allowed
      for j = min(i + 1, 3), 3, 1 do --if not, first try to find anything _higher_ that is allowed
        if allowedIgnitionLevelsLookup[j] then
          ignitionLevelSanitization[i] = j
          break
        end
      end

      if not allowedIgnitionLevelsLookup[i] then --if we couldn't find anything higher allowed
        for j = max(i - 1, 0), 0, -1 do --try to find anything lower that is allowed...
          if allowedIgnitionLevelsLookup[j] then
            ignitionLevelSanitization[i] = j
            break
          end
        end
      end
    end
  end
end

local function reset()
  M.disabledState = {}

  for _, s in pairs(smoothers) do
    s:set(0)
  end

  M.values.throttle = 0
  M.values.brake = 0
  M.values.steering = 0
  M.values.clutch = 0
  M.values.wheelspeed = 0
  M.values.odometer = 0
  M.values.avgWheelAV = 0
  M.values.airspeed = 0
  M.values.airflowspeed = 0
  M.values.horn = false
  M.values.boost = 0
  M.values.boostMax = 0
  
  --- ALESSIO added this one
  M.values.wheel_angle = 0

  lightbarState = 0
  lightsSavedState = 0

  local allowedIgnitionLevels = (v.data.electrics and v.data.electrics.allowedIgnitionLevels) or {0, 1, 2, 3} --read allowed ingition levels from jbeam or use all of them by default
  setAllowedIgnitionLevels(allowedIgnitionLevels)

  ignitionHoldingStarter = false
  ignitionHoldingStarterTimer = 0
  ignitionWasStartingEngine = false

  local spawnVehicleIgnitionLevel = settings.getValue("spawnVehicleIgnitionLevel") or 3
  if v.config.additionalVehicleData and v.config.additionalVehicleData.spawnWithEngineRunning ~= nil then
    spawnVehicleIgnitionLevel = v.config.additionalVehicleData.spawnWithEngineRunning and 3 or 0
  end
  M.values.ignitionLevel = sanitizeIgnitionLevel(spawnVehicleIgnitionLevel)
  previousIgnitionLevel = spawnVehicleIgnitionLevel - 1

  local ignitionLevelOverrideType = (v.data.electrics and v.data.electrics.ignitionLevelOverrideType) or "european"
  if ignitionLevelOverrideType == "european" then
    updateElectricsWithIgnitionLevel = updateElectricsWithIgnitionLevelEuropean
  elseif ignitionLevelOverrideType == "none" then
    updateElectricsWithIgnitionLevel = nop
  end
end

local function init()
  reset()
end

local function initLastStage()
  if M.values.ignitionLevel == 3 then
    M.values.ignitionLevel = 2
    previousIgnitionLevel = 3
  end
end

local function set_warn_signal(value)
  signalWarnState = value
  signalRightState = signalWarnState
  signalLeftState = signalWarnState
  automatic_indicator_stop = nop
  updateSignals()
end

local function toggle_warn_signal()
  set_warn_signal(not signalWarnState)
end

local function toggle_lights()
  lightsState = lightsState + 1
  if lightsState == 1 then
    lightOn = lightOn or sounds.createSoundscapeSound("LightOn")
    sounds.playSoundSkipAI(lightOn)
  elseif lightsState == 2 then
    lightOn = lightOn or sounds.createSoundscapeSound("LightOn")
    sounds.playSoundSkipAI(lightOn)
  elseif lightsState == 3 then
    lightsState = 0
    lightOff = lightOff or sounds.createSoundscapeSound("LightOff")
    sounds.playSoundSkipAI(lightOff)
  end
end

local function light_flash_highbeams(enabled)
  if enabled then
    lightsSavedState = lightsState
    lightsState = 2
  else
    lightsState = lightsSavedState
  end
end

local function set_lightbar_signal(state)
  if state == lightbarState then
    return
  end

  sirenSound = sirenSound or sounds.createSoundscapeSound("siren")
  lightbarState = state % (sirenSound ~= nil and 3 or 2)

  -- 1 = lights, no sound
  -- 2 = lights + sound
  toggleSound(lightbarState == 2, sirenSound)
end

local function toggle_lightbar_signal()
  set_lightbar_signal(lightbarState + 1)
end

local function setLightsState(newval)
  lightsState = newval
end

local function toggle_fog_lights()
  fogLightsState = not fogLightsState
end

local function set_fog_lights(state)
  fogLightsState = state
end

local function horn(state)
  hornState = state
  if hornState then
    hornSound1 = hornSound1 or sounds.createSoundscapeSound("horn")
    if not hornSound1 then return end
    if hornSound2 then
      obj:setVolume(hornSound2, 0)
    end
    obj:setVolume(hornSound1, 1)
    obj:playSFX(hornSound1)
  else
    if hornSound1 then obj:stopSFX(hornSound1) end
    hornSound1, hornSound2 = hornSound2, hornSound1
  end
end

local function setIgnitionLevel(ignitionLevel)
  --print("Ignition Level: " .. ignitionLevel)
  previousIgnitionLevel = M.values.ignitionLevel
  local newIgnitionLevel = sanitizeIgnitionLevel(ignitionLevel)
  M.values.ignitionLevel = newIgnitionLevel
  if ignitionMessageLookup[newIgnitionLevel] then
    guihooks.message({txt = ignitionMessageLookup[newIgnitionLevel], context = {}}, 4, "vehicle.ignition.ignitionLevel")
  end

  if newIgnitionLevel == 0 or newIgnitionLevel == 1 then
    controller.mainController.setEngineIgnition(false)
    controller.mainController.setStarter(false)
  elseif newIgnitionLevel == 2 then
    controller.mainController.setEngineIgnition(true)
    controller.mainController.setStarter(false)
  elseif newIgnitionLevel == 3 then
    controller.mainController.setEngineIgnition(true)
    controller.mainController.setStarter(true)
  end
end

local function toggleIgnitionLevelOnDown()
  ignitionHoldingStarter = true
end

local function toggleIgnitionLevelOnUp()
  if ignitionWasStartingEngine and M.values.ignitionLevel < 3 then
    --this case happens when the vehicle controller already switched to level 2 by itself after the engine fired up
    --make sure to pretend that we are still in level 3 so that the following logic works. Not nice, but couldn't find a better solution that actually works
    --this logic is also needed so that we can set level 3 from external (eg GE) and have the engine start autoamtically and go back to level 2 after doing so
    --(vehicle controller switches to level 2 after engine fired up)
    setIgnitionLevel(3)
  end
  local currentIgnitionLevel = M.values.ignitionLevel
  ignitionHoldingStarter = false
  ignitionWasStartingEngine = false

  if currentIgnitionLevel == 0 then
    setIgnitionLevel(1)
  elseif currentIgnitionLevel == 1 then
    if previousIgnitionLevel < currentIgnitionLevel then
      setIgnitionLevel(2)
    else
      setIgnitionLevel(0)
    end
  elseif currentIgnitionLevel == 2 then
    setIgnitionLevel(1)
  elseif currentIgnitionLevel == 3 then
    setIgnitionLevel(2)
  end
end

-- public interface
M.updateGFX = updateGFX
M.updateGFXSecondStep = updateGFXSecondStep
M.toggle_left_signal = toggle_left_signal
M.toggle_right_signal = toggle_right_signal
M.stop_turn_signal = stop_turn_signal
M.toggle_warn_signal = toggle_warn_signal
M.set_warn_signal = set_warn_signal
M.toggle_lightbar_signal = toggle_lightbar_signal
M.set_lightbar_signal = set_lightbar_signal
M.toggle_fog_lights = toggle_fog_lights
M.set_fog_lights = set_fog_lights
M.toggle_lights = toggle_lights
M.light_flash_highbeams = light_flash_highbeams
M.setLightsState = setLightsState
M.horn = horn
M.setIgnitionLevel = setIgnitionLevel
M.toggleIgnitionLevelOnDown = toggleIgnitionLevelOnDown
M.toggleIgnitionLevelOnUp = toggleIgnitionLevelOnUp
M.setAllowedIgnitionLevels = setAllowedIgnitionLevels
M.resetLastStage = initLastStage
M.reset = reset
M.init = init
M.initLastStage = initLastStage
return M
