-- RoboMaster EP real-physics pick-place, stable timed version.
-- No fake attaching. The cube only moves if the gripper really clamps it.
--
-- Servo mapping from the current CoppeliaSim scene:
-- servo 0: horizontal arm link
-- servo 1: vertical arm link
-- gripper: simRobomaster.set_gripper_target(...)

local simRobomaster = require('simRobomaster')

local robot
local rm
local box
local placeMarker

local step = 0
local stepStarted = false
local stepStartTime = 0

local cubeSize = 0.022
local cubeHalf = cubeSize / 2.0

-- Cube position. If the gripper reaches the wrong front/back point, tune cubeX.
local cubeX = 0.120
local cubeY = 0.000

-- Right turn 90 degrees.
local rightTurnRad = -1.5708

-- Servo angles in radians.
-- First use servo1 to aim above the cube, then use servo0 to lower.
local homeServo0 = -0.08
local homeServo1 = 0.00

local aboveServo1 = 0.65
local lowerServo0 = 0.58

local liftServo0 = 0.12
local liftServo1 = 0.55

local armSpeed = 4
local driveSpeed = 0.04
local turnSpeed = 0.10

local function elapsed()
    return sim.getSimulationTime() - stepStartTime
end

local function startStep(label)
    stepStarted = true
    stepStartTime = sim.getSimulationTime()
    sim.addLog(sim.verbosity_scriptinfos, 'STEP ' .. tostring(step) .. ': ' .. label)
end

local function nextStep()
    step = step + 1
    stepStarted = false
end

local function after(seconds)
    return elapsed() > seconds
end

local function setShapePhysical(shape, isStatic, respondable)
    pcall(sim.setObjectInt32Param, shape, sim.shapeintparam_static, isStatic and 1 or 0)
    pcall(sim.setObjectInt32Param, shape, sim.shapeintparam_respondable, respondable and 1 or 0)
end

local function getOrCreateCube()
    local ok, obj = pcall(sim.getObject, '/target_cube')
    if ok then
        return obj
    end

    obj = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {cubeSize, cubeSize, cubeSize})
    sim.setObjectAlias(obj, 'target_cube')
    sim.setShapeColor(obj, nil, sim.colorcomponent_ambient_diffuse, {1.0, 0.0, 0.0})
    return obj
end

local function getOrCreatePlaceMarker()
    local ok, obj = pcall(sim.getObject, '/place_marker')
    if ok then
        return obj
    end

    obj = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {0.070, 0.070, 0.006})
    sim.setObjectAlias(obj, 'place_marker')
    sim.setShapeColor(obj, nil, sim.colorcomponent_ambient_diffuse, {0.0, 1.0, 0.0})
    return obj
end

local function moveServo(servo, angle, label)
    sim.addLog(sim.verbosity_scriptinfos,
        string.format('%s: servo%d -> %.3f rad', label, servo, angle))
    simRobomaster.move_servo(rm, servo, angle)
end

function sysCall_init()
    robot = sim.getObject('/RoboMaster')
    rm = simRobomaster.create_ep(robot)

    box = getOrCreateCube()
    placeMarker = getOrCreatePlaceMarker()

    sim.setObjectParent(box, -1, true)
    sim.setObjectPosition(box, -1, {cubeX, cubeY, cubeHalf + 0.001})
    sim.setObjectOrientation(box, -1, {0.0, 0.0, 0.0})

    setShapePhysical(box, false, true)
    setShapePhysical(placeMarker, true, false)
    sim.setObjectPosition(placeMarker, -1, {0.0, -0.16, 0.004})

    simRobomaster.set_arm_servos_max_speed(rm, armSpeed)
    simRobomaster.set_gripper_target(rm, 'open', 1.0)

    step = 0
    stepStarted = false

    sim.addLog(sim.verbosity_scriptinfos,
        string.format('Stable sequence. cubeX=%.3f aboveServo1=%.3f lowerServo0=%.3f',
        cubeX, aboveServo1, lowerServo0))
end

function sysCall_actuation()
    if step < 5 then
        simRobomaster.set_gripper_target(rm, 'open', 1.0)
    end

    if step == 0 then
        if not stepStarted then
            startStep('open gripper and set home servo0')
            moveServo(0, homeServo0, 'home horizontal arm')
        elseif after(2.5) then
            nextStep()
        end

    elseif step == 1 then
        if not stepStarted then
            startStep('set home servo1')
            moveServo(1, homeServo1, 'home vertical arm')
        elseif after(2.5) then
            nextStep()
        end

    elseif step == 2 then
        if not stepStarted then
            startStep('servo1 moves gripper above cube')
            moveServo(1, aboveServo1, 'aim above cube')
        elseif after(4.0) then
            nextStep()
        end

    elseif step == 3 then
        if not stepStarted then
            startStep('servo0 lowers gripper')
            moveServo(0, lowerServo0, 'lower to cube')
        elseif after(5.0) then
            nextStep()
        end

    elseif step == 4 then
        if not stepStarted then
            startStep('pause at lowest point, gripper still open')
        elseif after(2.0) then
            nextStep()
        end

    elseif step == 5 then
        if not stepStarted then
            startStep('close gripper')
            simRobomaster.set_gripper_target(rm, 'close', 1.0)
        elseif after(3.0) then
            nextStep()
        end

    elseif step == 6 then
        if not stepStarted then
            startStep('lift with servo0')
            moveServo(0, liftServo0, 'lift horizontal arm')
        elseif after(4.0) then
            nextStep()
        end

    elseif step == 7 then
        if not stepStarted then
            startStep('lift with servo1')
            moveServo(1, liftServo1, 'lift vertical arm')
        elseif after(4.0) then
            nextStep()
        end

    elseif step == 8 then
        if not stepStarted then
            startStep('turn right 90 degrees')
            simRobomaster.move_to(rm, {x = 0.0, y = 0.0, theta = rightTurnRad}, driveSpeed, turnSpeed)
        elseif after(11.0) then
            nextStep()
        end

    elseif step == 9 then
        if not stepStarted then
            startStep('lower for release')
            moveServo(0, lowerServo0, 'lower for release')
        elseif after(5.0) then
            nextStep()
        end

    elseif step == 10 then
        if not stepStarted then
            startStep('open gripper to release')
            simRobomaster.set_gripper_target(rm, 'open', 1.0)
        elseif after(2.5) then
            nextStep()
        end

    elseif step == 11 then
        if not stepStarted then
            startStep('return servo0 home')
            moveServo(0, homeServo0, 'return horizontal arm')
        elseif after(4.0) then
            nextStep()
        end

    elseif step == 12 then
        if not stepStarted then
            startStep('return servo1 home')
            moveServo(1, homeServo1, 'return vertical arm')
        elseif after(4.0) then
            nextStep()
        end

    elseif step == 13 then
        -- Done.
    end
end
