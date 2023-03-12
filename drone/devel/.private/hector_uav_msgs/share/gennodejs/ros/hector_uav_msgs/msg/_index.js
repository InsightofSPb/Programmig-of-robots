
"use strict";

let Altimeter = require('./Altimeter.js');
let Compass = require('./Compass.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let RawRC = require('./RawRC.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let MotorCommand = require('./MotorCommand.js');
let HeadingCommand = require('./HeadingCommand.js');
let RC = require('./RC.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let YawrateCommand = require('./YawrateCommand.js');
let Supply = require('./Supply.js');
let RawImu = require('./RawImu.js');
let ControllerState = require('./ControllerState.js');
let RawMagnetic = require('./RawMagnetic.js');
let ThrustCommand = require('./ThrustCommand.js');
let RuddersCommand = require('./RuddersCommand.js');
let HeightCommand = require('./HeightCommand.js');
let MotorStatus = require('./MotorStatus.js');
let MotorPWM = require('./MotorPWM.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let ServoCommand = require('./ServoCommand.js');
let PoseGoal = require('./PoseGoal.js');
let PoseActionGoal = require('./PoseActionGoal.js');
let TakeoffActionResult = require('./TakeoffActionResult.js');
let TakeoffActionGoal = require('./TakeoffActionGoal.js');
let TakeoffActionFeedback = require('./TakeoffActionFeedback.js');
let TakeoffResult = require('./TakeoffResult.js');
let PoseActionResult = require('./PoseActionResult.js');
let PoseFeedback = require('./PoseFeedback.js');
let LandingActionFeedback = require('./LandingActionFeedback.js');
let LandingFeedback = require('./LandingFeedback.js');
let LandingAction = require('./LandingAction.js');
let LandingGoal = require('./LandingGoal.js');
let TakeoffFeedback = require('./TakeoffFeedback.js');
let TakeoffGoal = require('./TakeoffGoal.js');
let PoseAction = require('./PoseAction.js');
let LandingActionGoal = require('./LandingActionGoal.js');
let TakeoffAction = require('./TakeoffAction.js');
let PoseActionFeedback = require('./PoseActionFeedback.js');
let LandingResult = require('./LandingResult.js');
let PoseResult = require('./PoseResult.js');
let LandingActionResult = require('./LandingActionResult.js');

module.exports = {
  Altimeter: Altimeter,
  Compass: Compass,
  VelocityXYCommand: VelocityXYCommand,
  RawRC: RawRC,
  PositionXYCommand: PositionXYCommand,
  MotorCommand: MotorCommand,
  HeadingCommand: HeadingCommand,
  RC: RC,
  AttitudeCommand: AttitudeCommand,
  YawrateCommand: YawrateCommand,
  Supply: Supply,
  RawImu: RawImu,
  ControllerState: ControllerState,
  RawMagnetic: RawMagnetic,
  ThrustCommand: ThrustCommand,
  RuddersCommand: RuddersCommand,
  HeightCommand: HeightCommand,
  MotorStatus: MotorStatus,
  MotorPWM: MotorPWM,
  VelocityZCommand: VelocityZCommand,
  ServoCommand: ServoCommand,
  PoseGoal: PoseGoal,
  PoseActionGoal: PoseActionGoal,
  TakeoffActionResult: TakeoffActionResult,
  TakeoffActionGoal: TakeoffActionGoal,
  TakeoffActionFeedback: TakeoffActionFeedback,
  TakeoffResult: TakeoffResult,
  PoseActionResult: PoseActionResult,
  PoseFeedback: PoseFeedback,
  LandingActionFeedback: LandingActionFeedback,
  LandingFeedback: LandingFeedback,
  LandingAction: LandingAction,
  LandingGoal: LandingGoal,
  TakeoffFeedback: TakeoffFeedback,
  TakeoffGoal: TakeoffGoal,
  PoseAction: PoseAction,
  LandingActionGoal: LandingActionGoal,
  TakeoffAction: TakeoffAction,
  PoseActionFeedback: PoseActionFeedback,
  LandingResult: LandingResult,
  PoseResult: PoseResult,
  LandingActionResult: LandingActionResult,
};
