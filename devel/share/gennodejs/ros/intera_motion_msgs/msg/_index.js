
"use strict";

let Trajectory = require('./Trajectory.js');
let WaypointOptions = require('./WaypointOptions.js');
let TrajectoryOptions = require('./TrajectoryOptions.js');
let TrackingOptions = require('./TrackingOptions.js');
let MotionStatus = require('./MotionStatus.js');
let Waypoint = require('./Waypoint.js');
let TrajectoryAnalysis = require('./TrajectoryAnalysis.js');
let JointTrackingError = require('./JointTrackingError.js');
let EndpointTrackingError = require('./EndpointTrackingError.js');
let InterpolatedPath = require('./InterpolatedPath.js');
let WaypointSimple = require('./WaypointSimple.js');
let MotionCommandActionGoal = require('./MotionCommandActionGoal.js');
let MotionCommandActionResult = require('./MotionCommandActionResult.js');
let MotionCommandGoal = require('./MotionCommandGoal.js');
let MotionCommandFeedback = require('./MotionCommandFeedback.js');
let MotionCommandActionFeedback = require('./MotionCommandActionFeedback.js');
let MotionCommandAction = require('./MotionCommandAction.js');
let MotionCommandResult = require('./MotionCommandResult.js');

module.exports = {
  Trajectory: Trajectory,
  WaypointOptions: WaypointOptions,
  TrajectoryOptions: TrajectoryOptions,
  TrackingOptions: TrackingOptions,
  MotionStatus: MotionStatus,
  Waypoint: Waypoint,
  TrajectoryAnalysis: TrajectoryAnalysis,
  JointTrackingError: JointTrackingError,
  EndpointTrackingError: EndpointTrackingError,
  InterpolatedPath: InterpolatedPath,
  WaypointSimple: WaypointSimple,
  MotionCommandActionGoal: MotionCommandActionGoal,
  MotionCommandActionResult: MotionCommandActionResult,
  MotionCommandGoal: MotionCommandGoal,
  MotionCommandFeedback: MotionCommandFeedback,
  MotionCommandActionFeedback: MotionCommandActionFeedback,
  MotionCommandAction: MotionCommandAction,
  MotionCommandResult: MotionCommandResult,
};
