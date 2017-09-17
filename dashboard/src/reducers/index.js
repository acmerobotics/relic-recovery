import { combineReducers } from 'redux';
import telemetry from './telemetry';
import socket from './socket';
import config from './config';
import fieldOverlay from './fieldOverlay';

export default combineReducers({
  telemetry,
  socket,
  config,
  fieldOverlay
});
