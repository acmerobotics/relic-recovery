import { RECEIVE_TELEMETRY } from '../actions/telemetry';

const initialState = {
  timestamp: 0,
  captionValueSeparator: '',
  itemSeparator: '',
  lines: [],
  log: {
    lines: []
  }
};

const telemetry = (state = initialState, action) => {
  switch (action.type) {
  case RECEIVE_TELEMETRY:
    return action.data;
  default:
    return state;
  }
};

export const getAllItems = (telemetry) => {
  return telemetry.lines.reduce((a, b) => (
    a.concat(b.items)
  ), []);
};

export default telemetry;
