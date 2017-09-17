export const RECEIVE_CONFIG = 'RECEIVE_CONFIG';
export const GET_CONFIG = 'GET_CONFIG';
export const UPDATE_CONFIG = 'UPDATE_CONFIG';
export const UPDATE_OPTION_VALUE = 'UPDATE_OPTION_VALUE';

export const receiveConfig = (config) => ({
  type: RECEIVE_CONFIG,
  data: config
});

export const updateOptionValue = (optionGroup, option, newValue) => ({
  type: UPDATE_OPTION_VALUE,
  optionGroup,
  option,
  newValue
});

export const getConfig = () => ({
  type: GET_CONFIG
});

export const updateConfig = () => (
  (dispatch, getState) => (
    dispatch({
      type: UPDATE_CONFIG,
      data: getState().config
    })
  )
);
