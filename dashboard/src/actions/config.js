export const RECEIVE_CONFIG = 'RECEIVE_CONFIG';
export const GET_CONFIG = 'GET_CONFIG';
export const UPDATE_CONFIG = 'UPDATE_CONFIG';

export const receiveConfig = (config) => ({
  type: RECEIVE_CONFIG,
  data: config
});

export const getConfig = () => ({
  type: GET_CONFIG
});

export const updateConfig = (update) => (
  (dispatch) => (
    dispatch({
      type: UPDATE_CONFIG,
      data: update
    })
  )
);
