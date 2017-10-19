import React from 'react';
import PropTypes from 'prop-types';

const StringInput = ({ value, onChange }) => (
  <input
    className="valid"
    type="text"
    value={value}
    onChange={evt => onChange(evt.target.value)}
  />
);

StringInput.propTypes = {
  value: PropTypes.string.isRequired,
  onChange: PropTypes.func // TODO: fix!
};

export default StringInput;
