import React from 'react';
import PropTypes from 'prop-types';

const StringOption = ({ value, onChange }) => (
  <input
    className="valid"
    type="text"
    value={value}
    onChange={evt => onChange(evt.target.value)}
  />
);

StringOption.propTypes = {
  value: PropTypes.string.isRequired,
  onChange: PropTypes.func.isRequired
};

export default StringOption;
