import React from 'react';
import PropTypes from 'prop-types';

const BooleanOption = ({ value, onChange }) => (
  <input type="checkbox" value={value} onChange={evt => onChange(evt.target.checked)} />
);

BooleanOption.propTypes = {
  value: PropTypes.bool.isRequired,
  onChange: PropTypes.func.isRequired
};

export default BooleanOption;
