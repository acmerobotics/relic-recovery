import React from 'react';
import PropTypes from 'prop-types';
import DoubleOption from './DoubleOption';

const PIDOption = ({ value, onChange }) => (
  <table className="valid">
    <tbody>
      <tr>
        <td>P: </td>
        <td>
          <DoubleOption value={value.p} onChange={p => onChange({
            ...value,
            p
          })} />
        </td>
      </tr>
      <tr>
        <td>I: </td>
        <td>
          <DoubleOption value={value.i} onChange={i => onChange({
            ...value,
            i
          })} />
        </td>
      </tr>
      <tr>
        <td>D: </td>
        <td>
          <DoubleOption value={value.d} onChange={d => onChange({
            ...value,
            d
          })} />
        </td>
      </tr>
    </tbody>
  </table>
);

PIDOption.propTypes = {
  value: PropTypes.shape({
    p: PropTypes.oneOfType([
      PropTypes.number,
      PropTypes.string
    ]).isRequired,
    i: PropTypes.oneOfType([
      PropTypes.number,
      PropTypes.string
    ]).isRequired,
    d: PropTypes.oneOfType([
      PropTypes.number,
      PropTypes.string
    ]).isRequired
  }).isRequired,
  onChange: PropTypes.func.isRequired
};

export default PIDOption;
