import React from 'react';
import { connect } from 'react-redux';
import PropTypes from 'prop-types';
import BooleanOption from '../components/options/BooleanOption';
import PIDOption from '../components/options/PIDOption';
import EnumOption from '../components/options/EnumOption';
import StringOption from '../components/options/StringOption';
import IntOption from '../components/options/IntOption';
import DoubleOption from '../components/options/DoubleOption';
import { updateOptionValue } from '../actions/config';

export const ConfigOptionType = {
  BOOLEAN: 'boolean',
  INT: 'int',
  DOUBLE: 'double',
  STRING: 'string',
  ENUM: 'enum',
  PID: 'pid'
};

class ConfigOption extends React.Component {
  renderOption() {
    const { option } = this.props;

    switch (option.type) {
    case ConfigOptionType.INT:
      return <IntOption value={option.value} onChange={this.props.onChange} />;
    case ConfigOptionType.DOUBLE:
      return <DoubleOption value={option.value} onChange={this.props.onChange} />;
    case ConfigOptionType.STRING:
      return <StringOption value={option.value} onChange={this.props.onChange} />;
    case ConfigOptionType.BOOLEAN:
      return <BooleanOption value={option.value} onChange={this.props.onChange} />;
    case ConfigOptionType.ENUM:
      return <EnumOption value={option.value} values={option.values} onChange={this.props.onChange} />;
    case ConfigOptionType.PID:
      return <PIDOption value={option.value} onChange={this.props.onChange} />;
    default:
      return <p>Unknown option type: {option.type}</p>;
    }
  }

  render() {
    return (
      <tr>
        <td>{this.props.option.name}</td>
        <td>{this.renderOption()}</td>
      </tr>
    );
  }
}

ConfigOption.propTypes = {
  option: PropTypes.shape({
    type: PropTypes.string.isRequired,
    name: PropTypes.string.isRequired,
    value: PropTypes.any.isRequired,
    values: PropTypes.arrayOf(PropTypes.string)
  }).isRequired,
  optionGroup: PropTypes.string.isRequired,
  onChange: PropTypes.func.isRequired
};

const mapDispatchToProps = (dispatch, ownProps) => ({
  onChange: (newValue) => {
    dispatch(updateOptionValue(ownProps.optionGroup, ownProps.option, newValue));
  }
});

export default connect(null, mapDispatchToProps)(ConfigOption);
