import React from 'react';
import { connect } from 'react-redux';
import PropTypes from 'prop-types';
import Heading from '../components/Heading';
import Icon from '../components/Icon';
import BooleanInput from '../components/inputs/BooleanInput';
import EnumInput from '../components/inputs/EnumInput';
import StringInput from '../components/inputs/StringInput';
import IntInput from '../components/inputs/IntInput';
import DoubleInput from '../components/inputs/DoubleInput';
import { updateOptionValue } from '../actions/config';

export const OptionType = {
  BOOLEAN: 'boolean',
  INT: 'int',
  DOUBLE: 'double',
  STRING: 'string',
  ENUM: 'enum',
  CUSTOM: 'custom'
};

class Option extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      expanded: false
    };

    this.toggleVisibility = this.toggleVisibility.bind(this);
  }

  toggleVisibility() {
    this.setState({
      expanded: !this.state.expanded
    });
  }

  render() {
    const { name, value, schema, onChange} = this.props;

    if (!value) {
      return null;
    }

    let type;
    if (typeof schema === 'object') {
      if ('type' in schema) {
        type = schema.type;
      } else {
        type = OptionType.CUSTOM;
      }
    } else {
      type = schema;
    }

    if (type === OptionType.CUSTOM) {
      const options = Object.keys(value).map((key) => ((
        <Option
          key={key}
          name={key}
          value={value[key]}
          schema={schema[key]} />
      )));

      return (
        <tr>
          <td>
            <Heading level={3} text={name} >
              <Icon icon={ this.state.expanded ? 'remove' : 'add' } size='small' onClick={this.toggleVisibility} />
            </Heading>
            {
              this.state.expanded ?
                (
                  <table>
                    <tbody>{options}</tbody>
                  </table>
                )
                : null
            }
          </td>
        </tr>
      );
    }

    let input;

    switch (type) {
    case OptionType.INT:
      input = <IntInput value={value} onChange={onChange} />;
      break;
    case OptionType.DOUBLE:
      input = <DoubleInput value={value} onChange={onChange} />;
      break;
    case OptionType.STRING:
      input = <StringInput value={value} onChange={onChange} />;
      break;
    case OptionType.BOOLEAN:
      input = <BooleanInput value={value} onChange={onChange} />;
      break;
    case OptionType.ENUM:
      input = <EnumInput value={value} values={schema.values} onChange={onChange} />;
      break;
    default:
      input = <p>Unknown option type: {type}</p>;
    }

    return (
      <tr>
        <td>{name}</td>
        <td>{input}</td>
      </tr>
    );
  }
}

Option.propTypes = {
  name: PropTypes.string.isRequired,
  schema: PropTypes.any.isRequired,
  value: PropTypes.any,
  onChange: PropTypes.func // TODO: fix!
};

const mapDispatchToProps = (dispatch, ownProps) => ({
  onChange: (newValue) => {
    dispatch(updateOptionValue(ownProps.optionGroup, ownProps.option, newValue));
  }
});

export default connect(null, mapDispatchToProps)(Option);
