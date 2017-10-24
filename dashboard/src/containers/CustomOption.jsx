import React from 'react';
import { connect } from 'react-redux';
import PropTypes from 'prop-types';
import Heading from '../components/Heading';
import Icon from '../components/Icon';
import { updateOptionValue } from '../actions/config';
import BasicOption from './BasicOption';
import OptionType from '../enums/OptionType';

class CustomOption extends React.Component {
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
    const { name, value, schema } = this.props;

    const options = Object.keys(value)
      .filter((key) => key in schema)
      .map((key) => {
        const onUpdate = (update) => this.props.onUpdate({
          [key]: update
        });

        const type = OptionType.getFromSchema(schema[key]);

        if (type == OptionType.CUSTOM) {
          return (
            <CustomOption
              key={key}
              name={key}
              value={value[key]}
              schema={schema[key]}
              onUpdate={onUpdate} />
          );
        }

        return (
          <BasicOption
            key={key}
            name={key}
            value={value[key]}
            schema={schema[key]}
            onUpdate={onUpdate} />
        );
      });

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
}

CustomOption.propTypes = {
  name: PropTypes.string.isRequired,
  schema: PropTypes.any.isRequired,
  value: PropTypes.any.isRequired,
  onUpdate: PropTypes.func.isRequired
};

const mapDispatchToProps = (dispatch, ownProps) => ({
  onChange: (newValue) => {
    dispatch(updateOptionValue(ownProps.optionGroup, ownProps.option, newValue));
  }
});

export default connect(null, mapDispatchToProps)(CustomOption);
