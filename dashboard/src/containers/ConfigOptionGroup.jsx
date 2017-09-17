import React from 'react';
import PropTypes from 'prop-types';
import ConfigOption from './ConfigOption';
import Heading from '../components/Heading';
import Icon from '../components/Icon';

class ConfigOptionGroup extends React.Component {
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
    return (
      <div>
        <Heading level={3} text={this.props.name} >
          <Icon icon={ this.state.expanded ? 'remove' : 'add' } size='small' onClick={this.toggleVisibility} />
        </Heading>
        {
          this.state.expanded ?
            (
              <table>
                <tbody>
                  {this.props.options.map((option, optionIndex) => (
                    <ConfigOption
                      key={optionIndex}
                      optionGroup={this.props.name}
                      option={option} />
                  ))}
                </tbody>
              </table>
            )
            : null
        }
      </div>
    );
  }
}

ConfigOptionGroup.propTypes = {
  name: PropTypes.string.isRequired,
  options: PropTypes.array.isRequired
};

export default ConfigOptionGroup;
