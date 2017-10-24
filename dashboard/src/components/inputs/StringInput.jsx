import React from 'react';
import PropTypes from 'prop-types';

class StringInput extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      value: this.props.value
    };

    this.handleChange = this.handleChange.bind(this);
    this.handleKeyPress = this.handleKeyPress.bind(this);
  }

  componentWillReceiveProps() {
    this.setState({
      value: this.props.value
    });
  }

  handleChange(evt) {
    this.setState({
      value: evt.target.value
    });

    this.props.onChange(evt.target.value);
  }

  handleKeyPress(evt) {
    if (evt.key === 'Enter') {
      this.props.onUpdate(this.state.value);
      evt.stopPropagation();
    }
  }

  render() {
    return (
      <input
        className="valid"
        type="text"
        value={this.state.value}
        onChange={this.handleChange}
        onKeyPress={this.handleKeyPress}
      />
    );
  }
}

StringInput.propTypes = {
  value: PropTypes.string.isRequired,
  onChange: PropTypes.func.isRequired,
  onUpdate: PropTypes.func.isRequired
};

export default StringInput;
