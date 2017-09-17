import React, { Component } from 'react';
import PropTypes from 'prop-types';
import { connect } from 'react-redux';
import Heading from '../components/Heading';
import MultipleCheckbox from '../components/MultipleCheckbox';
import GraphCanvas from './GraphCanvas';
import IconGroup from '../components/IconGroup';
import Icon from '../components/Icon';
import { getAllItems } from '../reducers/telemetry';

class GraphView extends Component {
  constructor(props) {
    super(props);

    this.state = {
      graphing: false,
      keys: [],
    };

    this.handleClick = this.handleClick.bind(this);
    this.handleDocumentKeydown = this.handleDocumentKeydown.bind(this);
  }

  componentDidMount() {
    document.addEventListener('keydown', this.handleDocumentKeydown);
  }

  componentWillUnmount() {
    document.removeEventListener('keydown', this.handleDocumentKeydown);
  }

  handleDocumentKeydown(evt) {
    console.log(evt.code);
    if (!this.state.graphing && (evt.code === 'Enter' || evt.code === 'NumpadEnter')) {
      this.setState({
        graphing: true,
      });
    } else if (this.state.graphing && evt.code === 'Escape') {
      this.setState({
        graphing: false,
      });
    }
  }

  handleClick() {
    this.setState({
      graphing: !this.state.graphing,
    });
  }

  render() {
    return (
      <div>
        <Heading level={2} text="Graph">
          <IconGroup>
            <Icon
              icon={this.state.graphing ? 'close' : 'chart'}
              size="small"
              onClick={this.handleClick} />
          </IconGroup>
        </Heading>
        {
          this.state.graphing ?
            <GraphCanvas
              timestamp={this.props.timestamp}
              items={this.props.items.filter(({ caption }) => (
                this.state.keys.indexOf(caption) !== -1
              ))} />
            :
            (
              <MultipleCheckbox
                arr={this.props.items
                  .filter(item => !isNaN(parseFloat(item.value)))
                  .map(item => item.caption)}
                onChange={selected => this.setState({ keys: selected })}
                selected={this.state.keys} />
            )
        }
      </div>
    );
  }
}

const itemPropType = PropTypes.shape({
  caption: PropTypes.string,
  value: PropTypes.string
});

GraphView.propTypes = {
  timestamp: PropTypes.number.isRequired,
  items: PropTypes.arrayOf(itemPropType)
};

const mapStateToProps = ({ telemetry }) => ({
  timestamp: telemetry.timestamp,
  items: getAllItems(telemetry)
});

export default connect(mapStateToProps)(GraphView);
