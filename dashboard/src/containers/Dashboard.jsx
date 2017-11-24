import React, { Component } from 'react';
import { connect as reduxConnect } from 'react-redux';
import PropTypes from 'prop-types';
import Heading from '../components/Heading';
import Header from '../components/Header';
import IconGroup from '../components/IconGroup';
import Icon from '../components/Icon';
import TelemetryView from './TelemetryView';
import ConfigView from './ConfigView';
import GraphView from './GraphView';
import FieldView from './FieldView';
import Tile from '../components/Tile';
import TileGrid from '../components/TileGrid';
import { connect, disconnect } from '../actions/socket';

class Dashboard extends Component {
  constructor(props) {
    super(props);

    this.layouts = [
      'Field + 2 Graph',
      'Field + Graph',
      'Graph'
    ];

    this.state = {
      layoutIndex: 0,
      graphing: false
    };

    this.onLayoutChange = this.onLayoutChange.bind(this);
    this.handleGraphingChange = this.handleGraphingChange.bind(this);
  }

  componentDidMount() {
    this.props.dispatch(connect('192.168.49.1', 8000));
  }

  componentWillUnmount() {
    this.props.dispatch(disconnect());
  }

  handleGraphingChange(graphing) {
    this.setState({
      graphing
    });
  }

  onLayoutChange(evt) {
    this.setState({
      layoutIndex: evt.target.selectedIndex
    });
  }

  renderGrid() {
    if (this.state.layoutIndex === 0) {
      return (
        <TileGrid>
          <Tile row="1 / span 3" col={1} hidden>
            <FieldView />
          </Tile>
          <Tile row={1} col={2}>
            <GraphView />
          </Tile>
          <Tile row="2 / span 2" col={2}>
            <GraphView />
          </Tile>
          <Tile row="1 / span 2" col={3}>
            <ConfigView />
          </Tile>
          <Tile row={3} col={3}>
            <TelemetryView />
          </Tile>
        </TileGrid>
      );
    }

    if (this.state.layoutIndex === 1) {
      return (
        <TileGrid>
          <Tile row="1 / span 3" col={1} hidden>
            <FieldView />
          </Tile>
          <Tile row="1 / span 3" col={2}>
            <GraphView />
          </Tile>
          <Tile row="1 / span 2" col={3}>
            <ConfigView />
          </Tile>
          <Tile row={3} col={3}>
            <TelemetryView />
          </Tile>
        </TileGrid>
      );
    }

    return (
      <TileGrid>
        <Tile row="1 / span 3" col="1 / span 2">
          <GraphView />
        </Tile>
        <Tile row="1 / span 2" col={3}>
          <ConfigView />
        </Tile>
        <Tile row={3} col={3}>
          <TelemetryView />
        </Tile>
      </TileGrid>
    );
  }

  render() {
    return (
      <div>
        <Header>
          <Heading text="FTC Dashboard" level={1}>
            <IconGroup>
              <select value={this.layouts[this.layoutIndex]} onChange={this.onLayoutChange}>
                {
                  this.layouts.map((layout) => (
                    <option key={layout} name={layout}>{layout}</option>
                  ))
                }
              </select>
              <p>&nbsp;&nbsp;&nbsp;&nbsp;</p>
              {
                this.props.isConnected ?
                  <p>{this.props.pingTime}ms&nbsp;&nbsp;&nbsp;&nbsp;</p>
                  : null
              }
              <Icon icon={this.props.isConnected ? 'wifi' : 'no-wifi'} size="large" />
            </IconGroup>
          </Heading>
        </Header>
        { this.renderGrid() }
      </div>
    );
  }
}

Dashboard.propTypes = {
  isConnected: PropTypes.bool.isRequired,
  pingTime: PropTypes.number.isRequired,
  dispatch: PropTypes.func.isRequired
};

const mapStateToProps = ({ socket }) => ({
  isConnected: socket.isConnected,
  pingTime: socket.pingTime
});

export default reduxConnect(mapStateToProps)(Dashboard);
