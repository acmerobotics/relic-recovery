import React from 'react';
import PropTypes from 'prop-types';
import { connect } from 'react-redux';
import Option from './Option';
import Heading from '../components/Heading';
import IconGroup from '../components/IconGroup';
import Icon from '../components/Icon';
import { getConfig, updateConfig } from '../actions/config';

const ConfigView = ({ config, configSchema, onRefresh }) => (
  <div>
    <Heading level={2} text="Configuration">
      <IconGroup>
        <Icon icon="refresh" size="small" onClick={onRefresh} />
      </IconGroup>
    </Heading>
    <table>
      <tbody>
        {Object.keys(configSchema).map((key) => (
          <Option
            key={key}
            name={key}
            value={config[key]}
            schema={configSchema[key]} />
        ))}
      </tbody>
    </table>
  </div>
);

ConfigView.propTypes = {
  config: PropTypes.object.isRequired,
  configSchema: PropTypes.object.isRequired,
  onRefresh: PropTypes.func.isRequired
};

const mapStateToProps = ({ config, configSchema }) => ({
  config,
  configSchema
});

const mapDispatchToProps = (dispatch) => ({
  onRefresh: () => {
    dispatch(getConfig());
  },
  onSave: () => {
    dispatch(updateConfig());
  }
});

export default connect(mapStateToProps, mapDispatchToProps)(ConfigView);
