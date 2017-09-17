import React from 'react';
import PropTypes from 'prop-types';
import { connect } from 'react-redux';
import ConfigOptionGroup from './ConfigOptionGroup';
import Heading from '../components/Heading';
import IconGroup from '../components/IconGroup';
import Icon from '../components/Icon';
import { getConfig, updateConfig } from '../actions/config';

const ConfigView = ({ config, onRefresh, onSave }) => (
  <div>
    <Heading level={2} text="Configuration">
      <IconGroup>
        <Icon icon="refresh" size="small" onClick={onRefresh} />
        {
          (config.every(v => !v.invalid || v.invalid.length === 0)) ?
            <Icon icon="save" size="small" onClick={onSave} /> : undefined
        }
      </IconGroup>
    </Heading>
    {config.map((optionGroup, optionGroupIndex) => (
      <ConfigOptionGroup
        key={optionGroupIndex}
        name={optionGroup.name}
        options={optionGroup.options} />
    ))}
  </div>
);

ConfigView.propTypes = {
  config: PropTypes.array.isRequired,
  onRefresh: PropTypes.func.isRequired,
  onSave: PropTypes.func.isRequired
};

const mapStateToProps = ({ config }) => ({
  config
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
