import React from 'react';
import { connect } from 'react-redux';
import PropTypes from 'prop-types';
import Heading from '../components/Heading';

const TelemetryView = ({ telemetry }) => {
  const telemetryLines = telemetry.lines.map(line => (
    <span key={line.items[0].caption}>
      {
        line.items.map(item => (
          `${item.caption}${telemetry.captionValueSeparator}${item.value}`
        )).join(telemetry.itemSeparator)
      }
      <br />
    </span>
  ));
  const telemetryLog = telemetry.log.lines.map((line, i) => (
    <span key={i}>{line}<br /></span>
  ));
  return (
    <div>
      <Heading level={2} text="Telemetry" />
      <p>{telemetryLines}</p>
      <p>{telemetryLog}</p>
    </div>
  );
};

const itemPropType = PropTypes.shape({
  caption: PropTypes.string,
  value: PropTypes.string
});

const linePropType = PropTypes.shape({
  items: PropTypes.arrayOf(itemPropType)
});

TelemetryView.propTypes = {
  telemetry: PropTypes.shape({
    lines: PropTypes.arrayOf(linePropType).isRequired,
    log: PropTypes.shape({
      lines: PropTypes.arrayOf(PropTypes.string).isRequired
    }).isRequired,
    itemSeparator: PropTypes.string.isRequired,
    captionValueSeparator: PropTypes.string.isRequired,
    timestamp: PropTypes.number.isRequired
  }).isRequired
};

const mapStateToProps = ({ telemetry }) => ({
  telemetry
});

export default connect(mapStateToProps)(TelemetryView);
