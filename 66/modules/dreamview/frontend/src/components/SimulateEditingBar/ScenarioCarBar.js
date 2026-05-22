import React, { Fragment } from 'react';
import { inject, observer } from 'mobx-react';
import classNames from 'classnames';
import CheckboxItem from 'components/common/CheckboxItem';

@inject('store')
@observer
export default class ScenarioCar extends React.Component {
  renderEmptyPoint() {
    return (
      <div className="vehicle-content">
        <div className="empty-list">Please add first!</div>
      </div>
    );
  }

  renderVehicleStart() {
    const { scenarioEditingManager } = this.props.store;
    const { routingPoints } = scenarioEditingManager;

    return routingPoints.map((item, index) => {
      return (
        <div className="vehicle-content" key={item.id}>
          <div className="content-header">
            <div className="header-left">NO.{index + 1} point</div>
            <button
              className="header-right mute-button"
              onClick={() =>
                scenarioEditingManager.removeOneRoutingPoint(item.id)
              }
            >
              Del
            </button>
          </div>
          <div className="content-center">
            <div className="center-info">
              <span className="info-title">X: </span>
              <span className="info-center">{item.x.toFixed(8)}</span>
            </div>
            <div className="center-info">
              <span className="info-title">Y: </span>
              <span className="info-center">{item.y.toFixed(8)}</span>
            </div>
            <div className="center-info">
              <span className="info-title">Heading: </span>
              <span className="info-center">
                {item.heading ? item.heading.toFixed(2) : 0}
                rad
              </span>
            </div>
          </div>
        </div>
      );
    });
  }

  render() {
    const { options, scenarioEditingManager } = this.props.store;
    const { routingPoints, isLoopRunning } = scenarioEditingManager;

    return (
      <div className="main-vehicle">
        <div className="vehicle-title">
          <div className="vehicle-left">VEHICLE ROUTING</div>
          <div style={{ display: 'flex' }}>
            <button
              className={classNames({
                'vehicle-right': true,
                'mute-button': true,
                'vehicle-right-active': options.enableRoutingEditing,
              })}
              onClick={() => {
                this.props.store.handleOptionToggle('enableRoutingEditing');
              }}
            >
              Add
            </button>
          </div>
        </div>
        {routingPoints.length !== 0 ? (
          <Fragment>
            <CheckboxItem
              id="isLoopRunning"
              title="Is Loop Running"
              isChecked={isLoopRunning}
              extraClasses="others-checkbox"
              onClick={() => {
                scenarioEditingManager.toggleLoopRunning();
              }}
            />
            {this.renderVehicleStart()}
          </Fragment>
        ) : (
          this.renderEmptyPoint()
        )}
      </div>
    );
  }
}
