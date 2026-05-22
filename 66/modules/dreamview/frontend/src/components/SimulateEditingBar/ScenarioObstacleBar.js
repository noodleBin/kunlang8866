import React, { Fragment } from 'react';
import classNames from 'classnames';
import { inject, observer } from 'mobx-react';
import WS from 'store/websocket';

@inject('store')
@observer
export default class ScenarioObstacle extends React.Component {
  // render obstacles List
  renderParticipantList() {
    const { scenarioEditingManager } = this.props.store;
    const { obstaclesList } = scenarioEditingManager;

    return (
      <div className="header-content">
        <div className="content-title">OBSTACLE LIST</div>
        {obstaclesList.map((item) => {
          return (
            <div
              className="content-item"
              key={item.id}
              onClick={() => scenarioEditingManager.changeCurObstacle(item.id)}
            >
              <span className="item-left">{item.id}</span>
              <span className="item-right">{item.type}</span>
            </div>
          );
        })}
      </div>
    );
  }

  // render basic information
  renderBasicInfo() {
    const { scenarioEditingManager } = this.props.store;
    const { curObstacle } = scenarioEditingManager;

    return (
      <div className="basic-content">
        <div className="content-header">Basic information</div>
        <div className="content-info">
          <div className="info-items">
            <span className="item-title">ID: </span>
            <input
              disabled
              className="item-info"
              type="text"
              value={curObstacle.id}
            />
          </div>
          <div className="info-items">
            <span className="item-title">Obstacle type: </span>
            <select
              className="item-info"
              value={curObstacle.type}
              onChange={(event) => scenarioEditingManager.changeType(event)}
            >
              <option disabled value="">
                ---please select---
              </option>
              <option value="STACKER">STACKER</option>
              <option value="FORKLIFT_STACKER">FORKLIFT_STACKER</option>
              <option value="WHEELCRANE">WHEELCRANE</option>
              <option value="UNKNOWN">UNKNOWN</option>
              <option value="VEHICLE">VEHICLE</option>
              <option value="PEDESTRIAN">PEDESTRIAN</option>
              <option value="BICYCLE">BICYCLE</option>
            </select>
          </div>
          <div className="info-items">
            <span className="item-title">Width: </span>
            <input
              className="item-info"
              type="number"
              min="0"
              step=".1"
              value={curObstacle.width}
              onChange={(event) => scenarioEditingManager.changeWidth(event)}
            />
          </div>
          <div className="info-items">
            <span className="item-title">Length: </span>
            <input
              className="item-info"
              type="number"
              min="0"
              step=".1"
              value={curObstacle.length}
              onChange={(event) => scenarioEditingManager.changeLength(event)}
            />
          </div>
          <div className="info-items">
            <span className="item-title">Height: </span>
            <input
              className="item-info"
              type="number"
              min="0"
              step=".1"
              value={curObstacle.height}
              onChange={(event) => scenarioEditingManager.changeHeight(event)}
            />
          </div>
        </div>
      </div>
    );
  }

  // render obstacle status
  renderInitialStatus() {
    const { scenarioEditingManager } = this.props.store;
    const { curObstacle } = scenarioEditingManager;
    return (
      <div className="basic-content">
        <div className="content-header">Initial state</div>
        <div className="content-info">
          <div className="info-items">
            <span className="item-title">X: </span>
            <span>{curObstacle.position[0].toFixed(2)}</span>
          </div>
          <div className="info-items">
            <span className="item-title">Y: </span>
            <span>{curObstacle.position[1].toFixed(2)}</span>
          </div>
          <div className="info-items">
            <span className="item-title">Heading: </span>
            <span>{curObstacle.heading.toFixed(2)}</span>rad
          </div>
        </div>
      </div>
    );
  }

  // render running status
  renderRunningStatus() {
    const { scenarioEditingManager } = this.props.store;
    const { curObstacle } = scenarioEditingManager;

    return (
      <div className="basic-content">
        <div className="content-header">Runtime configuration</div>
        <div className="content-info">
          <div className="info-items">
            <span className="item-title">Move way: </span>
            <select
              className="item-info"
              value={curObstacle.move_way}
              onChange={(event) => scenarioEditingManager.changeMoveWay(event)}
            >
              <option disabled value="">
                ---please select---
              </option>
              <option value="static">Static</option>
              <option value="move">Move according to trajectory</option>
            </select>
          </div>
          {curObstacle.move_way === 'move' ? (
            <div className="info-items">
              <span className="item-title">Speed: </span>
              <input
                className="item-info"
                type="number"
                min="0"
                step=".5"
                value={curObstacle.speed}
                onChange={(event) => scenarioEditingManager.changeSpeed(event)}
              />
            </div>
          ) : null}
        </div>
      </div>
    );
  }

  // // render path status
  renderPathStatus() {
    const { scenarioEditingManager, options } = this.props.store;
    const { curObstacle } = scenarioEditingManager;
    const ifShow = curObstacle.move_way === 'move' ? true : false;

    return (
      <div className="basic-content">
        {ifShow ? (
          <div>
            <div className="content-header header-flex">
              <div className="header-left">Trace point</div>
              <button
                className={classNames({
                  'header-right': true,
                  'mute-button': true,
                  'header-right-active': options.addObstaclePathPoint,
                })}
                onClick={() => {
                  this.props.store.handleOptionToggle('addObstaclePathPoint');
                }}
              >
                Add
              </button>
            </div>
            {curObstacle.trace && curObstacle.trace.length > 0 ? (
              curObstacle.trace.map((item, index) => {
                return (
                  <div className="content-info">
                    <button
                      className="delInfo-button mute-button"
                      onClick={() => {
                        scenarioEditingManager.delObstacleTracePoint(item.id);
                      }}
                    >
                      Del
                    </button>
                    <div className="info-items">
                      <span className="item-title">X: </span>
                      <span>{item.position[0].toFixed(2)}</span>
                    </div>
                    <div className="info-items">
                      <span className="item-title">Y: </span>
                      <span>{item.position[1].toFixed(2)}</span>
                    </div>
                    {index !== curObstacle.trace.length - 1 ? (
                      <div className="info-items">
                        <span className="item-title">Speed: </span>
                        <input
                          className="item-info"
                          type="number"
                          min="0"
                          value={item.speed}
                          onChange={(event) => {
                            scenarioEditingManager.changeObstacleTraceSpeed(
                              event,
                              item.id,
                            );
                          }}
                        />
                      </div>
                    ) : null}
                  </div>
                );
              })
            ) : (
              <div style={{ overflow: 'hidden' }}>
                <div className="empty-list">please add.</div>
              </div>
            )}
          </div>
        ) : (
          <Fragment></Fragment>
        )}
      </div>
    );
  }

  // // render trigger status
  renderTriggerStatus() {
    const { scenarioEditingManager, options } = this.props.store;
    const { curObstacle } = scenarioEditingManager;
    const ifShow = curObstacle.move_way === 'move' ? true : false;
    const ifShowRadius =
      curObstacle.trigger_position.length !== 0 ? true : false;

    return (
      <div className="basic-content">
        {ifShow ? (
          <div>
            <div className="content-header header-flex">
              <div className="header-left">Trigger point</div>
              <div style={{ display: 'flex' }}>
                <button
                  className={classNames({
                    'header-right': true,
                    'mute-button': true,
                    'header-right-active': options.addObstacleTriggerPoint,
                  })}
                  onClick={() => {
                    this.props.store.handleOptionToggle(
                      'addObstacleTriggerPoint',
                    );
                  }}
                  disabled={ifShowRadius}
                >
                  Add
                </button>
                {ifShowRadius ? (
                  <button
                    className="header-right mute-button"
                    style={{ 'margin-left': '5px' }}
                    onClick={() => {
                      scenarioEditingManager.delObstacleTriggerPoint(
                        curObstacle.trigger_position[0].id,
                      );
                    }}
                  >
                    Del
                  </button>
                ) : (
                  <Fragment></Fragment>
                )}
              </div>
            </div>
            {ifShowRadius ? (
              <div className="content-info">
                <div className="info-items">
                  <span className="item-title">Radius:</span>
                  <input
                    className="item-info"
                    min="0"
                    step="1"
                    type="number"
                    value={curObstacle.trigger_radius}
                    onChange={(event) =>
                      scenarioEditingManager.changeTriggerRadius(event)
                    }
                  ></input>
                  m
                </div>
                <div className="info-items">
                  <span className="item-title">X: </span>
                  <span>
                    {curObstacle.trigger_position[0].position[0].toFixed(2)}
                  </span>
                </div>
                <div className="info-items">
                  <span className="item-title">Y: </span>
                  <span>
                    {curObstacle.trigger_position[0].position[1].toFixed(2)}
                  </span>
                </div>
              </div>
            ) : (
              <div style={{ overflow: 'hidden' }}>
                <div className="empty-list">please add.</div>
              </div>
            )}
          </div>
        ) : (
          <Fragment></Fragment>
        )}
      </div>
    );
  }

  render() {
    const { options, scenarioEditingManager } = this.props.store;
    const { obstaclesList } = scenarioEditingManager;

    return (
      <div className="main-obstacle">
        <div className="obstacle-title">
          <div className="obstacle-left">MAIN OBSTACLE</div>
          <div style={{ display: 'flex' }}>
            {obstaclesList.length !== 0 ? (
              <button
                className={classNames({
                  'obstacle-right': true,
                  'mute-button': true,
                })}
                onClick={() => WS.addSimulationObstacles(obstaclesList)}
              >
                Save
              </button>
            ) : null}
            <button
              className={classNames({
                'obstacle-right': true,
                'mute-button': true,
                'obstacle-right-active': options.enableObstacleEditing,
              })}
              onClick={() => {
                this.props.store.handleOptionToggle('enableObstacleEditing');
              }}
            >
              Add
            </button>
          </div>
        </div>
        {obstaclesList.length !== 0 ? (
          <Fragment>
            {this.renderParticipantList()}
            {this.renderBasicInfo()}
            {this.renderInitialStatus()}
            {this.renderRunningStatus()}
            {this.renderPathStatus()}
            {this.renderTriggerStatus()}
            <button
              className="obstacle-del mute-button"
              style={{ margin: '0 auto', display: 'block' }}
              onClick={() => {
                if (
                  options.addObstacleTriggerPoint ||
                  options.addObstaclePathPoint
                ) {
                  return alert(
                    'Currently in editing status, please exit first.',
                  );
                }
                scenarioEditingManager.delCurObstacle();
              }}
            >
              REMOVE CURRENT OBSTACLE
            </button>
          </Fragment>
        ) : (
          <div className="empty-list">please add at least one obstacle</div>
        )}
      </div>
    );
  }
}
