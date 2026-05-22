import React from 'react';
import { inject, observer } from 'mobx-react';

import EditingTip from 'components/SimulateEditingBar/EditingTip';
import ConfirmationDialog from 'components/common/ConfirmModal';
import { TEMPORARY_MOVE_TASK_TYPE } from 'store/scenario_editing_manager';

import removeAllRoutingIcon from 'assets/images/routing/remove_all_routing.png';
import removeAllObstaclesIcon from 'assets/images/routing/remove_all_obstacles.png';
import sendRequestIcon from 'assets/images/routing/send_request.png';
import actionIcon from 'assets/images/routing/action.png';
import editIcon from 'assets/images/routing/edit_simulate.png';
import inDefaultScenarioIcon from 'assets/images/routing/in_default_scenario.png';
import exitDefaultScenarioIcon from 'assets/images/routing/exit_default_scenario.png';
import classNames from 'classnames';

class SimulateEditingButton extends React.Component {
  render() {
    const { label, icon, onClick, disabled } = this.props;

    return (
            <button onClick={onClick}
              className={
                classNames({
                  'button': true,
                  disabled
                })
              }
              disabled={disabled}
            >
              <img src={icon} />
              <span>{label}</span>
            </button>
    );
  }
}

@inject('store') @observer
export default class RouteEditingMenu extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      showTemporaryMoveConfirmDialog: false,
    };
    this.editingPanelRef = React.createRef();
  }

  componentDidMount() {
    const panel = this.editingPanelRef.current;
    if (panel) {
      panel.addEventListener('wheel', this.handleEditingPanelWheel, { passive: false });
    }
  }

  componentWillUnmount() {
    const panel = this.editingPanelRef.current;
    if (panel) {
      panel.removeEventListener('wheel', this.handleEditingPanelWheel);
    }
    this.resetTemporaryMoveSession();
  }

  isTemporaryMoveReadyToConfirm = () => {
    const { scenarioEditingManager } = this.props.store;
    return (
      scenarioEditingManager.isTemporaryMoveSelecting &&
      scenarioEditingManager.temporaryMovePointSelected
    );
  };

  resetTemporaryMoveSession = () => {
    const { scenarioEditingManager, options } = this.props.store;
    scenarioEditingManager.resetTemporaryMoveSelectingSession();
    if (options.enableRoutingEditing) {
      this.props.store.handleOptionToggle('enableRoutingEditing');
    }
    scenarioEditingManager.syncSimulationElementsVisibility(options.showSimulateEditingBar);
  };

  finishTemporaryMoveSelecting = () => {
    const { scenarioEditingManager, options } = this.props.store;
    this.resetTemporaryMoveSession();
    if (!options.showEditScenario) {
      scenarioEditingManager.disableScenarioEditing();
    }
  };

  handleStartTemporaryMove = () => {
    const { scenarioEditingManager, options } = this.props.store;
    if (!this.isTemporaryMoveReadyToConfirm()) {
      if (options.showEditScenario) {
        this.props.store.handleOptionToggle('showEditScenario');
      }
      scenarioEditingManager.enableScenarioEditing();
      this.props.store.options.selectCamera('Map');
      scenarioEditingManager.clearTemporaryMovePoint();
      scenarioEditingManager.startTemporaryMoveSelectingMode();
      if (!options.enableRoutingEditing) {
        this.props.store.handleOptionToggle('enableRoutingEditing');
      }
      scenarioEditingManager.syncSimulationElementsVisibility(options.showSimulateEditingBar);
      return;
    }
    this.setState({ showTemporaryMoveConfirmDialog: true });
  };

  handleModifyTemporaryMovePoint = () => {
    const { scenarioEditingManager, options } = this.props.store;
    if (!scenarioEditingManager.isTemporaryMoveSelecting) {
      scenarioEditingManager.startTemporaryMoveSelectingMode();
    }
    this.props.store.options.selectCamera('Map');
    if (!options.enableRoutingEditing) {
      this.props.store.handleOptionToggle('enableRoutingEditing');
    }
    scenarioEditingManager.syncSimulationElementsVisibility(options.showSimulateEditingBar);
  };

  handleConfirmTemporaryMove = () => {
    const { scenarioEditingManager } = this.props.store;
    const success = scenarioEditingManager.sendTemporaryMoveRequest();
    if (success) {
      this.finishTemporaryMoveSelecting();
      this.setState({ showTemporaryMoveConfirmDialog: false });
    }
  };

  handleCancelTemporaryMove = () => {
    this.setState({ showTemporaryMoveConfirmDialog: false });
  };

  handleEndTemporaryMove = () => {
    const { scenarioEditingManager } = this.props.store;
    if (!scenarioEditingManager.isTemporaryMoveActive) {
      return;
    }
    const success = scenarioEditingManager.endTemporaryMove();
    if (success) {
      this.finishTemporaryMoveSelecting();
    }
  };

  handleEditingPanelWheel = (event) => {
    const panel = this.editingPanelRef.current;
    if (!panel) {
      return;
    }
    const hasHorizontalOverflow = panel.scrollWidth > panel.clientWidth;
    if (!hasHorizontalOverflow) {
      return;
    }
    const delta = event.deltaX !== 0 ? event.deltaX : event.deltaY;
    if (delta === 0) {
      return;
    }
    panel.scrollLeft += delta;
    event.preventDefault();
  };

  handleTemporaryMoveTaskTypeChange = (event) => {
    const { scenarioEditingManager } = this.props.store;
    scenarioEditingManager.setTemporaryMoveTaskType(event.target.value);
  };

  handleEditScenarioClick = () => {
    const { scenarioEditingManager, options } = this.props.store;
    if (scenarioEditingManager.isTemporaryMoveMode()
      && !scenarioEditingManager.exitTemporaryMoveMode()) {
      return;
    }
    this.setState({ showTemporaryMoveConfirmDialog: false });
    this.props.store.handleOptionToggle('showEditScenario');
    scenarioEditingManager.syncSimulationElementsVisibility(options.showSimulateEditingBar);
  };

  render() {
    const { scenarioEditingManager, options } = this.props.store;
    const isSearchReachStackerTemporaryMove =
      scenarioEditingManager.isSearchReachStackerTemporaryMoveTask();
    const canModifyTemporaryMovePoint = this.isTemporaryMoveReadyToConfirm();

    return (
            <div className="simulate-editing-bar">
                <div
                    className="editing-panel"
                    ref={this.editingPanelRef}
                >
                    <ConfirmationDialog
                        isOpen={this.state.showTemporaryMoveConfirmDialog}
                        onConfirm={this.handleConfirmTemporaryMove}
                        onCancel={this.handleCancelTemporaryMove}
                        message="Confirm sending temporary move request?"
                    />
                    <SimulateEditingButton
                        label="Edit Scenario"
                        icon={editIcon}
                        onClick={this.handleEditScenarioClick}
                    />
                    <SimulateEditingButton
                        label="Save Scenario"
                        icon={exitDefaultScenarioIcon}
                        disabled={!options.showEditScenario}
                        onClick={() => {
                          scenarioEditingManager.downloadScenario();
                        }}
                    />
                    <SimulateEditingButton
                        label="Import Scenario"
                        icon={inDefaultScenarioIcon}
                        disabled={!options.showEditScenario}
                        onClick={() => {
                          scenarioEditingManager.readScenario();
                        }}
                    />
                    <SimulateEditingButton
                        label="Send Request"
                        icon={sendRequestIcon}
                        onClick={() => {
                          if (scenarioEditingManager.sendRoutingRequest(false)) {
                            if (options.showEditScenario) {
                              this.props.store.handleOptionToggle('showEditScenario');
                            }
                          }
                        }}
                    />
                    <SimulateEditingButton
                        label="Remove All Routing"
                        icon={removeAllRoutingIcon}
                        disabled={!options.showEditScenario}
                        onClick={() => {
                          scenarioEditingManager.removeAllRoutingPoints();
                        }}
                    />
                    <SimulateEditingButton
                        label="Remove All Obstacles"
                        icon={removeAllObstaclesIcon}
                        disabled={!options.showEditScenario}
                        onClick={() => {
                          scenarioEditingManager.removeAllObstacles();
                        }}
                    />
                    <div className="temporary-move-task-type">
                        <span>Temporary Move Task</span>
                        <select
                            value={scenarioEditingManager.temporaryMoveTaskType}
                            onChange={this.handleTemporaryMoveTaskTypeChange}
                        >
                            <option value={TEMPORARY_MOVE_TASK_TYPE.SEARCH_REACH_STACKER}>
                              SEARCH_REACH_STACKER
                            </option>
                            <option value={TEMPORARY_MOVE_TASK_TYPE.TEMPORARY_VEH_RELOCATION}>
                              TEMPORARY_VEH_RELOCATION
                            </option>
                        </select>
                    </div>
                    <SimulateEditingButton
                        label={this.isTemporaryMoveReadyToConfirm()
                          ? 'Confirm Temporary Move'
                          : 'Start Temporary Move'}
                        icon={actionIcon}
                        onClick={this.handleStartTemporaryMove}
                    />
                    <SimulateEditingButton
                        label="Modify Move Point"
                        icon={editIcon}
                        disabled={!canModifyTemporaryMovePoint}
                        onClick={this.handleModifyTemporaryMovePoint}
                    />
                    <SimulateEditingButton
                        label="End Temporary Move"
                        icon={actionIcon}
                        disabled={
                          !scenarioEditingManager.isTemporaryMoveActive ||
                          isSearchReachStackerTemporaryMove
                        }
                        onClick={this.handleEndTemporaryMove}
                    />
                    <input type='file' style={{display: 'none'}} id='files' accept='.json' />
                    <EditingTip />
                </div>
            </div>
    );
  }
}
