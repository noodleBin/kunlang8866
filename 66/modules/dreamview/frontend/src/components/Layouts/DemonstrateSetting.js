import React, { Fragment } from 'react';
import { inject, observer } from 'mobx-react';
import Selector from 'components/Header/Selector';
import ConfirmationDialog, {
  ConfirmPsdDialog,
  ConfirmDoublePsdDialog,
} from 'components/common/ConfirmModal';
import yardsQD from 'components/Layouts/yardsQD';
import trainsQD from 'components/Layouts/trainsQD';
import yardsDJZ from 'components/Layouts/yardsDJZ';
import trainsDJZ from 'components/Layouts/trainsDJZ';
import CoordinateSelector from './CoordinateSelector';
import CheckboxItem from 'components/common/CheckboxItem';

import WS from 'store/websocket';

@inject('store')
@observer
export default class DemonstrateSetting extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      currentYard: null,
      currentTrain: null,
      types: [
        'DEFAULT',
        'PARKINGSPACE',
        'RAILWAY_WAITINGAREA_STATIC',
        'RAILWAY_WAITINGAREA_DYNAMIC',
        'RAILWAY_OPERATIONAREA_DYNAMIC',
        'LOADING_OPERATIONAREA_SAMEDIRECTION_1',
        'LOADING_OPERATIONAREA_SAMEDIRECTION_2',
        'LOADING_OPERATIONAREA_SAMEDIRECTION_3_0',
        'LOADING_OPERATIONAREA_SAMEDIRECTION_3_1',
        'LOADING_OPERATIONAREA_DIFFDIRECTION_1',
        'YARD_WAITINGAREA_STATIC',
        'YARD_OPERATIONAREA_STATIC',
        'YARD_OPERATIONAREA_DYNAMIC',
        'UNLOAD_OPERATIONAREA_SAMEDIRECTION_1',
        'UNLOAD_OPERATIONAREA_SAMEDIRECTION_2',
        'UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_0',
        'UNLOAD_OPERATIONAREA_SAMEDIRECTION_3_1',
        'UNLOAD_OPERATIONAREA_DIFFDIRECTION_1',
        'BACKWARD_ROUTING_NEED_PLANNING_REROUTING',
        'BACKWARD_ROUTING_DIRECTLY',
        'LONG_ADJUSTMENT_FRONT',
        'LONG_ADJUSTMENT_BACK',
      ],
      currentType: 'DEFAULT',
      blacklistedLane: null,
      locationName: '',
      isDialogOpenArrived: false,
      isLoopRunning: false,
      showEditProfile: true,
      vehicleConfig: this.props.vehicleConfig,
      allVehicleConfig: this.props.allVehicleConfig,
      showPsdModal: false,
      showDoublePsdModal: false,
      showBarrierCommandPsdModal: false,
      fineTuningNumber: 0,
      ports: ['DJZ', 'QDG'],
      currentPort: 'QDG',
      currentYards: yardsQD,
      currentTrains: trainsQD,
      barrierCommandEnabled: false,
      barrierCommands: ['FORCE_OPEN', 'FORCE_CLOSE'],
      currentBarrierCommand: 'FORCE_OPEN',
    };
  }

  handleSaveCarPosition = () => {
    this.props.store.scenarioEditingManager.saveCurCarPosition(
      this.state.locationName,
    );
    this.setState({ locationName: '' });
  };

  handleLoopRunning = () => {
    this.setState({ isLoopRunning: !this.state.isLoopRunning });
  };

  handleConfirmLoopRunning = () => {
    this.setState({ isLoopRunning: false });
    this.props.store.scenarioEditingManager.startLoopRunning();
  };

  handleCancelLoopRunning = () => {
    this.setState({ isLoopRunning: false });
  };

  handleSubmitArrived = () => {
    this.setState({ isDialogOpenArrived: true });
  };

  handleConfirmArrived = () => {
    this.setState({ isDialogOpenArrived: false });
    WS.requestImmediatelyArrived();
  };

  handleCancelArrived = () => {
    this.setState({ isDialogOpenArrived: false });
  };

  handleStartEditProfile = () => {
    this.setState({ showEditProfile: false });
  };

  handleCancelEditProfile = () => {
    WS.getVehicleConfig();

    this.setState({
      showEditProfile: true,
      vehicleConfig: this.props.vehicleConfig,
      allVehicleConfig: this.props.allVehicleConfig,
    });
  };

  handleSaveEditProfile = () => {
    this.setState({ showEditProfile: true });
    const pattern = /speed/;
    const allVehicleConfig = Object.keys(this.state.allVehicleConfig).reduce(
      (obj, key) => {
        if (pattern.test(key)) {
          obj[key] = this.state.allVehicleConfig['planning_upper_speed_limit'];
          return obj;
        }
        obj[key] = this.state.allVehicleConfig[key];
        return obj;
      },
      {},
    );

    WS.modifyVehicleConfig(allVehicleConfig);
  };

  handleChange = (key, event) => {
    const value = event.target.value;
    this.setState((prevState) => ({
      ...prevState,
      vehicleConfig: {
        ...prevState.vehicleConfig,
        [key]: value,
      },
      allVehicleConfig: {
        ...prevState.allVehicleConfig,
        [key]: value,
      },
    }));
  };

  handleConfirmPsd = (psd) => {
    this.setState({ showPsdModal: false });
    WS.changeFASAEBStatus(false, true, psd);
  };

  handleCancelPsd = () => {
    this.setState({ showPsdModal: false });
  };

  handleConfirmDoublePsd = (firstPwd, secondPwd) => {
    this.setState({ showDoublePsdModal: false });
    WS.changeFASAEBStatus(false, false, firstPwd, secondPwd);
  };

  handleCancelDoublePsd = () => {
    this.setState({ showDoublePsdModal: false });
  };

  componentDidMount() {
    WS.getVehicleConfig();
  }

  componentWillUnmount() {
    if (this.props.store.hmi.barrierCommandEnabled) {
      WS.sendBarrierCommand(false, this.state.currentBarrierCommand);
    }
  }

  componentWillReceiveProps(nextProps) {
    if (nextProps.vehicleConfig) {
      this.setState({
        vehicleConfig: nextProps.vehicleConfig,
      });
    }
    if (nextProps.allVehicleConfig) {
      this.setState({
        allVehicleConfig: nextProps.allVehicleConfig,
      });
    }
  }

  handleSelectYard = (coord) => {
    this.setState({ currentYard: coord });
  };

  handleSelectTrain = (coord) => {
    this.setState({ currentTrain: coord });
  };

  handleToggleBarrierCommand = () => {
    const { hmi } = this.props.store;
    const nextEnabled = !hmi.barrierCommandEnabled;
    hmi.updateBarrierCommandEnabled(nextEnabled);
    if (!nextEnabled) {
      WS.sendBarrierCommand(false, this.state.currentBarrierCommand);
    }
  };

  handleSendBarrierCommand = () => {
    if (!this.props.store.hmi.barrierCommandEnabled) {
      alert('Please enable barrier command first.');
      return;
    }
    this.setState({ showBarrierCommandPsdModal: true });
  };

  handleConfirmBarrierCommandPsd = (password) => {
    this.setState({ showBarrierCommandPsdModal: false });
    WS.sendBarrierCommand(
      this.props.store.hmi.barrierCommandEnabled,
      this.state.currentBarrierCommand,
      password,
    );
  };

  handleCancelBarrierCommandPsd = () => {
    this.setState({ showBarrierCommandPsdModal: false });
  };

  render() {
    const barrierCommandEnabled = this.props.store.hmi.barrierCommandEnabled;
    return (
      <div className="header demonstrate-show">
        <ConfirmationDialog
          isOpen={this.state.isDialogOpenArrived}
          onConfirm={this.handleConfirmArrived}
          onCancel={this.handleCancelArrived}
          message="Are you sure?"
        />
        <ConfirmationDialog
          isOpen={this.state.isLoopRunning}
          onConfirm={this.handleConfirmLoopRunning}
          onCancel={this.handleCancelLoopRunning}
          message="Are you sure?"
        />
        <ConfirmPsdDialog
          isOpen={this.state.showPsdModal}
          onConfirm={this.handleConfirmPsd}
          onCancel={this.handleCancelPsd}
        />
        <ConfirmDoublePsdDialog
          isOpen={this.state.showDoublePsdModal}
          onConfirm={this.handleConfirmDoublePsd}
          onCancel={this.handleCancelDoublePsd}
        />
        <ConfirmPsdDialog
          isOpen={this.state.showBarrierCommandPsdModal}
          onConfirm={this.handleConfirmBarrierCommandPsd}
          onCancel={this.handleCancelBarrierCommandPsd}
        />
        <div style={{ width: '100%' }}>
          <h3>Simulate sending site</h3>
          <div style={{ marginBottom: '10px' }}>
            <span>Type: </span>
            <Selector
              name="Type"
              options={this.state.types}
              currentOption={this.state.currentType}
              onChange={(event) => {
                this.setState({
                  currentType: event.target.value,
                });
              }}
            />
          </div>

          <div style={{ marginBottom: '10px' }}>
            <span>Blacklisted_Road: </span>
            <div style={{ margin: '5px' }}>
              <input
                type="text"
                placeholder="Please enter IDs separated by commas"
                value={this.state.blacklistedLane}
                onChange={(event) =>
                  this.setState({ blacklistedLane: event.target.value })
                }
                className="input-text"
                style={{ backgroundColor: '#000000' }}
              />
            </div>
          </div>

          <div style={{ marginBottom: '10px' }}>
            <span>Port: </span>
            <Selector
              name="Port"
              options={this.state.ports}
              currentOption={this.state.currentPort}
              onChange={(event) => {
                this.setState(
                  {
                    currentPort: event.target.value,
                  },
                  () => {
                    if (
                      this.state.currentPort &&
                      this.state.currentPort === 'DJZ'
                    ) {
                      this.setState({
                        currentTrains: trainsDJZ,
                        currentYards: yardsDJZ,
                      });
                    } else if (
                      this.state.currentPort &&
                      this.state.currentPort === 'QDG'
                    ) {
                      this.setState({
                        currentTrains: trainsQD,
                        currentYards: yardsQD,
                      });
                    }
                  },
                );
              }}
            />
          </div>
          <div style={{ marginBottom: '10px' }}>
            <div className="simItem-title">
              <span>Storage_Yard_Waiting: </span>
              <button
                className="header-button"
                onClick={() => {
                  if (this.state.currentYard) {
                    this.props.store.scenarioEditingManager.sendDemonstrateRequest(
                      this.state.currentYard,
                      this.state.currentType,
                      this.state.blacklistedLane,
                    );
                  } else {
                    alert('Please select a yard');
                  }
                }}
              >
                Send
              </button>
            </div>
            <CoordinateSelector
              options={this.state.currentYards}
              onChange={this.handleSelectYard}
            />
          </div>
          <div
            style={{
              marginBottom: '10px',
              display: this.state.currentPort === 'DJZ' ? 'none' : 'block',
            }}
          >
            <div className="simItem-title">
              <span>Train_Waiting_Area: </span>
              <button
                className="header-button"
                onClick={() => {
                  if (this.state.currentTrain) {
                    this.props.store.scenarioEditingManager.sendDemonstrateRequest(
                      this.state.currentTrain,
                      this.state.currentType,
                      this.state.blacklistedLane,
                    );
                  } else {
                    alert('Please select a train');
                  }
                }}
              >
                Send
              </button>
            </div>
            <CoordinateSelector
              options={this.state.currentTrains}
              onChange={this.handleSelectTrain}
            />
          </div>
          <button
            className="header-button"
            style={{ width: '100%', marginTop: '10px' }}
            onClick={() => this.handleSubmitArrived()}
          >
            Immediately arrive at the station
          </button>
          <button
            className="header-button"
            style={{ width: '100%', marginTop: '10px' }}
            onClick={() => this.handleLoopRunning()}
          >
            Start running loop test
          </button>
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <CheckboxItem
              id="change_Prolonged_FASAEB"
              title="LongTerm FAS-AEB"
              isChecked={this.props.store.hmi.FASAEBStatus}
              extraClasses="AEB-checkbox"
              onClick={() => {
                if (this.props.store.hmi.FASAEBStatus) {
                  this.setState({ showDoublePsdModal: true });
                } else {
                  WS.changeFASAEBStatus(true, false, null);
                }
              }}
            />
            <CheckboxItem
              id="change_ShortTerm_FASAEB"
              title="ShortTerm FAS-AEB"
              isChecked={this.props.store.hmi.FASAEBStatus}
              extraClasses="AEB-checkbox"
              onClick={() => {
                if (this.props.store.hmi.FASAEBStatus) {
                  this.setState({ showPsdModal: true });
                } else {
                  WS.changeFASAEBStatus(true, true, null);
                }
              }}
            />
          </div>
          <div style={{ marginBottom: '10px' }}>
            <input
              type="number"
              step={0.1}
              min={0}
              placeholder="Please enter number"
              value={this.state.fineTuningNumber}
              onChange={(event) => {
                const fineTuningNumber = parseFloat(event.target.value);
                this.setState({ fineTuningNumber });
              }}
              className="input-text"
              style={{ backgroundColor: '#000000' }}
            />
            <button
              className="header-button"
              style={{ width: '100%', marginTop: '10px' }}
              onClick={() =>
                WS.fineTuningRequest(
                  'TINY_ADJUSTMENT_FRONT',
                  this.state.fineTuningNumber,
                )
              }
            >
              TINY_ADJUSTMENT_FRONT
            </button>
            <button
              className="header-button"
              style={{ width: '100%', marginTop: '10px' }}
              onClick={() =>
                WS.fineTuningRequest(
                  'TINY_ADJUSTMENT_BACK',
                  this.state.fineTuningNumber,
                )
              }
            >
              TINY_ADJUSTMENT_BACK
            </button>
          </div>
        </div>
        <div style={{ width: '100%' }}>
          <h3>Boom Barrier Control</h3>
          <div style={{ marginBottom: '10px' }}>
            <CheckboxItem
              id="enable_barrier_command"
              title="Enable Barrier Command"
              isChecked={barrierCommandEnabled}
              extraClasses="AEB-checkbox"
              onClick={this.handleToggleBarrierCommand}
            />
          </div>
          {barrierCommandEnabled && (
            <div style={{ marginBottom: '10px' }}>
              <span>Command: </span>
              <Selector
                name="Barrier Command"
                options={this.state.barrierCommands}
                currentOption={this.state.currentBarrierCommand}
                onChange={(event) => {
                  this.setState({
                    currentBarrierCommand: event.target.value,
                  });
                }}
              />
              <button
                className="header-button"
                style={{ width: '100%', marginTop: '10px' }}
                onClick={this.handleSendBarrierCommand}
              >
                Send Barrier Command
              </button>
            </div>
          )}
        </div>
        <div style={{ width: '100%' }}>
          <h3>Edit Profile</h3>
          <div className="simItem-title">
            {this.state.showEditProfile ? (
              <button
                className="header-button"
                onClick={() => this.handleStartEditProfile()}
              >
                Edit
              </button>
            ) : (
              <Fragment>
                <button
                  className="header-button"
                  onClick={() => this.handleCancelEditProfile()}
                >
                  Cancle
                </button>
                <button
                  className="header-button"
                  onClick={() => this.handleSaveEditProfile()}
                >
                  Save
                </button>
              </Fragment>
            )}
          </div>
          {this.state.vehicleConfig &&
            Object.entries(this.state.vehicleConfig).map(([key, value]) => (
              <div key={key} style={{ margin: '10px 0' }}>
                <label>{key}:</label>
                <input
                  type="text"
                  value={value}
                  onChange={(event) => this.handleChange(key, event)}
                  className="input-text"
                  disabled={this.state.showEditProfile}
                />
              </div>
            ))}
        </div>
        <div style={{ width: '100%' }}>
          <h3>Save Location</h3>
          <div style={{ marginBottom: '10px' }}>
            <div className="simItem-title">
              <span>Location Name: </span>
              <button
                className="header-button"
                onClick={() => this.handleSaveCarPosition()}
              >
                Save
              </button>
            </div>
            <div>
              <input
                type="text"
                placeholder="Please enter a name to save the current location"
                value={this.state.locationName}
                onChange={(event) =>
                  this.setState({ locationName: event.target.value })
                }
                className="input-text"
                style={{ backgroundColor: '#000000' }}
              />
            </div>
          </div>
        </div>
        <div style={{ width: '100%' }}>
          <h3>Change BackgroundMusic</h3>
          <div style={{ marginBottom: '10px' }}>
            <CheckboxItem
              id="change_bckground_music_checkbox"
              title="Change BackgroundMusic"
              isChecked={this.props.store.hmi.bckGroundMusicStatus}
              extraClasses="AEB-checkbox"
              onClick={() => {
                WS.changeBckGroundMusicStatus(!this.props.store.hmi.bckGroundMusicStatus);
              }}
            />
          </div>
        </div>
      </div>
    );
  }
}
