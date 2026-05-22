import React from 'react';
import { inject, observer } from 'mobx-react';
import classNames from 'classnames';

import UTTERANCE from 'store/utterance';
import WS from 'store/websocket';

class CommandGroup extends React.Component {
  render() {
    const { name, commands, disabled, extraCommandClass, extraButtonClass } =
      this.props;

    const entries = Object.keys(commands).map((key) => (
      <button
        className={classNames('command-button', extraButtonClass)}
        disabled={disabled}
        key={key}
        onClick={commands[key]}
      >
        {key}
      </button>
    ));

    const text = name ? <span className="name">{`${name}:`}</span> : null;

    return (
      <div className={classNames('command-group', extraCommandClass)}>
        {text}
        {entries}
      </div>
    );
  }
}

@inject('store')
@observer
export default class QuickStarter extends React.Component {
  constructor(props) {
    super(props);

    this.setup = {
      Setup: () => {
        WS.executeModeCommand('SETUP_MODE');
        UTTERANCE.speakOnce('Setup');
      },
    };

    this.startRecord = {
      'Start Record': () => {
        WS.sendStartRecord();
      },
    };

    this.reset = {
      'Reset All': () => {
        WS.executeModeCommand('RESET_MODE');
        UTTERANCE.speakOnce('Reset All');
      },
    };

    this.restart = {
      'Restart Century': () => {
        WS.executeModeCommand('RESTART_MODULE');
        UTTERANCE.speakOnce('RESTART CENTURY');
      },
    };

    this.auto = {
      'Start Auto': () => {
        WS.executeModeCommand('ENTER_AUTO_MODE');
        UTTERANCE.speakOnce('Start Auto');
      },
    };
  }

  componentWillUpdate() {
    UTTERANCE.cancelAllInQueue();
  }

  render() {
    const { hmi } = this.props.store;
    const { lockTaskPanel } = this.props.store.options;

    return (
      <div className="card">
        <div className="card-header">
          <span>Quick Start</span>
        </div>
        <div className="card-content-column">
          <CommandGroup disabled={lockTaskPanel} commands={this.setup} />
          {console.log(hmi.recordInfoStatus)}
          <CommandGroup
            disabled={hmi.recordInfoStatus}
            commands={this.startRecord}
          />
          <CommandGroup disabled={lockTaskPanel} commands={this.reset} />
          <CommandGroup disabled={lockTaskPanel} commands={this.restart} />
          <CommandGroup
            disabled={!hmi.enableStartAuto || lockTaskPanel}
            commands={this.auto}
            extraButtonClass="start-auto-button"
            // extraCommandClass="start-auto-command"
          />
        </div>
      </div>
    );
  }
}
