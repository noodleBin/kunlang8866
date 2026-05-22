import React from 'react';
import { inject, observer } from 'mobx-react';
import ScenarioCarBar from './ScenarioCarBar';
import ScenarioObstacleBar from './ScenarioObstacleBar';

@inject('store') @observer
export default class EditScenarioSetting extends React.Component {

  render() {

    return (
      <div className="scenario-monitor teleop">
        <ScenarioCarBar />
        <ScenarioObstacleBar />
      </div>
    );
  }
}
