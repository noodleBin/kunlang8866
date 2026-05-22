import React from 'react';
import {
  Tab, Tabs, TabList, TabPanel,
} from 'react-tabs';

import DriveEventEditor from 'components/DataRecorder/DriveEventEditor';
import AudioEventEditor from 'components/DataRecorder/AudioEventEditor';
import RecorderEventEditor from 'components/DataRecorder/RecorderEventEditor';

export default class DataRecorder extends React.Component {
  render() {
    const { allChannels } = this.props;

    return (
      <div className="card data-recorder">
        <Tabs>
            <TabList>
              <Tab>Add Drive Event</Tab>
              <Tab>Add Audio Event</Tab>
              <Tab>One click packet recording</Tab>
            </TabList>
            <TabPanel>
                <DriveEventEditor />
            </TabPanel>
            <TabPanel>
                <AudioEventEditor />
            </TabPanel>
            <TabPanel>
                <RecorderEventEditor
                  allChannels={allChannels}
                />
            </TabPanel>
        </Tabs>
      </div>
    );
  }
}
