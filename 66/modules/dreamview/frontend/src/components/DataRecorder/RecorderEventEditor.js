import React from 'react';
import _ from 'lodash';

import WS from 'store/websocket';

export default class RecorderEventEditor extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      allChannels: this.props.allChannels,
      isAllSelected: false,
      selectedOptions: [],
    };
    this.handleStart = this.handleStart.bind(this);
    this.handleStop = this.handleStop.bind(this);
    this.handleSelectAll = this.handleSelectAll.bind(this);
    this.handleCheckboxChange = this.handleCheckboxChange.bind(this);
  }

  componentDidMount() {
    WS.getAllChannels();
  }

  componentWillReceiveProps(nextProps) {
    if (nextProps.allChannels) {
      this.setState({ allChannels: nextProps.allChannels });
    }
  }

  handleStart(event) {
    event.preventDefault();
    if (this.state.selectedOptions.length === 0) {
      return alert('Please select at least one channel');
    }
    WS.StartRecord(this.state.selectedOptions);
  }

  handleStop() {
    WS.stopRecord();
  }

  handleSelectAll() {
    const { allChannels, isAllSelected} = this.state;
    if (!isAllSelected) {
      this.setState({
        selectedOptions: allChannels,
        isAllSelected: true,
      });
    } else {
      this.setState({
        selectedOptions: [],
        isAllSelected: false,
      });
    }
  }

  handleCheckboxChange(event) {
    const { selectedOptions } = this.state;
    const value = event.target.value;
    if (event.target.checked) {
      this.setState({ selectedOptions: [...selectedOptions, value] });
    } else {
      this.setState({ selectedOptions: selectedOptions.filter((item) => item !== value) });
    }
  }

  render() {
    return (
      <React.Fragment>
        <div className="recorder-item">
          <div className='recorder-item-header'>
            <button className="submit-button" onClick={this.handleSelectAll}>
              Select All
            </button>
            <button className="submit-button" onClick={this.handleStart}>
              Start
            </button>
            <button className="submit-button" onClick={this.handleStop}>
              Stop
            </button>
          </div>

          <div className='recorder-item-body'>
            {this.state.allChannels.map((item) => (
              <div key={item} className='checkbox-container'>
                <input
                  type="checkbox"
                  value={item}
                  checked={this.state.selectedOptions.includes(item)}
                  onChange={this.handleCheckboxChange}
                  id={item}
                />
                <label for={item}>{item}</label>
              </div>
            ))}
          </div>
        </div>
      </React.Fragment>
    );
  }
}
