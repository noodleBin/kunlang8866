import React from 'react';
import { inject, observer } from 'mobx-react';
import classNames from 'classnames';

@observer
export class SystemInfoItem extends React.Component {
  render() {
    const { text, info } = this.props;

    return (
      <li className="monitor-item">
        <span className={classNames('text', 'warn')}>{text}</span>
        <span className={classNames('time', 'warn')}>{info}</span>
      </li>
    );
  }
}

@inject('store')
@observer
export default class Carstatus extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      isOpen: true,
    };
  }

  setIsOpen = (isOpen) => {
    this.setState({ isOpen });
  };

  render() {
    const { monitor } = this.props;
    const { isOpen } = this.state;

    return (
      <div className="rss" style={{ height: 'auto' }}>
        <div
          className="rss-header"
          style={{
            display: 'flex',
            justifyContent: 'space-around',
            alignItems: 'center',
            cursor: 'pointer',
          }}
          onClick={() => this.setIsOpen(!isOpen)}
        >
          <div>Car Status Info</div>
          <div>{isOpen ? '▲' : '▼'}</div>
        </div>
        {isOpen && (
          <div className="rss-content-column">
            <ul className="rss-console">
              <SystemInfoItem text={'CPU Usage:'} info={monitor.cpuInfo} />
              <SystemInfoItem text={'Disk Usage:'} info={monitor.diskInfo} />
              <SystemInfoItem
                text={'Memory Usage:'}
                info={monitor.memoryInfo}
              />
              <SystemInfoItem text={'AD status:'} info={monitor.statusInfo} />
              <SystemInfoItem text={'Version:'} info={monitor.centuryVersion} />
            </ul>
          </div>
        )}
      </div>
    );
  }
}
