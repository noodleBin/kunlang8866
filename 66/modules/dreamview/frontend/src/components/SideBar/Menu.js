import React from 'react';
import { inject, observer } from 'mobx-react';
import _ from 'lodash';

import RadioItem from 'components/common/RadioItem';

import menuData from 'store/config/MenuData';
import perceptionIcon from 'assets/images/menu/perception.png';
import predictionIcon from 'assets/images/menu/prediction.png';
import routingIcon from 'assets/images/menu/routing.png';
import decisionIcon from 'assets/images/menu/decision.png';
import planningIcon from 'assets/images/menu/planning.png';
import cameraIcon from 'assets/images/menu/point_of_view.png';
import positionIcon from 'assets/images/menu/position.png';
import mapIcon from 'assets/images/menu/map.png';

import { POINT_CLOUD_WS } from 'store/websocket';

import './style.scss';

const MenuIconMapping = {
  perception: perceptionIcon,
  prediction: predictionIcon,
  routing: routingIcon,
  decision: decisionIcon,
  planning: planningIcon,
  camera: cameraIcon,
  position: positionIcon,
  map: mapIcon,
};

@inject('store') @observer
class MenuItemCheckbox extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      channels: [],
    };
  }

  componentDidMount() {
    const { id, options } = this.props;
    if (id === 'perceptionPointCloud') {
      POINT_CLOUD_WS.getPointCloudChannel()
        .then((channels) => {
          this.setState({ channels }, () => {
            POINT_CLOUD_WS.togglePointCloud(options.showPointCloud);
          });
        })
        .catch((err) => {
          this.setState({ channels: [] }, () => {
            if (this.state.channels.length === 0) {
              POINT_CLOUD_WS.togglePointCloud(false);
            }
          });
        });
    }
  }

  onStatusSelectChange = (e) => {
    const value = e.target.value;
    if (value) {
      POINT_CLOUD_WS.changePointCloudChannel(value)
        .then((res) => {
          const newChannels = this.state.channels.map((item) => {
            return {
              ...item,
              status: item.name === res ? !item.status : item.status,
            };
          });
          this.setState({ channels: newChannels });
        })
        .catch((err) => {
          console.error(err);
        });
    }
  };

  render() {
    const { id, title, optionName, options, isCustomized, store } = this.props;
    const { hmi } = store;

    return (
      <ul className="item">
        <li
          id={id}
          onClick={() => {
            if (id === 'perceptionPointCloud') {
              if (this.state.channels.length === 0) {
                return alert('There is currently no point cloud data available, unable to open it!');
              }
              options.toggle(optionName, isCustomized);
              POINT_CLOUD_WS.togglePointCloud(options.showPointCloud);
            } else {
              options.toggle(optionName, isCustomized);
            }
          }}
        >
          <div className="switch">
            <input
              type="checkbox"
              name={id}
              className="toggle-switch"
              id={id}
              checked={
                isCustomized
                  ? options.customizedToggles.get(optionName)
                  : options[optionName]
              }
              readOnly
            />
            <label className="toggle-switch-label" htmlFor={id} />
          </div>
          <span>{title}</span>
          {
            id === 'perceptionPointCloud' && this.state.channels.length !== 0 &&
            <span className='point_cloud_channel_select'>
              <div className="selector" onClick={(e) => e.stopPropagation()}>
                <ul class="select">
                  {
                    this.state.channels.map((channel) => {
                      return (
                        <li key={channel.name}>
                          <label>
                            <input type="checkbox"
                              value={channel.name}
                              checked={channel.status === true}
                              onChange={this.onStatusSelectChange}
                              disabled={options.showPointCloud === false}
                            />
                            {channel.name}
                          </label>
                        </li>
                      );
                    })
                  }
                </ul>
                <div class="slider">
                    <input type="range" min="0.01" max="0.28" step="0.01"
                      value={hmi.pointSize}
                      onChange={(event) => hmi.changePointSize(event)}
                    />
                    <p id="rangeValue">{hmi.pointSize}</p>
                </div>
              </div>
            </span>
          }
        </li>
      </ul>
    );
  }
}

@observer
class SubMenu extends React.Component {
  constructor(props) {
    super(props);

    this.menuIdOptionMapping = {};
    for (const name in PARAMETERS.options) {
      const option = PARAMETERS.options[name];
      if (option.menuId) {
        this.menuIdOptionMapping[option.menuId] = name;
      }
    }
  }

  render() {
    const {
      tabId, tabTitle, tabType, data, options,
    } = this.props;
    let entries = null;
    if (tabType === 'checkbox') {
      entries = Object.keys(data)
        .map((key) => {
          const item = data[key];
          if (options.togglesToHide[key]) {
            return null;
          }
          return (
            <MenuItemCheckbox
              key={key}
              id={key}
              title={item}
              optionName={this.menuIdOptionMapping[key]}
              options={options}
              isCustomized={false}
            />
          );
        });
      if (tabId === 'planning' && options.customizedToggles.size > 0) {
        const extraEntries = options.customizedToggles.keys().map((pathName) => {
          const title = _.startCase(_.snakeCase(pathName));
          return (
            <MenuItemCheckbox
              key={pathName}
              id={pathName}
              title={title}
              optionName={pathName}
              options={options}
              isCustomized
            />
          );
        });
        entries = entries.concat(extraEntries);
      }
    } else if (tabType === 'radio') {
      // Now we only have camera tab using radio in menu
      if (tabId === 'camera') {
        const cameraAngles = Object.values(data)
          .filter((angle) => PARAMETERS.options.cameraAngle[`has${angle}`] !== false);
        entries = cameraAngles.map((item) => (
          <RadioItem
            key={`${tabId}_${item}`}
            id={tabId}
            onClick={() => {
              options.selectCamera(item);
            }}
            checked={options.cameraAngle === item}
            title={_.startCase(item)}
          />
        ));
      }
    }
    const result = (
      <div className="card">
        <div className="card-header summary">
          <span>
            <img src={MenuIconMapping[tabId]} />
            {tabTitle}
          </span>
        </div>
        <div className="card-content-column">{entries}</div>
      </div>
    );
    return result;
  }
}

@observer
export default class LayerMenu extends React.Component {
  render() {
    const { options } = this.props;

    const subMenu = Object.keys(menuData)
      .map((key) => {
        const item = menuData[key];

        if (OFFLINE_PLAYBACK && !item.supportInOfflineView) {
          return null;
        }
        return (
          <SubMenu
            key={item.id}
            tabId={item.id}
            tabTitle={item.title}
            tabType={item.type}
            data={item.data}
            options={options}
          />
        );
      });

    return (
      <div className="tool-view-menu" id="layer-menu">
        {subMenu}
      </div>
    );
  }
}
