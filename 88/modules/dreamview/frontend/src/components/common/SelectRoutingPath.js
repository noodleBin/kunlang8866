import React, { Component } from 'react';
import './SelectRoutingPath.scss';
import RENDERER from 'renderer';
import STORE from 'store';
import WS from 'store/websocket';

export default class SelectRoutingPath extends Component {
  constructor(props) {
    super(props);
    this.state = {
      routeID: 0,
    };
  }

  selectRoutePath = () => {
    WS.selectRoutePath(this.state.routeID);
    STORE.options.toggle('showSelectRoutePath');
  };

  cancleSelect = () => {
    STORE.options.toggle('showSelectRoutePath');
  };

  changeRoutePath = (item) => {
    RENDERER.routing.drawRoutingPath(
      [item],
      RENDERER.coordinates,
      RENDERER.scene,
    );
    this.setState({ routeID: item.id });
  };

  render() {
    const { isOpen } = this.props;
    if (!isOpen) {
      return null;
    }

    return (
      <div className="routing-path-modal">
        <div className="routing-path-modal__content">
          {RENDERER.routing.allRoutes.length !== 0 &&
            RENDERER.routing.allRoutes.routePath.map((item) => {
              return (
                <div
                  className="routing-path-modal__item"
                  key={item.id}
                  onClick={() => this.changeRoutePath(item)}
                >
                  {'Routing_path: ' + item.id}
                </div>
              );
            })}
        </div>
        <div className="routing-path-modal__footer">
          <div className="routing-path-modal__footer__buttons">
            <button onClick={() => this.selectRoutePath()}>
              Send routing path
            </button>
            <button onClick={() => this.cancleSelect()}>Cancle</button>
          </div>
        </div>
      </div>
    );
  }
}
