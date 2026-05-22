import STORE from 'store';
import RENDERER from 'renderer';
import Worker from 'utils/webworker.js';
import { safeParseJSON } from 'store/websocket/websocket_camera.js';

export default class PointCloudWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.worker = new Worker();
    this.enabled = false;
  }

  initialize() {
    try {
      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return;
    }
    this.websocket.onmessage = (event) => {
      this.worker.postMessage({
        source: 'point_cloud',
        data: event.data,
      });
    };
    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };
    this.worker.onmessage = (event) => {
      if (event.data.type === 'PointCloudStatus') {
        STORE.setOptionStatus('showPointCloud', event.data.enabled);
        if (STORE.options.showPointCloud === false) {
          RENDERER.updatePointCloud({ num: [] });
        }
      } else if (
        STORE.options.showPointCloud === true &&
        event.data.num !== undefined
      ) {
        RENDERER.updatePointCloud(event.data);
      }
    };
  }

  requestPointCloud() {
    if (
      this.websocket.readyState === this.websocket.OPEN &&
      STORE.options.showPointCloud === true
    ) {
      this.websocket.send(
        JSON.stringify({
          type: 'RequestPointCloud',
        }),
      );
    }
  }

  isEnabled() {
    return this.enabled;
  }

  togglePointCloud(enable) {
    this.enabled = enable;
    this.websocket.send(
      JSON.stringify({
        type: 'TogglePointCloud',
        enable,
      }),
    );
    if (STORE.options.showPointCloud === false) {
      RENDERER.updatePointCloud({ num: [] });
    }
  }

  getPointCloudChannel() {
    return new Promise((resolve, reject) => {
      this.websocket.send(
        JSON.stringify({
          type: 'GetPointCloudChannel',
        }),
      );
      this.websocket.addEventListener('message', (event) => {
        if (event.data instanceof ArrayBuffer) {
          return;
        }
        const message = safeParseJSON(event.data);
        if (
          message &&
          message.data &&
          message.data.name === 'GetPointCloudChannelListSuccess'
        ) {
          if (
            message &&
            message.data &&
            message.data.info &&
            message.data.info.channel
          ) {
            resolve(
              message &&
                message.data &&
                message.data.info &&
                message.data.info.channel,
            );
          } else {
            reject(message && message.data && message.data.info);
          }
        } else if (
          message &&
          message.data &&
          message.data.name === 'GetPointCloudChannelListFail'
        ) {
          reject(message && message.data && message.data.info);
        }
      });
    });
  }

  changePointCloudChannel(channel) {
    return new Promise((resolve, reject) => {
      this.websocket.send(
        JSON.stringify({
          type: 'ChangePointCloudChannel',
          data: {
            channel,
          }
        }),
      );
      this.websocket.addEventListener('message', (event) => {
        if (event.data instanceof ArrayBuffer) {
          return;
        }
        const message = safeParseJSON(event.data);
        if (message && message.type === 'ChangePointCloudChannelSuccess') {
          resolve(channel);
        } else if (message && message.type === 'ChangePointCloudChannelFail') {
          reject('ChangePointCloudChannelFail');
        }
      });
    });
  }
}
