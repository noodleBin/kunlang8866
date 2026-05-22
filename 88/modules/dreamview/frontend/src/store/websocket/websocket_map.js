import STORE from 'store';
import RENDERER from 'renderer';
import Worker from 'utils/webworker.js';

export default class MapDataWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.currentMode = null;
    this.currentMap = null;
    this.worker = new Worker();
    this.messageQueue = [];
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
        source: 'map',
        data: event.data,
      });
    };
    this.worker.onmessage = (event) => {
      const removeOldMap =
        STORE.hmi.inNavigationMode ||
        this.currentMode !== STORE.hmi.currentMode ||
        this.currentMap !== STORE.hmi.currentMap;
      this.currentMode = STORE.hmi.currentMode;
      this.currentMap = STORE.hmi.currentMap;
      RENDERER.updateMap(event.data, removeOldMap);
      STORE.setInitializationStatus(true);
    };
    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };
    this.websocket.addEventListener('open', () => {
      this.messageQueue.forEach((data) => this.websocket.send(data));
      this.messageQueue = [];
    });
  }

  requestMapData(elements) {
    if (this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(
        JSON.stringify({
          type: 'RetrieveMapData',
          elements,
        }),
      );
    } else {
      this.messageQueue.push(
        JSON.stringify({
          type: 'RetrieveMapData',
          elements,
        }),
      );
    }
  }

  requestRelativeMapData(elements) {
    if (this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(
        JSON.stringify({
          type: 'RetrieveRelativeMapData',
          elements,
        }),
      );
    } else {
      this.messageQueue.push(
        JSON.stringify({
          type: 'RetrieveRelativeMapData',
          elements,
        }),
      );
    }
  }
}
