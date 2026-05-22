import { observable, action } from 'mobx';

import _ from 'lodash';

import RENDERER from 'renderer';
import MAP_NAVIGATOR from 'components/Navigation/MapNavigator';
import {
  ObstacleColorMapping,
  DEFAULT_PEDESTRIAN_SIZE,
  DEFAULT_BICYCLE_SIZE,
  DEFAULT_VEHICLE_SIZE,
  DEFAULT_UNKNOWN_SIZE,
  DEFAULT_STACKER_SIZE,
  DEFAULT_FORKLIFTSTACKER_SIZE,
  DEFAULT_WHEELCRANE_SIZE
} from 'renderer/obstacles.js';

export const TEMPORARY_MOVE_TASK_TYPE = Object.freeze({
  SEARCH_REACH_STACKER: 'SEARCH_REACH_STACKER',
  TEMPORARY_VEH_RELOCATION: 'TEMPORARY_VEH_RELOCATION',
});

export default class ScenarioEditingManager {

  @observable routingPoints = [];
  @observable obstaclesList = [];
  @observable curObstacle = {};
  @observable curObstacleId = null;
  @observable isLoopRunning = false;
  @observable isTemporaryMoveSelecting = false;
  @observable temporaryMovePointSelected = false;
  @observable isTemporaryMoveActive = false;
  @observable temporaryMoveTaskType =
    TEMPORARY_MOVE_TASK_TYPE.SEARCH_REACH_STACKER;
  @observable activeTemporaryMoveTaskType = null;

  parkingRoutingDistanceThreshold = 20.0;

  enableMeasuringDistance() {
    RENDERER.enableMeasuringDistance();
    RENDERER.enableOrbitControls(false);
  }

  disableMeasuringDistance(cameraAngle) {
    RENDERER.disableMeasuringDistance();
    if (cameraAngle === 'Map') {
      RENDERER.enableOrbitControls(true);
    }
  }

  updateCurSorStyle(curSorStyle) {
    RENDERER.updateCurSorStyle(curSorStyle);
  }

  enableScenarioEditing() {
    RENDERER.enableScenarioEditing();
  }

  disableScenarioEditing() {
    RENDERER.disableScenarioEditing();
  }

  enableObstacleEditing() {
    RENDERER.enableObstacleEditing();
  }

  disableObstacleEditing() {
    RENDERER.disableObstacleEditing();
  }

  enableAddObstaclePathPoint() {
    RENDERER.enableAddObstaclePathPoint();
  }

  disableAddObstaclePathPoint() {
    RENDERER.disableAddObstaclePathPoint();
  }

  enableAddObstacleTriggerPoint() {
    RENDERER.enableAddObstacleTriggerPoint();
  }

  disableAddObstacleTriggerPoint() {
    RENDERER.disableAddObstacleTriggerPoint();
  }

  removeOneObstacleTracePoint(id) {
    RENDERER.removeOneObstacleTracePoint(id);
    this.curObstacle.trace = this.curObstacle.trace.filter(item => item.id !== id);
    if (this.curObstacle.trace.length !== 0) {
      this.curObstacle.trace[this.curObstacle.trace.length - 1].speed = 0;
    }
  }

  removeAllObstacleTracePoints() {
    RENDERER.removeAllObstacleTracePoints();
    this.curObstacle.trace = [];
  }

  removeOneObstacleTriggerPoint(id) {
    RENDERER.removeOneObstacleTriggerPoint(id);
    this.curObstacle.trigger_position =
      this.curObstacle.trigger_position.filter(item => item.id !== id);
  }

  removeAllObstacles() {
    RENDERER.removeAllObstacles();
    this.obstaclesList = [];
    this.curObstacle = {};
    this.curObstacleId = null;
  }

  removeOneObstacle() {
    RENDERER.removeOneObstacle(this.curObstacle);
    this.obstaclesList = this.obstaclesList.filter(item => item.id !== this.curObstacleId);
    this.curObstacle = this.obstaclesList.length !== 0 ? this.obstaclesList[0] : {};
    this.curObstacleId = this.obstaclesList.length !== 0 ? this.obstaclesList[0].id : null;
  }

  @action toggleLoopRunning() {
    this.isLoopRunning = !this.isLoopRunning;
  }

  @action addObstacle(obstacle) {
    this.obstaclesList.push(obstacle);
    this.curObstacle = this.obstaclesList[this.obstaclesList.length - 1];
    this.curObstacleId = obstacle.id;
  }

  @action changeCurObstacle(id,name = 'obstacleGroup') {
    if (name === 'obstacleGroup') {
      this.curObstacleId = id;
      this.curObstacle = this.obstaclesList.find(obstacle => {
        return obstacle.id === id;
      });
    } else if (name === 'trace') {
      this.curObstacle = this.obstaclesList.find(obstacle =>
        obstacle.trace.some(ele => ele.id === id)
      );
      this.curObstacleId = this.curObstacle.id;
    } else if (name === 'trigger') {
      this.curObstacle = this.obstaclesList.find(obstacle =>
        obstacle.trigger_position.some(ele => ele.id === id)
      );
      this.curObstacleId = this.curObstacle.id;
    }
  }

  @action changeType(event) {
    const color = ObstacleColorMapping[event.target.value];

    this.curObstacle.type = event.target.value;
    let multipleX = null;
    let multipleY = null;
    let multipleZ = null;

    if (event.target.value === 'PEDESTRIAN') {
      this.curObstacle.width = DEFAULT_PEDESTRIAN_SIZE.X;
      this.curObstacle.length = DEFAULT_PEDESTRIAN_SIZE.Y;
      this.curObstacle.height = DEFAULT_PEDESTRIAN_SIZE.Z;
      multipleX = Number(DEFAULT_PEDESTRIAN_SIZE.X / DEFAULT_UNKNOWN_SIZE.X);
      multipleY = Number(DEFAULT_PEDESTRIAN_SIZE.Y / DEFAULT_UNKNOWN_SIZE.Y);
      multipleZ = Number(DEFAULT_PEDESTRIAN_SIZE.Z / DEFAULT_UNKNOWN_SIZE.Z);
    }

    if (event.target.value === 'BICYCLE') {
      this.curObstacle.width = DEFAULT_BICYCLE_SIZE.X;
      this.curObstacle.length = DEFAULT_BICYCLE_SIZE.Y;
      this.curObstacle.height = DEFAULT_BICYCLE_SIZE.Z;
      multipleX = Number(DEFAULT_BICYCLE_SIZE.X / DEFAULT_UNKNOWN_SIZE.X);
      multipleY = Number(DEFAULT_BICYCLE_SIZE.Y / DEFAULT_UNKNOWN_SIZE.Y);
      multipleZ = Number(DEFAULT_BICYCLE_SIZE.Z / DEFAULT_UNKNOWN_SIZE.Z);
    }

    if (event.target.value === 'VEHICLE') {
      this.curObstacle.width = DEFAULT_VEHICLE_SIZE.X;
      this.curObstacle.length = DEFAULT_VEHICLE_SIZE.Y;
      this.curObstacle.height = DEFAULT_VEHICLE_SIZE.Z;
      multipleX = Number(DEFAULT_VEHICLE_SIZE.X / DEFAULT_UNKNOWN_SIZE.X);
      multipleY = Number(DEFAULT_VEHICLE_SIZE.Y / DEFAULT_UNKNOWN_SIZE.Y);
      multipleZ = Number(DEFAULT_VEHICLE_SIZE.Z / DEFAULT_UNKNOWN_SIZE.Z);
    }

    if (event.target.value === 'STACKER') {
      this.curObstacle.width = DEFAULT_STACKER_SIZE.X;
      this.curObstacle.length = DEFAULT_STACKER_SIZE.Y;
      this.curObstacle.height = DEFAULT_STACKER_SIZE.Z;
      multipleX = Number(DEFAULT_STACKER_SIZE.X / DEFAULT_UNKNOWN_SIZE.X);
      multipleY = Number(DEFAULT_STACKER_SIZE.Y / DEFAULT_UNKNOWN_SIZE.Y);
      multipleZ = Number(DEFAULT_STACKER_SIZE.Z / DEFAULT_UNKNOWN_SIZE.Z);
    }

    if (event.target.value === 'FORKLIFT_STACKER') {
      this.curObstacle.width = DEFAULT_FORKLIFTSTACKER_SIZE.X;
      this.curObstacle.length = DEFAULT_FORKLIFTSTACKER_SIZE.Y;
      this.curObstacle.height = DEFAULT_FORKLIFTSTACKER_SIZE.Z;
      multipleX = Number(DEFAULT_FORKLIFTSTACKER_SIZE.X / DEFAULT_UNKNOWN_SIZE.X);
      multipleY = Number(DEFAULT_FORKLIFTSTACKER_SIZE.Y / DEFAULT_UNKNOWN_SIZE.Y);
      multipleZ = Number(DEFAULT_FORKLIFTSTACKER_SIZE.Z / DEFAULT_UNKNOWN_SIZE.Z);
    }

    if (event.target.value === 'WHEELCRANE') {
      this.curObstacle.width = DEFAULT_WHEELCRANE_SIZE.X;
      this.curObstacle.length = DEFAULT_WHEELCRANE_SIZE.Y;
      this.curObstacle.height = DEFAULT_WHEELCRANE_SIZE.Z;
      multipleX = Number(DEFAULT_WHEELCRANE_SIZE.X / DEFAULT_UNKNOWN_SIZE.X);
      multipleY = Number(DEFAULT_WHEELCRANE_SIZE.Y / DEFAULT_UNKNOWN_SIZE.Y);
      multipleZ = Number(DEFAULT_WHEELCRANE_SIZE.Z / DEFAULT_UNKNOWN_SIZE.Z);
    }

    if (event.target.value === 'UNKNOWN') {
      this.curObstacle.width = DEFAULT_UNKNOWN_SIZE.X;
      this.curObstacle.length = DEFAULT_UNKNOWN_SIZE.Y;
      this.curObstacle.height = DEFAULT_UNKNOWN_SIZE.Z;
      multipleX = Number(DEFAULT_UNKNOWN_SIZE.X / DEFAULT_UNKNOWN_SIZE.X);
      multipleY = Number(DEFAULT_UNKNOWN_SIZE.Y / DEFAULT_UNKNOWN_SIZE.Y);
      multipleZ = Number(DEFAULT_UNKNOWN_SIZE.Z / DEFAULT_UNKNOWN_SIZE.Z);
    }
    const curObstacleItem = RENDERER.scene.children.filter(item =>
      item.groupId === this.curObstacleId);
    curObstacleItem[0].children.forEach(el => {
      el.scale.set(multipleX,multipleY,multipleZ);
      el.material.color.set(color);
      el.position.setZ(multipleZ / 2);
    });
    const curTracePoints = RENDERER.scene.children.filter(item =>
      this.curObstacle.trace.some(ele => item.obstacleTraceId === ele.id));
    curTracePoints.forEach(el => el.material.color.set(color));
    const curTriggerPoints = RENDERER.scene.children.filter(item =>
      this.curObstacle.trigger_position.some(ele => item.obstacleTriggerId === ele.id));
    curTriggerPoints.forEach(el => el.material.color.set(color));
  }

  @action changeSpeed(event) {
    this.curObstacle.speed = Number(event.target.value);
  }

  @action changeWidth(event) {
    const multipleX = Number(event.target.value / DEFAULT_UNKNOWN_SIZE.X);
    const curObstacleItem = RENDERER.scene.children.filter(item =>
      item.groupId === this.curObstacleId);
    curObstacleItem[0].children.forEach(el => {
      el.scale.setX(multipleX);
    });
    this.curObstacle.width = Number(event.target.value);
  }

  @action changeLength(event) {
    const multipleY = Number(event.target.value / DEFAULT_UNKNOWN_SIZE.Y);
    const curObstacleItem = RENDERER.scene.children.filter(item =>
      item.groupId === this.curObstacleId);
    curObstacleItem[0].children.forEach(el => {
      el.scale.setY(multipleY);
    });
    this.curObstacle.length = Number(event.target.value);
  }

  @action changeHeight(event) {
    const multipleZ = Number(event.target.value / DEFAULT_UNKNOWN_SIZE.Z);
    const curObstacleItem = RENDERER.scene.children.filter(item =>
      item.groupId === this.curObstacleId);
    curObstacleItem[0].children.forEach(el => {
      el.scale.setZ(multipleZ);
      el.position.setZ(multipleZ / 2);
    });
    this.curObstacle.height = Number(event.target.value);
  }

  @action changeObstaclePosition(geo) {
    this.curObstacle.position = [geo.x, geo.y, 0];
  }

  @action changeTracePosition(geo,id) {
    this.curObstacle.trace.find(ele => ele.id === id).position = [geo.x, geo.y, 0];
  }

  @action changeTriggerPosition(geo) {
    this.curObstacle.trigger_position[0].position = [geo.x, geo.y, 0];
  }

  @action changeMoveWay(event) {
    this.curObstacle.move_way = event.target.value;
    this.curObstacle.trigger_radius = 1;
    this.curObstacle.speed = 0;
    if (this.curObstacle.trace.length > 0) {
      this.removeAllObstacleTracePoints();
    }
    if (this.curObstacle.trigger_position.length > 0) {
      const triggerId = this.curObstacle.trigger_position[0].id;
      this.removeOneObstacleTriggerPoint(triggerId);
    }
  }

  @action addObstacleTrace(point) {
    const traceItem = {
      id: this.curObstacleId + '-'  + point.id,
      position: [point.x, point.y, 0],
      speed: 1
    };
    this.curObstacle.trace.push(traceItem);
  }

  @action addObstacleTrigger(point) {
    const triggerItem = {
      id: this.curObstacleId + '-' + point.id,
      position: [point.x, point.y, 0],
    };
    this.curObstacle.trigger_position.push(triggerItem);
  }

  @action changeTriggerRadius(event) {
    const triggerRadius = Number(event.target.value);
    this.curObstacle.trigger_radius = triggerRadius;
    const curTriggerPoint = RENDERER.scene.children.find(item =>
      item.obstacleTriggerId === this.curObstacle.trigger_position[0].id
    );
    curTriggerPoint.scale.set(triggerRadius,triggerRadius,triggerRadius);
  }

  @action changeObstacleTraceSpeed(event,id) {
    this.curObstacle.trace.find(item => item.id === id).speed =
      Number(event.target.value);
  }

  @action delObstacleTracePoint(id) {
    this.removeOneObstacleTracePoint(id);
  }

  @action delObstacleTriggerPoint(id) {
    this.removeOneObstacleTriggerPoint(id);
  }

  @action delCurObstacle() {
    this.removeOneObstacle(this.curObstacleId);
  }

  sendDemonstrateRequest(endPoint, endType, blacklistedLane) {
    RENDERER.sendDemonstrateRequest(endPoint,endType, blacklistedLane);
  }

  saveCurCarPosition(name) {
    RENDERER.saveCurCarPosition(name);
  }

  startLoopRunning() {
    RENDERER.startLoopRunning();
  }

  enableRouteEditing() {
    RENDERER.enableRouteEditing();
  }

  disableRouteEditing() {
    RENDERER.disableRouteEditing();
  }

  removeAllRoutingPoints() {
    RENDERER.removeAllRoutingPoints();
    this.routingPoints = [];
  }

  removeOneRoutingPoint(id) {
    RENDERER.removeOneRoutingPoint(id);
    this.routingPoints = this.routingPoints.filter(item => item.id !== id);
  }

  @action addPoint(point) {
    this.routingPoints.push(point);
  }

  sendRoutingRequest(inNavigationMode) {
    if (!inNavigationMode) {
      const success = RENDERER.sendRoutingRequest();
      if (success) {
        this.disableRouteEditing();
      }
      return success;
    }
    return MAP_NAVIGATOR.sendRoutingRequest();
  }

  @action startTemporaryMoveSelectingMode() {
    this.isTemporaryMoveSelecting = true;
    this.temporaryMovePointSelected = false;
  }

  @action stopTemporaryMoveSelectingMode() {
    this.isTemporaryMoveSelecting = false;
    this.temporaryMovePointSelected = false;
  }

  @action setTemporaryMovePointSelected(selected) {
    this.temporaryMovePointSelected = selected;
  }

  @action setTemporaryMoveTaskType(taskType) {
    this.temporaryMoveTaskType = taskType;
    // If temporary move is already active but task type is unknown
    // (e.g. page refresh), use current selection to resolve it once.
    if (this.isTemporaryMoveActive && !this.activeTemporaryMoveTaskType) {
      this.activeTemporaryMoveTaskType = taskType;
    }
  }

  @action setTemporaryMoveActive(active) {
    this.isTemporaryMoveActive = active;
    if (!active) {
      this.activeTemporaryMoveTaskType = null;
    }
  }

  @action setActiveTemporaryMoveTaskType(taskType) {
    this.activeTemporaryMoveTaskType = taskType;
  }

  isTemporaryMoveMode() {
    return this.isTemporaryMoveSelecting || this.isTemporaryMoveActive;
  }

  sendTemporaryMoveRequest() {
    const success = RENDERER.sendTemporaryMoveRequest(
      this.temporaryMoveTaskType,
    );
    if (success) {
      this.setActiveTemporaryMoveTaskType(this.temporaryMoveTaskType);
      this.setTemporaryMoveActive(true);
    }
    return success;
  }

  endTemporaryMove() {
    if (this.isSearchReachStackerTemporaryMoveTask()) {
      return false;
    }
    const success = RENDERER.endTemporaryMoveRequest();
    if (success) {
      this.setTemporaryMoveActive(false);
    }
    return success;
  }

  exitTemporaryMoveMode() {
    const shouldEndTemporaryMove = this.isTemporaryMoveActive;
    if (shouldEndTemporaryMove && !this.endTemporaryMove()) {
      return false;
    }
    this.resetTemporaryMoveSelectingSession();
    return true;
  }

  clearTemporaryMovePoint() {
    RENDERER.clearTemporaryMovePoint();
    this.setTemporaryMovePointSelected(false);
  }

  resetTemporaryMoveSelectingSession() {
    this.clearTemporaryMovePoint();
    this.stopTemporaryMoveSelectingMode();
  }

  hasTemporaryMovePoint() {
    return this.temporaryMovePointSelected;
  }

  hideSimulationRoutingPoints() {
    RENDERER.hideSimulationRoutingPoints();
  }

  displaySimulationRoutingPoints() {
    RENDERER.displaySimulationRoutingPoints();
  }

  hideSimulationObstacles() {
    RENDERER.hideSimulationObstacles();
  }

  displaySimulationObstacles() {
    RENDERER.displaySimulationObstacles();
  }

  hideSimulationElements() {
    this.hideSimulationRoutingPoints();
    this.hideSimulationObstacles();
  }

  displaySimulationElements() {
    this.displaySimulationRoutingPoints();
    this.displaySimulationObstacles();
  }

  syncSimulationElementsVisibility(showSimulateEditingBar) {
    if (!showSimulateEditingBar || this.isTemporaryMoveMode()) {
      this.hideSimulationElements();
      return;
    }
    this.displaySimulationElements();
  }

  updateParkingRoutingDistance(data) {
    if (data.threshold) {
      this.parkingRoutingDistanceThreshold = data.threshold;
    }
  }

  isSearchReachStackerTemporaryMoveTask() {
    return (
      this.temporaryMoveTaskType ===
      TEMPORARY_MOVE_TASK_TYPE.SEARCH_REACH_STACKER
    );
  }

  isSearchReachStackerActiveTemporaryMoveTask() {
    return (
      this.activeTemporaryMoveTaskType ===
      TEMPORARY_MOVE_TASK_TYPE.SEARCH_REACH_STACKER
    );
  }

  downloadScenario() {
    if (this.routingPoints.length === 0 && this.obstaclesList.length === 0) {
      return alert('Please add at least one obstacle or one routing.');
    }

    const savedScenario = {
      ego: this.routingPoints,
      obstacles: this.obstaclesList
    };

    const elementA = document.createElement('a');

    elementA.setAttribute('href', 'data:text/plain;charset=utf-8,'
      + JSON.stringify(savedScenario));
    elementA.setAttribute('download', +new Date() + '.json');
    elementA.style.display = 'none';
    document.body.appendChild(elementA);
    elementA.click();
    document.body.removeChild(elementA);
  }

  readScenario() {
    const readElement = document.getElementById('files');

    readElement.onchange = this.changeFiles.bind(this);
    readElement.click();
  }

  changeFiles() {
    const selectedEle = document.getElementById('files');
    const selectedFile = selectedEle.files[0];

    const reader = new FileReader();
    reader.readAsText(selectedFile);

    reader.onload = e => {
      const json = JSON.parse(reader.result);
      if (Array.isArray(json.obstacles)) {
        this.updateScenarioObstaclesList(json.obstacles);
      } else {
        alert('Please select right obstacle file.');
      }
      if (Array.isArray(json.ego)) {
        this.updateScenarioRoutesList(json.ego);
      } else {
        alert('Please select right routing file.');
      }
      selectedEle.value = '';
    };
  }

  updateScenarioObstaclesList(obstacles) {
    this.removeAllObstacles();

    if (obstacles.length === 0) {
      return;
    }

    RENDERER.updateScenarioObstacles(obstacles);
  }

  updateScenarioRoutesList(routes) {
    this.removeAllRoutingPoints();
    if (routes.length === 0) {
      return;
    }
    RENDERER.updateScenarioRouting(routes);
  }

  isInSelectorMode() {
    return RENDERER.scenarioEditor.isInSelectorMode();
  }

  enableSelectorObject() {
    RENDERER.enableSelectorObject();
  }

  disableSelectorObject() {
    RENDERER.disableSelectorObject();
  }
}
