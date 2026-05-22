import 'imports-loader?THREE=three!three/examples/js/controls/OrbitControls.js';
import * as THREE from 'three';

import routingPointPin from 'assets/images/routing/pin.png';

import WS from 'store/websocket';
import STORE from 'store';
import {
  DEFAULT_UNKNOWN_SIZE, DEFAULT_COLOR, DEFAULT_HEIGHT, ARROW_SIZE, ObstacleColorMapping
} from 'renderer/obstacles.js';
import {
  drawImage, drawPointArrow, disposeMesh, drawBox, drawArrow, drawCircle
} from 'utils/draw';
import { IsPointInRectangle, copyProperty } from 'utils/misc';
import Text3D from 'renderer/text3d';

import _ from 'lodash';

export default class ScenarioEditor {
  constructor() {
    this.routePoints = [];
    this.temporaryMovePoint = null;
    this.parkingInfo = null;
    this.inEditingMode = false;
    this.pointId = 0;
    this.parkingSpaceInfo = [];
    this.deadJunctionInfo = [];
    this.arrows = [];
    this.obstacles = [];
    this.obstacleId = 0;
    this.textRender = new Text3D();
    this.obstacleTraceList = [];
    this.obstacleTraceId = 0;
    this.obstacleTriggerList = [];
    this.obstacleTriggerId = 0;

  }

  isInEditingMode() {
    return this.inEditingMode;
  }

  isInSelectorMode() {
    return this.inEditingMode && this.obstacles.length !== 0
      && !STORE.options.enableRoutingEditing
      && !STORE.options.enableObstacleEditing
      && !STORE.options.addObstaclePathPoint
      && !STORE.options.addObstacleTriggerPoint;
  }

  enableEditingMode(camera) {
    this.inEditingMode = true;

    const pov = 'Map';
    camera.fov = PARAMETERS.camera[pov].fov;
    camera.near = PARAMETERS.camera[pov].near;
    camera.far = PARAMETERS.camera[pov].far;

    camera.updateProjectionMatrix();
    WS.requestMapElementIdsByRadius(PARAMETERS.scenarioEditor.radiusOfMapRequest);
  }

  disableEditingMode() {
    this.inEditingMode = false;
    this.parkingInfo = null;
  }

  hiddenAllRoutingPoints() {
    this.routePoints.length !== 0 &&
      this.routePoints.forEach((object) => {
        object.visible = false;
        if (object.arrowMesh) {
          object.arrowMesh.visible = false;
        }
      });
  }

  displayAllRoutingPoints() {
    this.routePoints.length !== 0 &&
      this.routePoints.forEach((object) => {
        object.visible = true;
        if (object.arrowMesh) {
          object.arrowMesh.visible = true;
        }
      });
  }

  drawPointArrowWithHeading(heading, origin, scene, hex = 0xff0000) {
    const arrowMesh = drawPointArrow(origin, hex, heading, 3);
    arrowMesh.heading = heading;
    this.arrows.push(arrowMesh);
    scene.add(arrowMesh);
  }

  drawPointArrow(currTarget, origin, coordinates, scene, notFirst, hex = 0xff0000) {
    // remove arrows generated during drag and drop
    if (notFirst) {
      const lastArrow = this.arrows.pop();
      disposeMesh(lastArrow);
      scene.remove(lastArrow);
    }
    const offsetOrigin = coordinates.applyOffset(origin);
    const offsetTarget = coordinates.applyOffset(currTarget);
    const heading = Math.atan2(
      offsetTarget.y - offsetOrigin.y, offsetTarget.x - offsetOrigin.x,
    );
    this.drawPointArrowWithHeading(heading, offsetOrigin, scene, hex);
  }

  addObstaclePoint(point, coordinates, scene) {
    const lastArrow = this.arrows.pop();
    const heading = lastArrow ? lastArrow.heading : 0;
    disposeMesh(lastArrow);
    scene.remove(lastArrow);

    const offsetPoint = coordinates.applyOffset({ x: point.x, y: point.y });
    const selectedParkingSpaceIndex = this.isPointInParkingSpace(offsetPoint);
    if (selectedParkingSpaceIndex !== -1) {
      this.parkingSpaceInfo[selectedParkingSpaceIndex].selectedCounts++;
    }

    const cubeSize = new THREE.Vector3(
      DEFAULT_UNKNOWN_SIZE.X, DEFAULT_UNKNOWN_SIZE.Y, DEFAULT_UNKNOWN_SIZE.Z);

    const obstacleMesh = drawBox(cubeSize, DEFAULT_COLOR, DEFAULT_HEIGHT);
    obstacleMesh.obstacleId = this.obstacleId;
    point.id = this.obstacleId;
    this.obstacleId += 1;

    obstacleMesh.position.set(
      offsetPoint.x, offsetPoint.y, DEFAULT_UNKNOWN_SIZE.Z / 2);
    obstacleMesh.rotation.set(0, 0, -(Math.PI / 2 - heading));

    const arrowMesh = drawArrow(ARROW_SIZE.LENGTH, ARROW_SIZE.LINEWIDTH,
      ARROW_SIZE.CONELENGTH, ARROW_SIZE.CONEWIDTH, DEFAULT_COLOR);
    arrowMesh.rotation.set(0, 0, -(Math.PI / 2 - heading));
    point.heading = heading;
    copyProperty(arrowMesh.position, obstacleMesh.position);

    const text = this.textRender.drawChar3D(obstacleMesh.obstacleId);
    copyProperty(text.charMesh.position, obstacleMesh.position);

    const group = new THREE.Group();
    group.add(obstacleMesh);
    group.add(arrowMesh);
    group.add(text.charMesh);
    group.name = 'obstacleGroup';
    group.groupId = obstacleMesh.obstacleId;

    this.obstacles.push(group);
    scene.add(group);
    const obstacle = {
      id: point.id,
      position: [point.x, point.y, point.z],
      heading: point.heading,
      length: DEFAULT_UNKNOWN_SIZE.Y,
      width: DEFAULT_UNKNOWN_SIZE.X,
      height: DEFAULT_UNKNOWN_SIZE.Z,
      type: 'UNKNOWN',
      move_way: 'static',
      speed: 0,
      trigger_position: [],
      trigger_radius: 1,
      trace: [],
      trigger_position: [],
      trigger_radius: 1,
      trace: [],
    };
    STORE.scenarioEditingManager.addObstacle(obstacle);
    return selectedParkingSpaceIndex;
  }

  hiddenAllObstacles() {
    this.obstacles.forEach((object) => {
      object.visible = false;
    });
    this.obstacleTraceList.forEach((object) => {
      object.visible = false;
    });
    this.obstacleTriggerList.forEach((object) => {
      object.visible = false;
    });
  }

  displayAllObstacles() {
    this.obstacles.forEach((object) => {
      object.visible = true;
    });
    this.obstacleTraceList.forEach((object) => {
      object.visible = true;
    });
    this.obstacleTriggerList.forEach((object) => {
      object.visible = true;
    });
  }

  updateObstacles(obstacle,coordinates,scene) {
    const offsetPoint =
      coordinates.applyOffset({ x: obstacle.position[0], y: obstacle.position[1] });
    const selectedParkingSpaceIndex = this.isPointInParkingSpace(offsetPoint);
    if (selectedParkingSpaceIndex !== -1) {
      this.parkingSpaceInfo[selectedParkingSpaceIndex].selectedCounts++;
    }
    const cubeSize =
      new THREE.Vector3(DEFAULT_UNKNOWN_SIZE.X, DEFAULT_UNKNOWN_SIZE.Y, DEFAULT_UNKNOWN_SIZE.Z);
    const multipleX = obstacle.width / DEFAULT_UNKNOWN_SIZE.X;
    const multipleY = obstacle.length / DEFAULT_UNKNOWN_SIZE.Y;
    const multipleZ = obstacle.height / DEFAULT_UNKNOWN_SIZE.Z;
    let color = DEFAULT_COLOR;
    for (const key in ObstacleColorMapping) {
      if (key === obstacle.type) {
        color = ObstacleColorMapping[key];
      }
    }

    const obstacleMesh = drawBox(cubeSize, color, DEFAULT_HEIGHT);
    obstacleMesh.obstacleId = obstacle.id;
    this.obstacleId = obstacle.id + 1;
    obstacleMesh.position.set(offsetPoint.x, offsetPoint.y, DEFAULT_UNKNOWN_SIZE.Z / 2);
    obstacleMesh.scale.set(multipleX,multipleY,multipleZ);
    obstacleMesh.position.setZ(multipleZ / 2);
    obstacleMesh.rotation.set(0, 0, obstacle.heading + (-Math.PI / 2));

    const arrowMesh = drawArrow(ARROW_SIZE.LENGTH, ARROW_SIZE.LINEWIDTH,
      ARROW_SIZE.CONELENGTH, ARROW_SIZE.CONEWIDTH, color);
    arrowMesh.rotation.set(0, 0, obstacle.heading + (-Math.PI / 2));
    arrowMesh.scale.set(multipleX,multipleY,multipleZ);
    copyProperty(arrowMesh.position, obstacleMesh.position);

    const text = this.textRender.drawChar3D(obstacleMesh.obstacleId);
    copyProperty(text.charMesh.position, obstacleMesh.position);

    const group = new THREE.Group();
    group.add(obstacleMesh);
    group.add(arrowMesh);
    group.add(text.charMesh);
    group.name = 'obstacleGroup';
    group.groupId = obstacleMesh.obstacleId;

    this.obstacles.push(group);
    scene.add(group);
    const updateObstacle = {
      id: obstacle.id,
      position: [obstacle.position[0], obstacle.position[1], obstacle.position[2]],
      heading: obstacle.heading,
      length: obstacle.length,
      width: obstacle.width,
      height: obstacle.height,
      type: obstacle.type,
      move_way: obstacle.move_way,
      speed: obstacle.speed,
      trigger_position: [],
      trigger_radius: obstacle.trigger_radius,
      trace: [],
    };
    STORE.scenarioEditingManager.addObstacle(updateObstacle);

    obstacle.trace.forEach((ele) => {
      const position = {
        x: ele.position[0],
        y: ele.position[1]
      };
      this.addObstaclePathPointTrace(scene,position,coordinates,color);
    });

    obstacle.trigger_position.forEach((ele) => {
      const position = {
        x: ele.position[0],
        y: ele.position[1]
      };
      const circle = this.addObstacleTriggerPoint(scene,position,coordinates,color);
      circle.scale.set(obstacle.trigger_radius,obstacle.trigger_radius,obstacle.trigger_radius);
    });

    if (STORE.scenarioEditingManager.isInSelectorMode()) {
      STORE.scenarioEditingManager.enableSelectorObject();
    };

    return selectedParkingSpaceIndex;
  }

  removeAllObstacles(scene) {
    let index = -1;
    const indexArr = [];
    this.obstacles.forEach((object) => {
      index = this.removeObstaclePoint(scene, object);
      if (index !== -1) {
        indexArr.push(index);
      }
    });
    this.obstacleTraceList.forEach((object) => {
      index = this.removeObstacleTracePoint(scene, object);
      if (index !== -1) {
        indexArr.push(index);
      }
    });
    this.obstacleTriggerList.forEach((object) => {
      index = this.removeObstacleTriggerPoint(scene, object);
      if (index !== -1) {
        indexArr.push(index);
      }
    });
    this.obstacles = [];
    this.obstacleTraceList = [];
    this.obstacleTriggerList = [];
    if (!STORE.scenarioEditingManager.isInSelectorMode()) {
      STORE.scenarioEditingManager.disableSelectorObject();
    };
    return indexArr;
  }

  removeOneObstacle(obj,scene) {
    const lastPoint = this.obstacles.find(item => item.groupId === obj.id);
    let index = -1;
    if (lastPoint) {
      index = this.removeObstaclePoint(scene, lastPoint);
    }
    this.obstacles = this.obstacles.filter(item => item.groupId !== obj.id);

    obj.trace.forEach((object) => {
      const pathPoint = this.obstacleTraceList.find(item => item.obstacleTraceId === object.id);
      if (pathPoint) {
        index = this.removeObstacleTracePoint(scene, pathPoint);
        if (index !== -1) {
          indexArr.push(index);
        }
      }
      this.obstacleTraceList =
        this.obstacleTraceList.filter(item => item.obstacleTraceId !== object.id);
    });

    obj.trigger_position.forEach((object) => {
      const triggerPoint =
        this.obstacleTriggerList.find(item => item.obstacleTriggerId === object.id);
      if (triggerPoint) {
        index = this.removeObstacleTriggerPoint(scene, triggerPoint);
        if (index !== -1) {
          indexArr.push(index);
        }
      }
      this.obstacleTriggerList =
        this.obstacleTriggerList.filter(item => item.obstacleTriggerId !== object.id);
    });
    return index;
  }

  removeObstaclePoint(scene, object) {
    scene.remove(object);
    let index = this.isPointInParkingSpace(_.get(object, 'position'));
    if (index !== -1) {
      if (--this.obstacles[index].selectedCounts > 0) {
        index = -1;
      }
    }
    if (object.geometry) {
      object.geometry.dispose();
    }
    if (object.material) {
      object.material.dispose();
    }
    return index;
  }

  addObstaclePathPointTrace(scene, pathPoint, coordinates, color = 0x00FA9A) {
    const curTraceId = STORE.scenarioEditingManager.curObstacleId;
    const offsetPoint = coordinates.applyOffset(
      { x: pathPoint.x, y: pathPoint.y });
    const material = new THREE.MeshBasicMaterial({
      color: color,
      transparent: true,
      opacity: 0.5,
    });
    const pointMesh = drawCircle(0.5, material);
    pointMesh.position.set(offsetPoint.x, offsetPoint.y, 0.24);
    pointMesh.obstacleTraceId = curTraceId + '-' + this.obstacleTraceId;
    pathPoint.id = this.obstacleTraceId;
    this.obstacleTraceId += 1;
    pointMesh.name = 'trace';
    this.obstacleTraceList.push(pointMesh);
    scene.add(pointMesh);
    STORE.scenarioEditingManager.addObstacleTrace(pathPoint);
    return pointMesh;
  }

  removeAllObstacleTracePoints(scene) {
    let index = -1;
    const indexArr = [];
    this.obstacleTraceList.forEach((item) => {
      index = this.removeObstacleTracePoint(scene, item);
      if (index !== -1) {
        indexArr.push(index);
      }
    });
    this.obstacleTraceList = [];
    return indexArr;
  }

  removeOneObstacleTracePoint(scene,id) {
    const curPoint = this.obstacleTraceList.find(item => item.obstacleTraceId === id);
    let index = -1;
    if (curPoint) {
      index = this.removeObstacleTracePoint(scene, curPoint);
    }
    this.obstacleTraceList = this.obstacleTraceList.filter(item => item.obstacleTraceId !== id);
    return index;
  }

  removeObstacleTracePoint(scene, object) {
    scene.remove(object);
    let index = this.isPointInParkingSpace(_.get(object, 'position'));
    if (index !== -1) {
      if (--this.obstacleTraceList[index].selectedCounts > 0) {
        index = -1;
      }
    }
    if (object.geometry) {
      object.geometry.dispose();
    }
    if (object.material) {
      object.material.dispose();
    }
    return index;
  }

  addObstacleTriggerPoint(scene, pathPoint, coordinates, color = 0x00FA9A) {
    const curTriggerId = STORE.scenarioEditingManager.curObstacleId;
    const offsetPoint = coordinates.applyOffset(
      { x: pathPoint.x, y: pathPoint.y });
    const material = new THREE.MeshBasicMaterial({
      color: color,
      transparent: true,
      opacity: 0.2,
    });
    const pointMesh = drawCircle(1, material);
    pointMesh.position.set(offsetPoint.x, offsetPoint.y, 0.24);
    pointMesh.obstacleTriggerId = curTriggerId  + '-' + this.obstacleTriggerId;
    pathPoint.id = this.obstacleTriggerId;
    this.obstacleTriggerId += 1;
    pointMesh.name = 'trigger';
    this.obstacleTriggerList.push(pointMesh);
    scene.add(pointMesh);
    STORE.scenarioEditingManager.addObstacleTrigger(pathPoint);
    return pointMesh;
  }

  removeOneObstacleTriggerPoint(scene,id) {
    const curPoint = this.obstacleTriggerList.find(item => item.obstacleTriggerId === id);
    let index = -1;
    if (curPoint) {
      index = this.removeObstacleTriggerPoint(scene, curPoint);
    }
    this.obstacleTriggerList =
      this.obstacleTriggerList.filter(item => item.obstacleTriggerId !== id);
    return index;
  }

  removeObstacleTriggerPoint(scene, object) {
    scene.remove(object);
    let index = this.isPointInParkingSpace(_.get(object, 'position'));
    if (index !== -1) {
      if (--this.obstacleTriggerList[index].selectedCounts > 0) {
        index = -1;
      }
    }
    if (object.geometry) {
      object.geometry.dispose();
    }
    if (object.material) {
      object.material.dispose();
    }
    return index;
  }

  addRoutingPoint(point, coordinates, scene) {
    const offsetPoint = coordinates.applyOffset({ x: point.x, y: point.y });
    const selectedParkingSpaceIndex = this.isPointInParkingSpace(offsetPoint);
    if (selectedParkingSpaceIndex !== -1) {
      this.parkingSpaceInfo[selectedParkingSpaceIndex].selectedCounts++;
    }
    const pointMesh = drawImage(routingPointPin, 3.5, 3.5, offsetPoint.x, offsetPoint.y, 0.3);
    pointMesh.pointId = this.pointId;
    point.id = this.pointId;
    this.pointId += 1;
    pointMesh.arrowMesh = this.arrows.pop();
    if (pointMesh.arrowMesh) {
      point.heading = pointMesh.arrowMesh.heading;
    }
    this.routePoints.push(pointMesh);
    scene.add(pointMesh);
    WS.checkRoutingPoint(point);
    STORE.scenarioEditingManager.addPoint(point);
    return selectedParkingSpaceIndex;
  }

  addTemporaryMovePoint(point, coordinates, scene) {
    this.clearTemporaryMovePoint(scene);
    const offsetPoint = coordinates.applyOffset({ x: point.x, y: point.y });
    const pointMesh = drawImage(routingPointPin, 3.5, 3.5, offsetPoint.x, offsetPoint.y, 0.3);
    pointMesh.arrowMesh = this.arrows.pop();
    this.temporaryMovePoint = pointMesh;
    scene.add(pointMesh);
    STORE.scenarioEditingManager.setTemporaryMovePointSelected(true);
    return true;
  }

  clearTemporaryMovePoint(scene) {
    if (!this.temporaryMovePoint) {
      STORE.scenarioEditingManager.setTemporaryMovePointSelected(false);
      return;
    }
    scene.remove(this.temporaryMovePoint);
    if (this.temporaryMovePoint.arrowMesh) {
      scene.remove(this.temporaryMovePoint.arrowMesh);
      disposeMesh(this.temporaryMovePoint.arrowMesh);
    }
    if (this.temporaryMovePoint.geometry) {
      this.temporaryMovePoint.geometry.dispose();
    }
    if (this.temporaryMovePoint.material) {
      this.temporaryMovePoint.material.dispose();
    }
    this.temporaryMovePoint = null;
    STORE.scenarioEditingManager.setTemporaryMovePointSelected(false);
  }

  hasTemporaryMovePoint() {
    return !!this.temporaryMovePoint;
  }

  setParkingSpaceInfo(parkingSpaceInfo, extraInformation, coordinates, scene) {
    this.parkingSpaceInfo = parkingSpaceInfo;
    this.parkingSpaceInfo.forEach((item, index) => {
      const extraInfo = extraInformation[index];
      if (_.isEmpty(extraInfo)) {
        return false;
      }
      const offsetPoints = item.polygon.point.map(point =>
        coordinates.applyOffset({ x: point.x, y: point.y })
      );
      const adjustPoints = extraInfo.order.map(number => {
        return offsetPoints[number];
      });
      item.polygon.point = adjustPoints;
      item.type = extraInfo.type;
      item.routingEndPoint = extraInfo.routingEndPoint;
      item.laneWidth = extraInfo.laneWidth;
      item.selectedCounts = 0;
      item.laneId = extraInfo.laneId;
    });
  }

  removeInvalidRoutingPoint(pointId, msg, scene) {
    let index = -1;
    alert(msg);
    if (pointId) {
      this.routePoints = this.routePoints.filter((point) => {
        if (point.pointId === pointId) {
          index = this.removeRoutingPoint(scene, point);
          return false;
        }
        return true;
      });
    }
    return index;
  }

  removeAllRoutePoints(scene) {
    let index = -1;
    const indexArr = [];
    this.routePoints.forEach((object) => {
      index = this.removeRoutingPoint(scene, object);
      if (index !== -1) {
        indexArr.push(index);
      }
    });
    this.routePoints = [];
    this.arrows = [];
    return indexArr;
  }

  removeOneRoutingPoint(id,scene) {
    const rmPoint = this.routePoints.find(item => item.pointId === id);
    this.routePoints = this.routePoints.filter(item => item.pointId !== id);
    let index = -1;
    if (rmPoint) {
      index = this.removeRoutingPoint(scene, rmPoint);
    }
    return index;
  }

  removeRoutingPoint(scene, object) {
    scene.remove(object);
    let index = this.isPointInParkingSpace(_.get(object, 'position'));
    if (index !== -1) {
      if (--this.parkingSpaceInfo[index].selectedCounts > 0) {
        index = -1;
      }
    }
    if (object.arrowMesh) {
      scene.remove(object.arrowMesh);
      disposeMesh(object.arrowMesh);
    }
    if (object.geometry) {
      object.geometry.dispose();
    }
    if (object.material) {
      object.material.dispose();
    }
    return index;
  }

  sendRoutingRequest(carOffsetPosition, carHeading, coordinates, routingPoints, isLoopRunning) {
    // parking routing request vs common routing request
    // add dead end junction routing request when select three points
    // and the second point is in dead end junction.
    if (this.routePoints.length === 0 && routingPoints.length === 0) {
      alert('Please provide at least an end point.');
      return false;
    }

    const index = _.isEmpty(this.routePoints) ?
      -1 : this.isPointInParkingSpace(this.routePoints[this.routePoints.length - 1].position);
    const parkingRoutingRequest = (index !== -1);
    if (parkingRoutingRequest) {
      const lastPoint = this.routePoints.pop();
      const { routingEndPoint, id, type, laneWidth, laneId } = this.parkingSpaceInfo[index];
      const parkingRequestPoints = this.routePoints.map((object) => {
        object.position.z = 0;
        const point = {
          ...coordinates.applyOffset(object.position, true)
        };
        if (object.arrowMesh) {
          point.heading = object.arrowMesh.heading;
        }
        return point;
      });
      // parkingRequestPoints.push(coordinates.applyOffset(routingEndPoint, true));
      const start = (parkingRequestPoints.length > 0) ? parkingRequestPoints[0]
        : coordinates.applyOffset(carOffsetPosition, true);
      const end = coordinates.applyOffset(lastPoint.position, true);
      if (lastPoint.arrowMesh) {
        end.heading = lastPoint.arrowMesh.heading;
      };
      const waypoint = (parkingRequestPoints.length > 1) ? parkingRequestPoints.slice(1) : [];
      this.routePoints.push(lastPoint);
      this.parkingSpaceInfo[index].polygon.point.forEach(vertex => {
        vertex.z = 0;
        parkingRequestPoints.push(coordinates.applyOffset(vertex, true));
      });
      const cornerPoints = parkingRequestPoints.slice(-4);
      const parkingInfo = {
        parkingSpaceId: _.get(id, 'id'),
        parkingPoint: coordinates.applyOffset(lastPoint.position, true),
        parkingSpaceType: type,
      };
      WS.sendParkingRequest(
        start, waypoint, end, parkingInfo, laneWidth, cornerPoints, laneId, false
      );
      return true;
    }

    const indexStart = _.isEmpty(this.routePoints) ?
      -1 : this.isPointInParkingSpace(this.routePoints[0].position);
    const parkingRoutingRequestStart = (indexStart !== -1);
    if (parkingRoutingRequestStart) {
      const firstPoint = this.routePoints.shift();
      const { routingEndPoint, id, type, laneWidth, laneId } = this.parkingSpaceInfo[indexStart];
      const parkingRequestPoints = this.routePoints.map((object) => {
        object.position.z = 0;
        // return coordinates.applyOffset(object.position, true);
        const point = {
          ...coordinates.applyOffset(object.position, true)
        };
        if (object.arrowMesh) {
          point.heading = object.arrowMesh.heading;
        }
        return point;
      });
      // parkingRequestPoints.unshift(coordinates.applyOffset(routingEndPoint, true));
      const start = coordinates.applyOffset(firstPoint.position, true);
      if (firstPoint.arrowMesh) {
        start.heading = firstPoint.arrowMesh.heading;
      };
      const end = parkingRequestPoints[parkingRequestPoints.length - 1];
      const waypoint = (parkingRequestPoints.length > 1) ? parkingRequestPoints.slice(0, -1) : [];
      this.routePoints.unshift(firstPoint);
      this.parkingSpaceInfo[indexStart].polygon.point.forEach(vertex => {
        vertex.z = 0;
        parkingRequestPoints.push(coordinates.applyOffset(vertex, true));
      });
      const cornerPoints = parkingRequestPoints.slice(-4);
      const parkingInfo = {
        parkingSpaceId: _.get(id, 'id'),
        parkingPoint: coordinates.applyOffset(firstPoint.position, true),
        parkingSpaceType: type,
      };
      WS.sendParkingRequest(
        start, waypoint, end, parkingInfo, laneWidth, cornerPoints, laneId, true
      );
      return true;
    }

    const points = _.isEmpty(routingPoints) ?
      this.routePoints.map((object) => {
        object.position.z = 0;
        // return coordinates.applyOffset(object.position, true);
        const point = {
          ...coordinates.applyOffset(object.position, true)
        };
        if (object.arrowMesh) {
          point.heading = object.arrowMesh.heading;
        }
        return point;
      }) : routingPoints.map((point) => {
        point.z = 0;
        return _.pick(point, ['x', 'y', 'z']);
      });
    if (points.length === 3 && !_.isEmpty(this.deadJunctionInfo)) {
      const deadJunctionIdx = this.determinDeadEndJunctionRequest(points);
      if (deadJunctionIdx !== -1) {
        const deadJunction = this.deadJunctionInfo[deadJunctionIdx];
        WS.sendDeadEndJunctionRoutingRequest(
          points[0], deadJunction.inEndPoint, deadJunction.outStartPoint,
          points[2], deadJunction.inLaneIds, deadJunction.outLaneIds,
          [deadJunction.routingPoint],
        );
        return true;
      }
    }

    let start = null;
    let start_heading = null;
    let waypoint = null;
    if (isLoopRunning) {
      start = coordinates.applyOffset(carOffsetPosition, true);
      start_heading = carHeading;
      waypoint = (points.length > 1) ? points.slice(0, -1) : [];
    } else {
      start = (points.length > 1) ? points[0]
        : coordinates.applyOffset(carOffsetPosition, true);
      start_heading = (points.length > 1) ? null : carHeading;
      waypoint = (points.length > 1) ? points.slice(1, -1) : [];
    }
    const end = points[points.length - 1];
    WS.requestRoute(start, start_heading, waypoint, end, this.parkingInfo, isLoopRunning);
    return true;
  }

  sendTemporaryMoveRequest(carOffsetPosition, carHeading, coordinates, taskType) {
    if (!this.temporaryMovePoint) {
      alert('Please provide a temporary move point.');
      return false;
    }

    const start = coordinates.applyOffset(carOffsetPosition, true);
    start.heading = carHeading;

    const end = coordinates.applyOffset(this.temporaryMovePoint.position, true);
    if (this.temporaryMovePoint.arrowMesh) {
      end.heading = this.temporaryMovePoint.arrowMesh.heading;
    }

    return WS.startTemporaryMove(start, end, taskType);
  }

  endTemporaryMoveRequest(carOffsetPosition, carHeading, coordinates) {
    const start = coordinates.applyOffset(carOffsetPosition, true);
    start.heading = carHeading;
    return WS.endTemporaryMove(start);
  }

  sendDemonstrateRequest(carOffsetPosition, carHeading, coordinates, end, type, blacklistedLane) {
    const start = coordinates.applyOffset(carOffsetPosition, true);
    const start_heading = carHeading;
    WS.sendDemonstrateRequest(start, start_heading, end, type, blacklistedLane);
  }

  saveCurCarPosition(name, position) {
    WS.saveCurCarPosition(name, position);
  }

  startLoopRunning(position,heading,coordinates) {
    const start = coordinates.applyOffset(position, true);
    // const end = {x:-2708.450657666,y:2111.6562965,z:10.924381586};
    WS.requestRoute(start, heading, [], start, this.parkingInfo, 'isOneClickLoopRunning');
  }

  isPointInParkingSpace(offsetPoint) {
    let index = -1;
    if (!_.isEmpty(this.parkingSpaceInfo) && !_.isEmpty(offsetPoint)) {
      index = _.findIndex(this.parkingSpaceInfo, item =>
        IsPointInRectangle(item.polygon.point, offsetPoint));
    }
    return index;
  }

  determinDeadEndJunctionRequest(offsetPoints) {
    const index = _.findIndex(this.deadJunctionInfo, deadJunction =>
      IsPointInRectangle(deadJunction.deadJunctionPoints, offsetPoints[1]));
    return index;
  }

  setDeadJunctionInfo(deadJunctionInfos) {
    this.deadJunctionInfo = deadJunctionInfos;
  }
}
