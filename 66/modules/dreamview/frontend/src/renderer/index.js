import * as THREE from 'three';
import Stats from 'stats.js';
import STORE from 'store';
import { ObstacleColorMapping } from 'renderer/obstacles.js';

import Styles from 'styles/main.scss';

import Coordinates from 'renderer/coordinates';
import AutoDrivingCar from 'renderer/adc';
import CheckPoints from 'renderer/check_points.js';
import Ground from 'renderer/ground';
import TileGround from 'renderer/tileground';
import Map from 'renderer/map';
import PlanningTrajectory from 'renderer/trajectory.js';
import PlanningStatus from 'renderer/status.js';
import PerceptionObstacles from 'renderer/obstacles.js';
import Decision from 'renderer/decision.js';
import Prediction from 'renderer/prediction.js';
import Routing from 'renderer/routing.js';
import ScenarioEditor from 'renderer/scenario_editor.js';
import Gnss from 'renderer/gnss.js';
import PointCloud from 'renderer/point_cloud.js';
import {
  drawCircle,
  disposeMesh,
  drawSegmentsFromPoints
} from 'utils/draw';
import Text3D from 'renderer/text3d';

const _ = require('lodash');

const preventEditTag = ['BUTTON', 'INPUT'];

class Renderer {
  constructor() {
    this.textRender = new Text3D();

    // Disable antialias for mobile devices.
    const useAntialias = !this.isMobileDevice();

    this.coordinates = new Coordinates();
    this.renderer = new THREE.WebGLRenderer({
      antialias: useAntialias,
      // Transparent background
      alpha: true,
    });
    this.scene = new THREE.Scene();
    if (OFFLINE_PLAYBACK) {
      this.scene.background = new THREE.Color(0x000C17);
    }

    // The dimension of the scene
    this.dimension = {
      width: 0,
      height: 0,
    };

    // The ground.
    this.ground = (PARAMETERS.ground.type === 'tile' || OFFLINE_PLAYBACK)
      ? new TileGround(this.renderer) : new Ground();

    // The map.
    this.map = new Map();

    // The main autonomous driving car.
    this.adc = new AutoDrivingCar('adc', this.scene);

    // The car that projects the starting point of the planning trajectory
    this.planningAdc = OFFLINE_PLAYBACK ? null : new AutoDrivingCar('planningAdc', this.scene);

    // The shadow localization
    this.shadowAdc = new AutoDrivingCar('shadowAdc', this.scene);

    // The planning trajectory.
    this.planningTrajectory = new PlanningTrajectory();

    // The planning status
    this.planningStatus = new PlanningStatus();

    // The perception obstacles.
    this.perceptionObstacles = new PerceptionObstacles();

    // The decision.
    this.decision = new Decision();

    // The prediction.
    this.prediction = new Prediction();

    // The routing.
    this.routing = new Routing();

    // The route editor
    this.scenarioEditor = new ScenarioEditor();

    // Distinguish between drawing point and drawing arrow
    this.routingPoint = null;
    this.startRoutingHeading = false;
    this.obstaclePoint = null;
    this.startObstacleHeading = false;

    this.startDragObject = false;
    this.curSelectObj = null;

    // The selected points distance
    this.firstPointsDistance = null;
    this.secondPointsDistance = null;

    this.firstPointsDistanceMesh = null;
    this.secondPointsDistanceMesh = null;

    this.selectedPointsDistances = [];
    this.pointsLineMesh = null;
    this.pointsLineTextMesh = null;

    // The GNSS/GPS
    this.gnss = new Gnss();

    this.pointCloud = new PointCloud();

    this.checkPoints = OFFLINE_PLAYBACK && new CheckPoints(this.coordinates, this.scene);

    // The Performance Monitor
    this.stats = null;
    if (PARAMETERS.debug.performanceMonitor) {
      this.stats = new Stats();
      this.stats.showPanel(1);
      this.stats.domElement.style.position = 'absolute';
      this.stats.domElement.style.top = null;
      this.stats.domElement.style.bottom = '0px';
      document.body.appendChild(this.stats.domElement);
    }

    // Geolocation of the mouse
    this.geolocation = { x: 0, y: 0 };
  }

  initialize(canvasId, width, height, options, cameraData) {
    this.options = options;
    this.cameraData = cameraData;
    this.canvasId = canvasId;

    // Camera
    this.viewAngle = PARAMETERS.camera.viewAngle;
    this.viewDistance = (
      PARAMETERS.camera.laneWidth
            * PARAMETERS.camera.laneWidthToViewDistanceRatio);
    this.camera = new THREE.PerspectiveCamera(
      PARAMETERS.camera[this.options.cameraAngle].fov,
      width / height,
      PARAMETERS.camera[this.options.cameraAngle].near,
      PARAMETERS.camera[this.options.cameraAngle].far,
    );
    this.camera.name = 'camera';
    this.scene.add(this.camera);

    this.updateDimension(width, height);
    this.renderer.setPixelRatio(window.devicePixelRatio);

    const container = document.getElementById(canvasId);
    container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x444444);
    const directionalLight = new THREE.DirectionalLight(0xffeedd);
    directionalLight.position.set(0, 0, 1).normalize();

    // The orbit axis of the OrbitControl depends on camera's up vector
    // and can only be set during creation of the controls. Thus,
    // setting camera up here. Note: it's okay if the camera.up doesn't
    // match the point of view setting, the value will be adjusted during
    // each update cycle.
    this.camera.up.set(0, 0, 1);

    // Orbit control for moving map
    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enable = false;

    // handler for editing with mouse down events
    // Routing handler
    this.onMouseDownRouteHandler = this.startRouteHandler.bind(this);
    this.onMouseMoveRouteHandler = this.moveRouteHandler.bind(this);
    this.onMouseUpRouteHandler = this.endRouteHandler.bind(this);
    // Obstacle handler
    this.onMouseDownObstacleHandler = this.startObstacleHandler.bind(this);
    this.onMouseMoveObstacleHandler = this.moveObstacleHandler.bind(this);
    this.onMouseUpObstacleHandler = this.endObstacleHandler.bind(this);
    this.onMouseAddObstaclePointPath = this.addObstaclePathPoints.bind(this);
    this.onMouseAddObstacleTriggerPoint = this.addObstacleTriggerPoints.bind(this);
    // Object Select handler
    this.onMouseDownSelectObject = this.onMouseDownSelectObject.bind(this);
    this.onMouseMoveSelectObject = this.onMouseMoveSelectObject.bind(this);
    this.onMouseUpSelectObject = this.onMouseUpSelectObject.bind(this);
    // Measuring Distance handler
    this.onMouseDownDistance = this.onMouseDownDistance.bind(this);
    this.onMouseMoveDistance = this.onMouseMoveDistance.bind(this);
    this.onMouseUpDistance = this.onMouseUpDistance.bind(this);

    this.scene.add(ambient);
    this.scene.add(directionalLight);

    // TODO maybe add sanity check.

    // Actually start the animation.
    this.animate();
  }

  maybeInitializeOffest(x, y, forced_update = false) {
    if (!this.coordinates.isInitialized() || forced_update) {
      this.coordinates.initialize(x, y);
    }
  }

  updateDimension(width, height) {
    if (width < Styles.MIN_MAIN_VIEW_WIDTH / 2 && this.dimension.width >= width) {
      // Reach minimum, do not update camera/renderer dimension anymore.
      return;
    }

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);

    this.dimension.width = width;
    this.dimension.height = height;
  }

  updateCurSorStyle(isChange) {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.style.cursor = isChange ? 'crosshair' : 'default';
    }
  }

  enableOrbitControls(enableRotate) {
    // update camera
    const carPosition = this.adc.mesh.position;
    this.camera.position.set(carPosition.x, carPosition.y, 50);
    if (this.coordinates.systemName === 'FLU') {
      this.camera.up.set(1, 0, 0);
    } else {
      this.camera.up.set(0, 0, 1);
    }
    const lookAtPosition = new THREE.Vector3(carPosition.x, carPosition.y, 0);
    this.camera.lookAt(lookAtPosition);

    // update control reset values to match current camera's
    this.controls.target0 = lookAtPosition.clone();
    this.controls.position0 = this.camera.position.clone();
    this.controls.zoom0 = this.camera.zoom;

    // set distance control
    this.controls.minDistance = 4;
    this.controls.maxDistance = 4000;

    // set vertical angle control
    this.controls.minPolarAngle = 0;
    this.controls.maxPolarAngle = Math.PI / 2;

    this.controls.enabled = true;
    this.controls.enableRotate = enableRotate;
    this.controls.reset();
  }

  adjustCamera(target, pov) {
    if (this.scenarioEditor.isInEditingMode()) {
      return;
    }

    this.camera.fov = PARAMETERS.camera[pov].fov;
    this.camera.near = PARAMETERS.camera[pov].near;
    this.camera.far = PARAMETERS.camera[pov].far;

    switch (pov) {
      case 'Default':
        let deltaX = (this.viewDistance * Math.cos(target.rotation.y)
                * Math.cos(this.viewAngle));
        let deltaY = (this.viewDistance * Math.sin(target.rotation.y)
                * Math.cos(this.viewAngle));
        let deltaZ = this.viewDistance * Math.sin(this.viewAngle);

        this.camera.position.x = target.position.x - deltaX;
        this.camera.position.y = target.position.y - deltaY;
        this.camera.position.z = target.position.z + deltaZ;
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt({
          x: target.position.x + deltaX,
          y: target.position.y + deltaY,
          z: 0,
        });

        this.controls.enabled = false;
        break;
      case 'Near':
        deltaX = (this.viewDistance * 0.5 * Math.cos(target.rotation.y)
                    * Math.cos(this.viewAngle));
        deltaY = (this.viewDistance * 0.5 * Math.sin(target.rotation.y)
                    * Math.cos(this.viewAngle));
        deltaZ = this.viewDistance * 0.5 * Math.sin(this.viewAngle);

        this.camera.position.x = target.position.x - deltaX;
        this.camera.position.y = target.position.y - deltaY;
        this.camera.position.z = target.position.z + deltaZ;
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt({
          x: target.position.x + deltaX,
          y: target.position.y + deltaY,
          z: 0,
        });

        this.controls.enabled = false;
        break;
      case 'Overhead':
        deltaY = (this.viewDistance * 0.5 * Math.sin(target.rotation.y)
                    * Math.cos(this.viewAngle));
        deltaZ = this.viewDistance * 2 * Math.sin(this.viewAngle);

        this.camera.position.x = target.position.x;
        this.camera.position.y = target.position.y + deltaY;
        this.camera.position.z = (target.position.z + deltaZ) * 2;
        if (this.coordinates.systemName === 'FLU') {
          this.camera.up.set(1, 0, 0);
        } else {
          this.camera.up.set(0, 1, 0);
        }
        this.camera.lookAt({
          x: target.position.x,
          y: target.position.y + deltaY,
          z: 0,
        });

        this.controls.enabled = false;
        break;
      case 'Map':
        if (!this.controls.enabled) {
          this.enableOrbitControls(true);
        }
        break;
      case 'CameraView': {
        const { position, rotation } = this.cameraData.get();

        const { x, y, z } = this.coordinates.applyOffset(position);
        this.camera.position.set(x, y, z);

        // Threejs camera is default facing towards to Z-axis negative direction,
        // but the actual camera is looking at Z-axis positive direction. So we need
        // to adjust the camera rotation considering the default camera orientation.
        this.camera.rotation.set(rotation.x + Math.PI, -rotation.y, -rotation.z);

        this.controls.enabled = false;

        const image = document.getElementById('camera-image');
        if (image && this.cameraData.imageSrcData) {
          image.src = this.cameraData.imageSrcData;
        }

        break;
      }
    }

    this.camera.updateProjectionMatrix();
  }

  enableMeasuringDistance() {
    this.scenarioEditor.enableEditingMode(this.camera, this.adc);
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.addEventListener('mousedown',
        this.onMouseDownDistance,
        false);
      element.addEventListener('mouseup',
        this.onMouseUpDistance,
        false);
      element.addEventListener('mousemove',
        this.onMouseMoveDistance,
        false);
    }
  }

  disableMeasuringDistance() {
    this.scenarioEditor.disableEditingMode(this.scene);
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.removeEventListener('mousedown',
        this.onMouseDownDistance,
        false);
      element.removeEventListener('mouseup',
        this.onMouseUpDistance,
        false);
      element.removeEventListener('mousemove',
        this.onMouseMoveDistance,
        false);
    }
    this.resetPointLine();
  }

  resetPointLine() {
    this.firstPointsDistanceMesh && (
      disposeMesh(this.firstPointsDistanceMesh),
      this.scene.remove(this.firstPointsDistanceMesh)
    );
    this.secondPointsDistanceMesh && (
      disposeMesh(this.secondPointsDistanceMesh),
      this.scene.remove(this.secondPointsDistanceMesh)
    );
    this.pointsLineMesh && (
      disposeMesh(this.pointsLineMesh),
      this.scene.remove(this.pointsLineMesh)
    );
    this.pointsLineTextMesh && (
      this.pointsLineTextMesh.children.forEach((c) => c.visible = false),
      this.scene.remove(this.pointsLineTextMesh)
    );
    this.selectedPointsDistances = [];
    this.firstPointsDistance = null;
    this.secondPointsDistance = null;
    this.pointsLineMesh = null;
    this.pointsLineTextMesh = null;
  }


  drawPoint(scene, pathPoint, coordinates, color = 0xA7C0B9) {
    const offsetPoint = coordinates.applyOffset(
      { x: pathPoint.x, y: pathPoint.y });
    const material = new THREE.MeshBasicMaterial({
      color: color,
      transparent: true,
      opacity: 1.0,
    });
    const pointMesh = drawCircle(0.5, material);
    pointMesh.position.set(offsetPoint.x, offsetPoint.y, 0.24);
    scene.add(pointMesh);
    return pointMesh;
  }

  onMouseDownDistance(event) {
    if (event.target &&  !_.isEqual('CANVAS', event.target.tagName)) {
      return;
    }
    if (event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    // return if the ground or coordinates is not loaded yet
    if (!this.coordinates.isInitialized() || !this.ground.mesh) {
      return;
    }

    if (this.firstPointsDistance && this.secondPointsDistance) {
      this.resetPointLine();
    }
  }

  onMouseMoveDistance(event) {
    const point = this.getGeolocation(event);
    this.pointsLineMesh && disposeMesh(this.pointsLineMesh);
    this.pointsLineMesh && this.scene.remove(this.pointsLineMesh);
    this.selectedPointsDistances = [];

    if (this.firstPointsDistance) {
      this.selectedPointsDistances.push(this.firstPointsDistance);
      this.selectedPointsDistances.push(point);
      const points = this.coordinates.applyOffsetToArray(
        this.selectedPointsDistances
      );
      this.pointsLineMesh = drawSegmentsFromPoints(
        points, 0xA7C0B9, 3, 6);
      this.pointsLineMesh.position.z = 0.24;
      this.scene.add(this.pointsLineMesh);
    }
  }

  onMouseUpDistance(event) {
    if (event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    if (!this.firstPointsDistance && !this.secondPointsDistance) {
      this.firstPointsDistance = this.getGeolocation(event);
      this.firstPointsDistanceMesh = this.drawPoint(
        this.scene, this.firstPointsDistance, this.coordinates);
      const element = document.getElementById(this.canvasId);
      if (element) {
        element.addEventListener('mousemove',
          this.onMouseMoveDistance,
          false);
      }
    } else if (this.firstPointsDistance && !this.secondPointsDistance) {
      this.secondPointsDistance = this.getGeolocation(event);
      this.secondPointsDistanceMesh = this.drawPoint(
        this.scene, this.secondPointsDistance, this.coordinates);
    }

    if (this.secondPointsDistance) {
      const point = this.coordinates.applyOffset(
        this.secondPointsDistance
      );
      const element = document.getElementById(this.canvasId);
      if (element) {
        element.removeEventListener('mousemove',
          this.onMouseMoveDistance,
          false);
      }
      const content =
        `Distance: ${this.firstPointsDistance.distanceTo(this.secondPointsDistance).toFixed(2)}m`;
      this.pointsLineTextMesh = this.textRender.drawText(content, this.scene, 0xA7C0B9);
      this.pointsLineTextMesh.position.set(
        point.x, point.y + 0.75, 2);
      this.scene.add(this.pointsLineTextMesh);
    }
  }

  enableScenarioEditing() {
    this.enableOrbitControls(false);
    this.scenarioEditor.enableEditingMode(this.camera, this.adc);
    this.scenarioEditor.displayAllRoutingPoints();
    this.scenarioEditor.displayAllObstacles();
  }

  disableScenarioEditing() {
    this.scenarioEditor.disableEditingMode(this.scene);
  }

  enableSelectorObject() {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.addEventListener('mousedown',
        this.onMouseDownSelectObject,
        false);
      element.addEventListener('mouseup',
        this.onMouseUpSelectObject,
        false);
      element.addEventListener('mousemove',
        this.onMouseMoveSelectObject,
        false);
    }
  }

  disableSelectorObject() {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.removeEventListener('mousedown',
        this.onMouseDownSelectObject,
        false);
      element.removeEventListener('mouseup',
        this.onMouseUpSelectObject,
        false);
      element.removeEventListener('mousemove',
        this.onMouseMoveSelectObject,
        false);
    }
  }

  onMouseDownSelectObject(event) {
    if (event.target && !_.isEqual('CANVAS', event.target.tagName)) {
      return;
    }
    if (event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    if (!this.coordinates.isInitialized() || !this.ground.mesh) {
      return;
    }

    this.curSelectObj = this.getMouseDownObject(event);
    if (this.curSelectObj) {
      this.startDragObject = true;
      this.controls.enabled = false;
      if (this.curSelectObj.name === 'obstacleGroup') {
        STORE.scenarioEditingManager.changeCurObstacle(this.curSelectObj.groupId);
      } else if (this.curSelectObj.name === 'trace') {
        STORE.scenarioEditingManager.changeCurObstacle(this.curSelectObj.obstacleTraceId,'trace');
      } else if (this.curSelectObj.name === 'trigger') {
        STORE.scenarioEditingManager.changeCurObstacle(this.curSelectObj.obstacleTriggerId,'trigger');
      }
    }
  }

  onMouseMoveSelectObject(event) {
    if (this.startDragObject) {
      const canvasPosition = event.currentTarget.getBoundingClientRect();

      const vector = new THREE.Vector3(
        ((event.clientX - canvasPosition.left) / this.dimension.width) * 2 - 1,
        -((event.clientY - canvasPosition.top) / this.dimension.height) * 2 + 1,
        0,
      );
      vector.unproject(this.camera);

      const direction = vector.sub(this.camera.position).normalize();
      const distance = -this.camera.position.z / direction.z;
      const pos = this.camera.position.clone().add(direction.multiplyScalar(distance));
      const geo = this.coordinates.applyOffset(pos, true);
      const offsetPoint = this.coordinates.applyOffset({ x: geo.x, y: geo.y });

      if (this.curSelectObj.name === 'obstacleGroup') {
        this.curSelectObj.children.forEach(item => {
          item.position.x = offsetPoint.x;
          item.position.y = offsetPoint.y;
        });
        STORE.scenarioEditingManager.changeObstaclePosition(geo);
      } else if (this.curSelectObj.name === 'trace') {
        this.curSelectObj.position.x = offsetPoint.x;
        this.curSelectObj.position.y = offsetPoint.y;
        STORE.scenarioEditingManager.changeTracePosition(geo,this.curSelectObj.obstacleTraceId);
      } else if (this.curSelectObj.name === 'trigger') {
        this.curSelectObj.position.x = offsetPoint.x;
        this.curSelectObj.position.y = offsetPoint.y;
        STORE.scenarioEditingManager.changeTriggerPosition(geo);
      }
    }
  }

  onMouseUpSelectObject() {
    this.startDragObject = false;
    this.curSelectObj = null;
    this.controls.enabled = true;
  }

  enableObstacleEditing() {
    document.getElementById(this.canvasId).addEventListener('mousedown',
      this.onMouseDownObstacleHandler,
      false);
    document.getElementById(this.canvasId).addEventListener('mouseup',
      this.onMouseUpObstacleHandler,
      false);
    document.getElementById(this.canvasId).addEventListener('mousemove',
      this.onMouseMoveObstacleHandler,
      false);
  }

  disableObstacleEditing() {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.removeEventListener('mousedown',
        this.onMouseDownObstacleHandler,
        false);
      element.removeEventListener('mouseup',
        this.onMouseUpObstacleHandler,
        false);
      element.removeEventListener('mousemove',
        this.onMouseMoveObstacleHandler,
        false);
    }
    this.obstaclePoint = null;
  }

  updateScenarioObstacles(obstacles) {
    for (let i = 0; i < obstacles.length; i++) {
      const selectedParkingSpaceIndex =
        this.scenarioEditor.updateObstacles(obstacles[i], this.coordinates, this.scene);
      if (selectedParkingSpaceIndex !== -1) {
        this.map.changeSelectedParkingSpaceColor(selectedParkingSpaceIndex);
      }
    }
  }

  removeAllObstacles() {
    const indexArr = this.scenarioEditor.removeAllObstacles(this.scene);
    if (!_.isEmpty(indexArr)) {
      indexArr.forEach(item => {
        this.map.changeSelectedParkingSpaceColor(item, 0xDAA520);
      });
    }
  }

  removeOneObstacle(obj) {
    const index = this.scenarioEditor.removeOneObstacle(obj,this.scene);
    if (index !== -1) {
      this.map.changeSelectedParkingSpaceColor(index, 0xDAA520);
    }
  }

  enableAddObstaclePathPoint() {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.addEventListener('mousedown',
        this.onMouseAddObstaclePointPath,
        false);
    }
  }

  disableAddObstaclePathPoint() {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.removeEventListener('mousedown',
        this.onMouseAddObstaclePointPath,
        false);
    }
  }

  addObstaclePathPoints(event) {
    if (event.target &&  !_.isEqual('CANVAS', event.target.tagName)) {
      return;
    }
    if (event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    // return if the ground or coordinates is not loaded yet
    if (!this.coordinates.isInitialized() || !this.ground.mesh) {
      return;
    }
    this.addObstaclePathPointTrace(this.getGeolocation(event),
      ObstacleColorMapping[STORE.scenarioEditingManager.curObstacle.type]);
    STORE.handleOptionToggle('addObstaclePathPoint');
  }

  addObstaclePathPointTrace(pathPoint, color) {
    this.scenarioEditor.addObstaclePathPointTrace(this.scene, pathPoint, this.coordinates, color);
  }

  removeOneObstacleTracePoint(id) {
    const index = this.scenarioEditor.removeOneObstacleTracePoint(this.scene,id);
    if (index !== -1) {
      this.map.changeSelectedParkingSpaceColor(index, 0xDAA520);
    }
  }

  removeAllObstacleTracePoints() {
    const indexArr = this.scenarioEditor.removeAllObstacleTracePoints(this.scene);
    if (!_.isEmpty(indexArr)) {
      indexArr.forEach(item => {
        this.map.changeSelectedParkingSpaceColor(item, 0xDAA520);
      });
    }
  }

  enableAddObstacleTriggerPoint() {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.addEventListener('mousedown',
        this.onMouseAddObstacleTriggerPoint,
        false);
    }
  }

  disableAddObstacleTriggerPoint() {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.removeEventListener('mousedown',
        this.onMouseAddObstacleTriggerPoint,
        false);
    }
  }

  addObstacleTriggerPoints(event) {
    if (event.target &&  !_.isEqual('CANVAS', event.target.tagName)) {
      return;
    }
    if (event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    // return if the ground or coordinates is not loaded yet
    if (!this.coordinates.isInitialized() || !this.ground.mesh) {
      return;
    }
    this.addObstacleTriggerPoint(this.getGeolocation(event),
      ObstacleColorMapping[STORE.scenarioEditingManager.curObstacle.type]);
    STORE.handleOptionToggle('addObstacleTriggerPoint');
  }

  addObstacleTriggerPoint(pathPoint, color) {
    this.scenarioEditor.addObstacleTriggerPoint(this.scene, pathPoint, this.coordinates, color);
  }

  removeOneObstacleTriggerPoint(id) {
    const index = this.scenarioEditor.removeOneObstacleTriggerPoint(this.scene,id);
    if (index !== -1) {
      this.map.changeSelectedParkingSpaceColor(index, 0xDAA520);
    }
  }

  enableRouteEditing() {
    document.getElementById(this.canvasId).addEventListener('mousedown',
      this.onMouseDownRouteHandler,
      false);
    document.getElementById(this.canvasId).addEventListener('mouseup',
      this.onMouseUpRouteHandler,
      false);
    document.getElementById(this.canvasId).addEventListener('mousemove',
      this.onMouseMoveRouteHandler,
      false);
  }

  disableRouteEditing() {
    const element = document.getElementById(this.canvasId);
    if (element) {
      element.removeEventListener('mousedown',
        this.onMouseDownRouteHandler,
        false);
      element.removeEventListener('mouseup',
        this.onMouseUpRouteHandler,
        false);
      element.removeEventListener('mousemove',
        this.onMouseMoveRouteHandler,
        false);
      this.startRoutingHeading = false;
      this.routingPoint = null;
    }
  }

  updateScenarioRouting(routes) {
    for (let i = 0; i < routes.length; i++) {
      if (routes[i].heading) {
        const coordinates = this.coordinates.applyOffset({x: routes[i].x,y: routes[i].y});
        this.scenarioEditor.drawPointArrowWithHeading(
          routes[i].heading,coordinates,this.scene,0xff0000);
      }
      const selectedParkingSpaceIndex =
        this.scenarioEditor.addRoutingPoint(routes[i], this.coordinates, this.scene);
      if (selectedParkingSpaceIndex !== -1) {
        this.map.changeSelectedParkingSpaceColor(selectedParkingSpaceIndex);
      }
    }
  }

  removeInvalidRoutingPoint(pointId, error) {
    const index = this.scenarioEditor.removeInvalidRoutingPoint(pointId, error, this.scene);
    if (index !== -1) {
      this.map.changeSelectedParkingSpaceColor(index, 0xDAA520);
    }
  }

  removeAllRoutingPoints() {
    const indexArr = this.scenarioEditor.removeAllRoutePoints(this.scene);
    if (!_.isEmpty(indexArr)) {
      indexArr.forEach(item => {
        this.map.changeSelectedParkingSpaceColor(item, 0xDAA520);
      });
    }
  }

  removeOneRoutingPoint(id) {
    const index = this.scenarioEditor.removeOneRoutingPoint(id,this.scene);
    if (index !== -1) {
      this.map.changeSelectedParkingSpaceColor(index, 0xDAA520);
    }
  }

  sendRoutingRequest(points = []) {
    return this.scenarioEditor.sendRoutingRequest(this.adc.mesh.position,
      this.adc.mesh.rotation.y,
      this.coordinates, points,
      STORE.scenarioEditingManager.isLoopRunning);
  }

  sendTemporaryMoveRequest(taskType) {
    return this.scenarioEditor.sendTemporaryMoveRequest(
      this.adc.mesh.position,
      this.adc.mesh.rotation.y,
      this.coordinates,
      taskType,
    );
  }

  endTemporaryMoveRequest() {
    return this.scenarioEditor.endTemporaryMoveRequest(
      this.adc.mesh.position,
      this.adc.mesh.rotation.y,
      this.coordinates,
    );
  }

  clearTemporaryMovePoint() {
    this.scenarioEditor.clearTemporaryMovePoint(this.scene);
  }

  hasTemporaryMovePoint() {
    return this.scenarioEditor.hasTemporaryMovePoint();
  }

  hideSimulationRoutingPoints() {
    this.scenarioEditor.hiddenAllRoutingPoints();
  }

  displaySimulationRoutingPoints() {
    this.scenarioEditor.displayAllRoutingPoints();
  }

  hideSimulationObstacles() {
    this.scenarioEditor.hiddenAllObstacles();
  }

  displaySimulationObstacles() {
    this.scenarioEditor.displayAllObstacles();
  }

  sendDemonstrateRequest(endPoint,endType,blacklistedLane) {
    return this.scenarioEditor.sendDemonstrateRequest(this.adc.mesh.position,
      this.adc.mesh.rotation.y,
      this.coordinates,endPoint,endType,blacklistedLane);
  }

  saveCurCarPosition(name) {
    return this.scenarioEditor.saveCurCarPosition(name,STORE.hmi.curCarPosition);
  }

  startLoopRunning() {
    return this.scenarioEditor.startLoopRunning(
      this.adc.mesh.position,
      this.adc.mesh.rotation.y,
      this.coordinates
    );
  }

  startRouteHandler(event) {
    if (event.target && preventEditTag.includes(event.target.tagName)) {
      return;
    }
    if (!this.scenarioEditor.isInEditingMode() || event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    // return if the ground or coordinates is not loaded yet
    if (!this.coordinates.isInitialized() || !this.ground.mesh) {
      return;
    }

    this.routingPoint = this.getGeolocation(event);
  }

  moveRouteHandler(event) {
    if (this.routingPoint) {
      this.scenarioEditor.drawPointArrow(
        this.getGeolocation(event), this.routingPoint,
        this.coordinates, this.scene, this.startRoutingHeading
      );
      this.startRoutingHeading = true;
    }
  }

  endRouteHandler(event) {
    if (event.target && preventEditTag.includes(event.target.tagName)) {
      return;
    }
    if (!this.scenarioEditor.isInEditingMode() || event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    if (this.routingPoint) {
      if (STORE.scenarioEditingManager.isTemporaryMoveSelecting) {
        this.scenarioEditor.addTemporaryMovePoint(
          this.routingPoint, this.coordinates, this.scene,
        );
        if (STORE.options.enableRoutingEditing) {
          STORE.handleOptionToggle('enableRoutingEditing');
        }
      } else {
        const selectedParkingSpaceIndex = this.scenarioEditor.addRoutingPoint(
          this.routingPoint, this.coordinates, this.scene, false,
        );
        if (selectedParkingSpaceIndex !== -1) {
          this.map.changeSelectedParkingSpaceColor(selectedParkingSpaceIndex);
        }
      }
    }
    this.routingPoint = null;
    this.startRoutingHeading = false;
    // STORE.handleOptionToggle('enableRoutingEditing');
  }

  // Obstacle Editing mouse event handlers
  startObstacleHandler(event) {
    if (event.target && preventEditTag.includes(event.target.tagName)) {
      return;
    }
    if (!this.scenarioEditor.isInEditingMode() || event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    // return if the ground or coordinates is not loaded yet
    if (!this.coordinates.isInitialized() || !this.ground.mesh) {
      return;
    }

    this.obstaclePoint = this.getGeolocation(event);
  }

  moveObstacleHandler(event) {
    if (this.obstaclePoint) {
      this.scenarioEditor.drawPointArrow(
        this.getGeolocation(event), this.obstaclePoint,
        this.coordinates, this.scene, this.startObstacleHeading
      );
      this.startObstacleHeading = true;
    }
  }

  endObstacleHandler(event) {
    if (event.target && preventEditTag.includes(event.target.tagName)) {
      return;
    }
    if (!this.scenarioEditor.isInEditingMode() || event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    if (this.obstaclePoint) {
      const selectedParkingSpaceIndex = this.scenarioEditor.addObstaclePoint(
        this.obstaclePoint, this.coordinates, this.scene
      );
      if (selectedParkingSpaceIndex !== -1) {
        this.map.changeSelectedParkingSpaceColor(selectedParkingSpaceIndex);
      }
    }
    this.obstaclePoint = null;
    this.startObstacleHeading = false;
    STORE.handleOptionToggle('enableObstacleEditing');
  }

  // Render one frame. This supports the main draw/render loop.
  render() {
    // TODO should also return when no need to update.
    if (!this.coordinates.isInitialized()) {
      return;
    }

    // Return if the car mesh is not loaded yet, or the ground is not
    // loaded yet.
    if (!this.adc.mesh || !this.ground.mesh) {
      return;
    }

    // Upon the first time in render() it sees ground mesh loaded,
    // added it to the scene.
    if (this.ground.type === 'default' && !this.ground.initialized) {
      this.ground.initialize(this.coordinates);
      this.ground.mesh.name = 'ground';
      this.scene.add(this.ground.mesh);
    }

    if (this.pointCloud.initialized === false) {
      this.pointCloud.initialize();
      this.scene.add(this.pointCloud.points);
    }

    this.adjustCamera(this.adc.mesh, this.options.cameraAngle);
    this.renderer.render(this.scene, this.camera);
  }

  animate() {
    requestAnimationFrame(() => {
      this.animate();
    });

    if (this.stats) {
      this.stats.update();
    }
    this.render();
  }

  updateWorld(world) {
    const adcPose = world.autoDrivingCar;
    this.adc.update(this.coordinates, adcPose);
    if (!_.isNumber(adcPose.positionX) || !_.isNumber(adcPose.positionY)) {
      console.error(`Invalid ego car position: ${adcPose.positionX}, ${adcPose.positionY}!`);
      return;
    }

    this.adc.updateRssMarker(world.isRssSafe);
    this.ground.update(world, this.coordinates, this.scene);
    this.planningTrajectory.update(world, world.planningData, this.coordinates, this.scene);
    this.planningStatus.update(world.planningData, this.coordinates, this.scene);

    const isBirdView = ['Overhead', 'Map'].includes(_.get(this, 'options.cameraAngle'));
    this.perceptionObstacles.update(world, this.coordinates, this.scene, isBirdView);
    this.decision.update(world, this.coordinates, this.scene);
    this.prediction.update(world, this.coordinates, this.scene);
    this.updateRouting(world.routingTime, world.routePath);
    this.gnss.update(world, this.coordinates, this.scene);
    this.map.update(world);

    const planningAdcPose = _.get(world, 'planningData.initPoint.pathPoint');
    if (this.planningAdc && planningAdcPose) {
      const pose = {
        positionX: planningAdcPose.x,
        positionY: planningAdcPose.y,
        heading: planningAdcPose.theta,
      };
      this.planningAdc.update(this.coordinates, pose);
    }

    const shadowLocalizationPose = world.shadowLocalization;
    if (shadowLocalizationPose) {
      const shadowAdcPose = {
        positionX: shadowLocalizationPose.positionX,
        positionY: shadowLocalizationPose.positionY,
        heading: shadowLocalizationPose.heading,
      };
      this.shadowAdc.update(this.coordinates, shadowAdcPose);
    }
  }

  updateRouting(routingTime, routePath) {
    this.routing.update(routingTime, routePath, this.coordinates, this.scene);
  }

  updateGroundImage(mapName) {
    this.ground.updateImage(mapName);
  }

  updateGroundMetadata(mapInfo) {
    this.ground.initialize(mapInfo);
  }

  updateMap(newData, removeOldMap = false) {
    if (removeOldMap) {
      this.map.removeAllElements(this.scene);
    }
    const extraInfo = this.map.appendMapData(newData, this.coordinates, this.scene);
    if (newData.parkingSpace && !_.isEmpty(extraInfo[0])) {
      this.scenarioEditor.setParkingSpaceInfo(
        newData.parkingSpace, extraInfo[0], this.coordinates, this.scene
      );
    }
    if (!_.isEmpty(extraInfo[1])) {
      this.scenarioEditor.setDeadJunctionInfo(extraInfo[1]);
    }
  }

  updatePointCloud(pointCloud) {
    if (!this.coordinates.isInitialized() || !this.adc.mesh) {
      return;
    }
    this.pointCloud.update(pointCloud, this.adc.mesh);
  }

  updateMapIndex(hash, elementIds, radius) {
    if (!this.scenarioEditor.isInEditingMode()
            || PARAMETERS.scenarioEditor.radiusOfMapRequest === radius) {
      this.map.updateIndex(hash, elementIds, this.scene);
    }
  }

  isMobileDevice() {
    return navigator.userAgent.match(/Android/i)
            || navigator.userAgent.match(/webOS/i)
            || navigator.userAgent.match(/iPhone/i)
            || navigator.userAgent.match(/iPad/i)
            || navigator.userAgent.match(/iPod/i);
  }

  getGeolocation(event) {
    if (!this.coordinates.isInitialized()) {
      return;
    }

    const canvasPosition = event.currentTarget.getBoundingClientRect();

    const vector = new THREE.Vector3(
      ((event.clientX - canvasPosition.left) / this.dimension.width) * 2 - 1,
      -((event.clientY - canvasPosition.top) / this.dimension.height) * 2 + 1,
      0,
    );

    vector.unproject(this.camera);

    const direction = vector.sub(this.camera.position).normalize();
    const distance = -this.camera.position.z / direction.z;
    const pos = this.camera.position.clone().add(direction.multiplyScalar(distance));
    const geo = this.coordinates.applyOffset(pos, true);

    return geo;
  }

  // Debugging purpose function:
  //  For detecting names of the lanes that your mouse cursor points to.
  getMouseOverLanes(event) {
    const canvasPosition = event.currentTarget.getBoundingClientRect();
    const mouse = new THREE.Vector3(
      ((event.clientX - canvasPosition.left) / this.dimension.width) * 2 - 1,
      -((event.clientY - canvasPosition.top) / this.dimension.height) * 2 + 1,
      0,
    );

    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, this.camera);
    const objects = this.map.data.lane.reduce(
      (result, current) => result.concat(current.drewObjects), []);
    const intersects = raycaster.intersectObjects(objects);
    const names = intersects.map((intersect) => intersect.object.name);
    return names;
  }

  getMouseDownObject(event) {
    const canvasPosition = event.currentTarget.getBoundingClientRect();
    const mouse = new THREE.Vector3(
      ((event.clientX - canvasPosition.left) / this.dimension.width) * 2 - 1,
      -((event.clientY - canvasPosition.top) / this.dimension.height) * 2 + 1,
      0,
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, this.camera);

    const obstacleGroup = [];
    this.scenarioEditor.obstacles.forEach(item => obstacleGroup.push(...item.children));

    const _objects = [...obstacleGroup,...this.scenarioEditor.obstacleTraceList,
      ...this.scenarioEditor.obstacleTriggerList];
    const intersects = raycaster.intersectObjects(_objects);
    let selectedObject = null;
    if (intersects.length > 0) {
      if (intersects[0].object.parent.name === 'obstacleGroup') {
        selectedObject = intersects[0].object.parent;
      } else {
        selectedObject = intersects[0].object;
      }
    }

    return selectedObject;
  }
}

const RENDERER = new Renderer();

export default RENDERER;
