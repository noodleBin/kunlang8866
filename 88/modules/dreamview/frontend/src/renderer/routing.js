import STORE from 'store';

import { drawThickBandFromPoints } from 'utils/draw';

const _ = require('lodash');

export default class Routing {
  constructor() {
    this.routePaths = [];
    this.lastRoutingTime = -1;
    this.allRoutes = [];
  }

  update(routingTime, routePath, coordinates, scene) {
    this.routePaths.forEach((path) => {
      path.visible = STORE.options.showRouting;
    });
    // There has not been a new routing published since last time.
    if (this.lastRoutingTime === routingTime || routePath === undefined) {
      return;
    }

    this.lastRoutingTime = routingTime;

    this.drawRoutingPath(routePath, coordinates, scene);
  }

  drawRoutingPath(routePath, coordinates, scene) {
    this.routePaths.forEach((path) => {
      scene.remove(path);
      path.material.dispose();
      path.geometry.dispose();
    });
    this.routePaths = [];

    routePath[0].Path.forEach((path) => {
      const points = coordinates.applyOffsetToArray(path.point);
      const pathMesh = drawThickBandFromPoints(
        points,
        0.3 /* width */,
        0xff0000 /* red */,
        0.6,
        5 /* z offset */,
      );
      pathMesh.visible = STORE.options.showRouting;
      scene.add(pathMesh);
      this.routePaths.push(pathMesh);
    });
  }

  updateRoutingPath(route) {
    if (
      this.lastRoutingTime === route.routingTime ||
      route.routePath === undefined
    ) {
      return;
    }

    this.lastRoutingTime = route.routingTime;
    this.allRoutes = route;
  }
}
