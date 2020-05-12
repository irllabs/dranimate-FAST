// FAST Atomatic Skinning Transforms
// Dranimate implementation
// Non-TypeScript for easier universal importing

import fast from './dist/web/dranimate-fast.js';
import fastModule from './dist/web/dranimate-fast.wasm';

const cppModule = fast({
  locateFile(path) {
    if(path.endsWith('.wasm')) {
      return fastModule;
    }
    return path;
  }
});

// Module loaded callback
export function onLoad(handler) { 
  cppModule.onRuntimeInitialized = handler; 
}

// C++ (WASM) Shape class wrapper
export class Shape {
  // Construct a FAST shape 
  // [vertices] Flat array of 2D or 3D vertices
  // [faces] Flat array of face indicies
  // [handles] Flat array of 2D or 3D handle coordinates
  // [dimensions] 2 or 3
  constructor(vertices, faces, dimensions) {
    console.log('-----------------------');
    console.log('Dranimate FAST ' + dimensions + 'D shape');
    console.log('-----------------------');
    console.log(vertices.length / dimensions + ' vertices');
    console.log(faces.length / 3 + ' faces');
    console.log('-----------------------');
    this.cppShape = new cppModule.Shape(vertices, faces, dimensions);
  }
	// Add control point at specific vertex
	addControlPoint(vertexIndex) {
		//console.log('SHAPE: addControlPoint');
    this.cppShape.addControlPoint(vertexIndex);
	}
	// Set control point position
	setControlPointPosition(controlPointIndex, x, y) {
		//console.log('SHAPE: setControlPointPosition', controlPointIndex, x, y);
    this.cppShape.setControlPointPosition(controlPointIndex, x, y);
	}
	// Precompute Fast ARAP DOF
	precompute() {
		//console.log('SHAPE: precompute');
    this.cppShape.precompute();
	}
  // Update FAST shape
  // Returns skinned vertices for frame
  update() {
    return this.cppShape.update();
  }
  // Get Bounded Biharmonic computed skinning weights
  getWeights() {
    return this.cppShape.getWeights();
  }
}
