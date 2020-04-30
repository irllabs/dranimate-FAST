import fast from './dist/web/dranimate-fast.js';
import fastModule from './dist/web/dranimate-fast.wasm';

const module = fast({
  locateFile(path) {
    if(path.endsWith('.wasm')) {
      return fastModule;
    }
    return path;
  }
});

export default {
  onLoad: (handler) => {
    module.onRuntimeInitialized = handler; 
  },
  // [vertices] Flat array of 2D or 3D vertices
  // [faces] Flat array of face indicies
  // [handles] Flat array of 2D or 3D handle coordinates
  // [dimensions] 2 or 3
  createShape: (vertices, faces, handles, dimensions) => {
    console.log('---------------------------------');
    console.log('Dranimate FAST: Creating ' + dimensions + 'D shape');
    console.log('---------------------------------');
    console.log(vertices.length / dimensions + ' vertices');
    console.log(faces.length / 3 + ' faces');
    console.log(handles.length / dimensions + ' handles');
    console.log('---------------------------------');
    return new module.Shape(vertices, faces, handles, dimensions);
  }
};
