import fast from './dist/web/dranimate-fast.js';
import fastModule from './dist/web/dranimate-fast.wasm';

//console.log('f', fast);
//console.log('fm', fastModule);

const module = fast({
  locateFile(path) {
    if(path.endsWith('.wasm')) {
      return fastModule;
    }
    return path;
  }
});

module.onRuntimeInitialized = () => {
  console.log('TEST!!!!', module._test());
  //console.log('CREATEMESH!!!!', module._createMesh());
};

export default module;

//export default dranimateFast;
//let test = Module().cwrap('test');
//module.exports = 'blee';
