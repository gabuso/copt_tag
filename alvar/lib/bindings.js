module.exports = require('../build/Release/alvar.node');


try {
  module.exports = require('../build/Release/alvar.node');
} catch (e) { try {
  module.exports = require('../build/default/alvar.node');
} catch (e) {
  throw e;
}}