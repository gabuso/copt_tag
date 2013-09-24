var alvar = require('../lib/alvar');
var fs = require('fs');

var marde = new alvar.Alvar();

fs.readFile('image_test.png', function (err, data) {
  if (err) throw err;

  marde.readImage(data);
  
});