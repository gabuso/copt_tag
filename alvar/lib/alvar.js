var Stream = require('stream').Stream
  , Buffers = require('buffers')
  , util = require('util')
  , path = require('path')

var bindings = require('./bindings')

var alvar = module.exports = {};

alvar.__proto__ = bindings;
/*

# Matrix #
The matrix is one of opencv's most core datatypes.

*/


var matrix = alvar.Matrix.prototype;
var al = alvar.Alvar.prototype;

al.getMarkersCoord = function()
{

	res = this.GetMarkersCoord();
	n = this.GetNumMarkerDetected();
	var org = function (){return {x:0 , y:0};};
	var marker= function() { return { id:-1, pt1 : org(),pt2 : org(),pt3 : org(),pt4 : org()};};

	var marker_array = [];

	var k = 0;
	for(var i=0;i<n;i++)
	{
		var cur_mark = marker();

		cur_mark.id = res[k];

		cur_mark.pt1.x = res[k+4];
		cur_mark.pt1.y = res[k+5];

		cur_mark.pt2.x = res[k+6];
		cur_mark.pt2.y = res[k+7];

		cur_mark.pt3.x = res[k+8];
		cur_mark.pt3.y = res[k+9];

		cur_mark.pt4.x = res[k+10];
		cur_mark.pt4.y = res[k+11];

		k = k+12;

		marker_array.push(cur_mark);
	}


	return marker_array;
};


