var Lyric = require('lyric-node');
var sylvester = require('sylvester');
var Matrix = sylvester.Matrix;

module.exports = SignalWindow;

function SignalWindow(len,order_)
{
	
	this.time = [];
	this.sig = [];
	this.len = len;
	this.order = order_;
}

SignalWindow.prototype.add = function(t,val)
{
	if(this.sig.length == this.len)
	{
		this.sig.shift();
		this.time.shift();
	}
	this.time.push(t);
	this.sig.push(val);
}

SignalWindow.prototype.getlast = function()
{

	return this.sig[this.sig.length-1];

}

SignalWindow.prototype.getinterppoly = function(dt)
{

	if(dt > this.time[this.time.length-1] - this.time[0])
		throw("delay too large in regard with stored data");
	
	var input_model = new Array();
	var time_d = new Array(this.time.length);
	
	for(var i=0;i<this.time.length;i++)
	{

		time_d[i] = this.time[i] - this.time[this.time.length-1];
	}

	input_model["x"] = time_d;
	input_model["y"] = this.sig;

	return Lyric.buildModel(input_model,this.order);
}

SignalWindow.prototype.getsigdelayed = function(dt)
{
	if(this.sig.length < this.len)
	{
		return 0;
	}

	var coeffs_poly = this.getinterppoly();

	var eval = new Array();
	eval["x"]= [-dt];

	var eval_y =  Lyric.applyModel(eval,coeffs_poly,this.order);

	return eval_y[0].y;
}	

SignalWindow.prototype.getdersigdelayed = function(dt,nd)
{
	if(this.sig.length < this.len)
	{
		return 0;
	}

	var coeffs_poly = this.getinterppoly();

	var self = this;
	var dd = function(o){
		if(o == 0)
		{
			return coeffs_poly;
		}
		else
		{
			var coeffs_der_poly = Matrix.Zeros(self.order+1,1);
		
			for(var i=0;i<self.order;i++)
			{
			
				var v =dd(o-1);
				coeffs_der_poly.elements[i][0] = v.elements[i+1][0]*(i+1);
			}
			
			return coeffs_der_poly;
		}
	}
	
	var eval = new Array();
	eval["x"]= [-dt];

	var eval_y =  Lyric.applyModel(eval,dd(nd),this.order);

	return eval_y[0].y;
}	
