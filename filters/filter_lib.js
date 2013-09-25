var sylvester = require('sylvester');
var Matrix = sylvester.Matrix;
var Vector = sylvester.Vector;

module.exports = Filter;
var fact =  [1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880, 3628800, 39916800, 479001600];

var prod_poly = function(a,b)
{
	var n = a.length;
	var m = b.length;

	var c = new Array((n-1)+(m-1)+1);

	for(var i=0;i<(n-1)+(m-1)+1;i++)
	{
		c[i] = 0;
		for(var j=0;j<=i;j++)
		{
			if(j>=n || i-j>=m)
				continue;
			
			c[i] = c[i] + a[j]*b[i-j];
		}

	}

	return c;


}


function Filter(order,w,Te,d)
{

	this.u = new Array(order+1);
	this.y = new Array(order+1);
	for(var i=0;i<order+1;i++)
	{
		this.u[i] = 0;
		this.y[i] = 0;
	}
	this.len = order+1;
	this.coeffs = Filter.get_coeffs(order,w,Te,d);

}

Filter.prototype.filt = function(un)
{
	

	this.u.shift();
	this.u.push(un);
	


	var a0 = this.coeffs.a[0];

	var y = 0;
	var n = this.coeffs.b.length; 
	for(var i=0;i<n;i++)
	{
		y = y + this.coeffs.b[i]/a0*this.u[n-1-i];
	}
	
	var n = this.coeffs.a.length; 
	for(var i=1;i<n;i++)
	{
		y = y -this.coeffs.a[i]/a0*this.y[n-1-i+1];
	}
	y = y/a0;
	this.y.shift();
	this.y.push(y);
	

	return y;
};


Filter.get_coeffs = function(order,w,Te,d)
{
	var n=order;
	var w0=2/Te;
	var bp = new Array(n+1);
	var bp1,bp2;
	if(d>=1)
	{
		 bp1 = new Array(n-d+1);
		 bp2 = new Array(d+1);
	}
	var ap = new Array(n+1);

	var K = Math.pow(w,n)/Math.pow(w0+w,n);
	var Kd = K*Math.pow(w0,d);

	for( var i=1;i<=n+1;i++)
	{
		binom = fact[n]/(fact[i-1]*fact[n-(i-1)]);
		if(d<1)
		{
	    	bp[i-1]= K*binom;
	    }
	    else
	    {
	    	if(i-1<n-d+1)
	    	{
	    		binom_nmd = fact[n-d]/(fact[i-1]*fact[n-d-(i-1)]);
	    		bp1[i-1] = Kd*binom_nmd;
	    	}
	    	if(i-1<d+1)
	    	{
	    		binom_d = fact[d]/(fact[i-1]*fact[d-(i-1)]);
	    		bp2[i-1] = Math.pow(-1,d-(i-1))*binom_d;
	    	}
	    }
	    ap[i-1] = binom*Math.pow(((w-w0)/(w+w0)),((i-1)));
	}

	if(d>=1)
	{	
		bp = prod_poly(bp1,bp2);
	}

	return {'a':ap,'b':bp};


};

