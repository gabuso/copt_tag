var sylvester = require('sylvester');
var Matrix = sylvester.Matrix;
var Vector = sylvester.Vector;

module.exports = Rls_UD_filter;


function Rls_UD_filter(npar_)
{
	this.npar = npar_;

	this.U = Matrix.I(this.npar);
	this.D = Vector.Zero(this.npar);
	for(var i =0;i<this.npar;i++)
	{

		this.D.elements[i] = 1e5;		
	}

	this.param = Vector.Zero(this.npar);

	this.sig_eps = 1;

	this.Qeps = Matrix.Zeros(this.npar,this.npar);

	this.yhat_n = 0;

	this.delta_n = 0;

}


Rls_UD_filter.prototype.add = function(y_e,m_vec_)
{

    var m_vec  = Vector.create(m_vec_);
	var x = m_vec;
	yhat = 0;

	for(var i=0;i<this.npar;i++)
	{

		yhat = yhat + this.param.elements[i]*m_vec.elements[i];
	}

	f = Vector.Zero(this.npar);
	v = Vector.Zero(this.npar);

	delta  = y_e - yhat;

	for(var j =this.npar-1;j>=0;j--)
	{
        f.elements[j] = x.elements[j];
        
        for(var i=0;i<j;i++)
        {
            f.elements[j] = f.elements[j] + this.U.elements[i][j]*x.elements[i];
        }
        
        v.elements[j] = this.D.elements[j]*f.elements[j];
        
    }


    var alpha = this.sig_eps*this.sig_eps+ f.elements[0]*v.elements[0];
    var gamma= 1/alpha;
    this.D.elements[0] = this.sig_eps*this.sig_eps*gamma*(this.D.elements[0]+this.Qeps.elements[0][0]);
    
    for(var j=1;j<=this.npar-1;j++)
    {       
        var beta = alpha;
        alpha = alpha + f.elements[j]*v.elements[j];
        var lambda = - f.elements[j]/beta;
        gamma = 1/alpha;
        
        this.D.elements[j] = beta*gamma*(this.D.elements[j]+this.Qeps.elements[j][j]);
        
        for(var i=0;i<=j-1;i++)
        {
            beta = this.U.elements[i][j];
            this.U.elements[i][j] = beta+v.elements[i]*lambda;
            v.elements[i] = v.elements[i]+v.elements[j]*beta;
            
        }
    }

    var epsilon =delta*gamma;
    
    for(var i=0;i<this.npar;i++)
    {
        this.param.elements[i] = this.param.elements[i]+v.elements[i]*epsilon;  
    }

    this.yhat_n = 0;
    for(var i=0;i<this.npar;i++)
	{

		this.yhat_n = this.yhat_n + this.param.elements[i]*m_vec.elements[i];
	}

	this.delta_n = (y_e-this.yhat_n);
}


    


