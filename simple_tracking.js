var alvar_bind = require('./alvar/lib/alvar');
var fs = require('fs');
var http = require('http');
var ardrone = require('ar-drone');
var util = require("util");
var events = require("events");
var sigwin = require('./filters/signalwindow');
var Rls_UD_filter = require('./filters/Rls_UD_filter');
var sylvester = require('sylvester');
var Matrix = sylvester.Matrix;
var Vector = sylvester.Vector;


var param_estim = new Rls_UD_filter(3);


//var  async = require('async');

var alvar = new alvar_bind.Alvar();

var DT = 150; // time between faces detection

var client, io, lastPng;
var tracking = false;
var debug = true;
var processingImage = false;

var enable_control = true;

var times = [];

var date_last_vid = new Date();
var date_last_nav = new Date();

var last_navdata;
var d = new Date();
var  log_file_name = './datalog/log_'+d.getDate()+(d.getMonth()+1)+d.getFullYear()+'_'+d.getHours()+'h'+d.getMinutes()+'.txt';

var last_cmd = {forward_cmd:NaN, side_cmd:NaN,depth:NaN,side_error:NaN,inte_x:0,inte_y:0};

var took_off = false;

var n_win= 25;
var order_interp = 5;
var vx_sig = new sigwin(n_win,order_interp);
var vy_sig = new sigwin(n_win,order_interp);
var pitch_sig = new sigwin(n_win,order_interp);
var roll_sig = new sigwin(n_win,order_interp);

var processingNavdata = false;

var vxc = 0;
var vyc = 0;
var inte_vx = 0;
var inte_vy = 0;

var org = function (){return {x:0 , y:0};};

//***********************************************//
// Emergy abort command 
//***********************************************//
process.stdin.resume();
process.stdin.setEncoding('utf8');
process.stdin.on('data', function (chunk) {
  chunk = chunk.trim();
  if (chunk == 'x') abort();
});

var aborting = false;
var abort = function () {
  console.log('Aborting...');
  aborting = true;
  client.stop();
  client.land();
  setInterval(function(){process.exit(1);},500);
};
//***********************************************//
//***********************************************//

var id_target = 0; 

var x1_c = 270;
var y1_c = 130;

var x2_c = 270;
var y2_c = 230;

var x3_c = 370;
var y3_c = 230;

var x4_c = 370;
var y4_c = 130;


//***********************************************//
// General Configuration
//***********************************************//
var client = new ardrone.createClient({timeout:4000});

client.config('general:navdata_demo', 'FALSE');
client.config('video:video_channel', '0');

var drone_conf = {'max_pitch':0, 'max_roll':0};

//***********************************************//
//***********************************************//


function apply_cmd(front,side,vert){

     if(!aborting)
     {   
        if(front<0)
        {
        //  console.log("going back");
          client.back(Math.abs(front));
        }
        else if(front>0)
        {
      //    console.log("going front");
          client.front(Math.abs(front));
        }

        if(side<0)
        {
      //    console.log("going left");
          client.left(Math.abs(side));
        }
        else if(side>0)
        {
      //    console.log("going right");
          client.right(Math.abs(side));
        }
        
        // if(vert<0)

        //   client.down(Math.abs(vert));
        // }
        // if(vert>0)
        //   client.up(Math.abs(vert));
        // }

        

      }
};


//***********************************************//
// Run Marker Detection Code
//***********************************************//

function detectMarker() {
	tracking=true;
	processingImage =false;
// console.log("hh2");
if(tracking && (!processingImage) && lastPng) {
  processingImage = true;
  var d_now = new Date();
    // console.log("ff");
    var dt = d_now-date_last_vid;
    if(dt>20)
    {
       console.log(d_now-date_last_vid);
      date_last_vid = d_now;
      alvar.treatImageBuffer(lastPng);
      // console.log(alvar_res)
      if(alvar.GetNumMarkerDetected()> 0)
      {	
                var target_found = false;
                var id_marker = -1;
                 markers_coord = alvar.getMarkersCoord();

                  for(var i=0;i<markers_coord.length;i++)
                    {
                      if(markers_coord[i].id==id_target)
                      {
                        target_found =true;
                        id_marker = i;
                        break;

                      }
                    }


                if(target_found)
                {
                        // var dist = function(pt1,pt2){return Math.sqrt((pt2.x-pt1.x)*(pt2.x-pt1.x)+(pt2.y-pt1.y)*(pt2.y-pt1.y));};

                        // var perimeter = function(m) {return dist(m.pt1,m.pt2)+dist(m.pt2,m.pt3)+dist(m.pt3,m.pt4)+dist(m.pt4,m.pt1);};

                        // var per =perimeter(x1,y1,x2,y2,x3,y3,x4,y4); 
                        // var per_c =perimeter(x1_c,y1_c,x2_c,y2_c,x3_c,y3_c,x4_c,y4_c);

                        var marker =markers_coord[id_marker];

                        var depth = alvar.GetDepthPix(id_target);

                        var side_error = ((marker.pt1.x + marker.pt2.x)/2 - ((x1_c + x2_c)/2+50)) ;

                       //  var k_damp = 0.012;
                       //  //0.02
                       //  var forward_cmd = 0.1 * (0.65-depth) - k_damp*last_navdata.demo.velocity.x/1000;
                       // // console.log("jj : " + depth);
                       // console.log("for :" + forward_cmd);
                       // //0.0002
                       // var side_cmd = -0.0006 * ((marker.pt1.x + marker.pt2.x)/2 - (x1_c + x2_c)/2)  - 0.06*last_navdata.demo.velocity.y/1000;

                       // console.log("side_cmd :" + side_cmd);

                       // var vert_cmd = 0.0002 * ((marker.pt1.y + marker.pt2.y)/2 - (y1_c + y2_c)/2);

                       // console.log("  ");

                      var x_e = (0.8-depth);
                      inte_vx = inte_vx + 0.03*dt/1000*x_e;
                       vxc = 0.4*x_e + inte_vx;


                      inte_vy = inte_vy + 0*0.00001*dt/1000*side_error;
                      //0.001
                       vyc = 0.002*side_error+inte_vy; 

                       // vxc=0;
                       // vyc=0;

                       // last_cmd.forward_cmd = forward_cmd;
                       // last_cmd.side_cmd = side_cmd;
                       console.log(vxc+"   "+vyc);

                       last_cmd.depth = depth;
                       last_cmd.side_error = side_error;

                               if(enable_control)
                               {
                                //  apply_cmd(forward_cmd,side_cmd,vert_cmd);
                                
                                }

               } 
          }

        }
// client.stop();
} else {
  if (tracking) setTimeout(detectMarker, DT);
};

};

client.takeoff(function(){took_off=true;});

client.createPngStream()
.on('error', console.log)
.on('data', function(pngBuffer) {
  lastPng = pngBuffer;
// console.log("nez");
detectMarker();
});

client.on('navdata',function(navdata){
  var d_now = new Date();
  var dt = d_now-date_last_nav;
  //console.log(d_now-date_last_nav);

  processingNavdata = false;

  if(dt>19 &&  navdata.hasOwnProperty('demo') && (!processingNavdata))
  {
    processingNavdata = true;
   

    date_last_nav = d_now;
    last_navdata = navdata;

    var t_s = d_now.getTime()/1000;

    vx_sig.add(t_s,last_navdata.demo.velocity.x/1000);
    vy_sig.add(t_s,last_navdata.demo.velocity.y/1000);
    pitch_sig.add(t_s,last_navdata.demo.rotation.pitch);
    roll_sig.add(t_s,last_navdata.demo.rotation.roll);

    var y_e = vx_sig.getdersigdelayed(0.2,1);
    var m_vec = [vx_sig.getsigdelayed(0.2) ,pitch_sig.getsigdelayed(0.2), 1];
    param_estim.add(y_e,m_vec);
  
    // if( last_navdata.demo.altitude >0.5)
    // {
    //   took_off = true;
    // }
    if(took_off)
    {


      // var vxc = 0;
      //  var vx = last_navdata.demo.velocity.x/1000;
      //  last_cmd.inte_x = last_cmd.inte_x + 0.05*dt/1000.0*(vxc-vx);
      //  var forward_cmd = (-0.62*vx+0.1*vxc + last_cmd.inte_x); 

      //  //var vyc = 0;
      //  var vy = last_navdata.demo.velocity.y/1000;
      //  last_cmd.inte_y = last_cmd.inte_y + 0.05*dt/1000.0*(vyc-vy);
      // var side_cmd = -0.62*vy+0.1*vyc + last_cmd.inte_y; 

       var vx = last_navdata.demo.velocity.x/1000;
       var ki_x = 0;
       if(Math.abs(vxc)<1e-3)
       {
        ki_x=0.3;
       }
       last_cmd.inte_x = last_cmd.inte_x + ki_x*dt/1000.0*(vxc-vx);
       //0.7 0.6
       var forward_cmd = (-1.0*vx+0.4*vxc + last_cmd.inte_x); 

       //var vyc = 0;
       var vy = last_navdata.demo.velocity.y/1000;
       var ki_y = 0;
       if(Math.abs(vyc)<1e-3)
       {
        ki_y=0.3;
       }
       //0.7 0.6
       last_cmd.inte_y = last_cmd.inte_y + ki_y*dt/1000.0*(vyc-vy);
      var side_cmd = -1.0*vy+0.4*vyc + last_cmd.inte_y; 
       

       vert_cmd = 0;

        if(enable_control)
         {
            apply_cmd(forward_cmd,side_cmd,vert_cmd);
          
          }

          last_cmd.forward_cmd = forward_cmd;
           last_cmd.side_cmd = side_cmd;

     }

    // var data_tosave = d_now.getTime()+';'+last_navdata.demo.velocity.x+';'+last_navdata.demo.velocity.y+';'+depth+';'+(((marker.pt1.x + marker.pt2.x)/2 - (x1_c + x2_c)/2)) +';'+forward_cmd+';'+side_cmd+';'+last_navdata.demo.altitude+'\n';
    var data_tosave = d_now.getTime()+';'+last_navdata.demo.velocity.x+';'+last_navdata.demo.velocity.y+';'+last_cmd.depth+';'
    +last_cmd.side_error+';'+last_cmd.forward_cmd+';'+last_cmd.side_cmd+';'+last_navdata.demo.altitude+';'+last_cmd.inte_x+';'
    +last_cmd.inte_y+';'+last_navdata.demo.rotation.pitch+';'+last_navdata.demo.rotation.roll+';'+last_navdata.references.pitch+';'
    +last_navdata.references.roll+';'+
    vx_sig.getsigdelayed(0.2)+';'+vy_sig.getsigdelayed(0.2)+';'+pitch_sig.getsigdelayed(0.2)+';'+roll_sig.getsigdelayed(0.2)+';'+
    param_estim.param.elements[0]+';'+param_estim.param.elements[1]+';'+param_estim.param.elements[2]+';'+param_estim.yhat_n+';'+
    vxc+';'+vyc+';'
    +'\n';
                                        fs.appendFile(log_file_name, data_tosave, function (err) {
                                          if (err) throw err;
                                          
                                        });
  //console.log(Math.floor(navdata.demo.velocity.x*1000)/1000 +" "+Math.floor(navdata.demo.velocity.y*1000)/1000);
}
// console.log(navdata);
//exit();
});

