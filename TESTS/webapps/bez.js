
// everything is a global variable? sure.
var bcanv, bctx;
var btimeout;
var bdt = 0.005;
var btime = 0.0;
var bstep = 0;
var bmode = 0;
var blevel = 7;
var bsimrun = false; // allows sim to be paused
var bdragging = false;
var bslider;
var numanc = 7;
var bxanc = [-50,-80,-100,0,100,80,50];
var byanc = [0,0,50,200,50,0,0];
var bxpos, bypos;
var fdf = 0.5;

// this gets called on page load?
bez_init();
function bez_init()
{
	bcanv = document.getElementById('bez_sim');
	bctx = bcanv.getContext('2d');
	bctx.font="Bold 12px Arial";

	bxpos = bxanc[0];
	bypos = byanc[0];

	var t = bez_getLength();
	fdf = t/(t+100.);

	// generic listeners from some other code I wrote
	bcanv.addEventListener('click', bez_handleclick);
	bcanv.addEventListener('mousedown', bez_handlemousedown);
	bcanv.addEventListener('mousemove', bez_handlemousemove);
	bcanv.addEventListener('mouseup', bez_handlemouseup);
	bsimrun = true;
	bez_draw();
	bez_run();
}


function bez_handlemousedown(event)
{
	var x = event.pageX - bcanv.offsetLeft;
	var y = event.pageY - bcanv.offsetTop;
	bdragging = true;
	bslider = -1;

	bslider = -1;
	for (var i=1; i<numanc-1; i++)
	{
		if (Math.sqrt(Math.pow(x-250-bxanc[i],2) + Math.pow(y-250+byanc[i],2)) <= 8)
		  bslider = i;
	}
}
function bez_handlemouseup(event)
{
	bdragging = false;
	bslider=-1;
	var t = bez_getLength();
	fdf = t/(t+100.);
}

function bez_handlemousemove(event)
{
	if (bdragging && bslider > 0)
		bez_moveAncor(event.pageX - bcanv.offsetLeft, event.pageY - bcanv.offsetTop);
}

function bez_moveAncor(x, y)
{
	var value, logval;

	if (bslider > 0)
	{
		bxanc[bslider] = x-250;
		byanc[bslider] = -y+250;
	}

	if (bsimrun==false) bez_draw();
}

function bez_handleclick(event)
{
  var x = event.pageX - bcanv.offsetLeft;
  var y = event.pageY - bcanv.offsetTop;

  if (bsimrun==false) bez_draw();
}


// contains the primary calculations of the sim
function bez_run()
{
	// call run again in a moment
	if (bsimrun==true) btimeout = setTimeout('bez_run()', 30); // 30ms delay?

	btime += bdt;
	if (btime > 1.0) btime -= 1.0;
	if (btime < fdf)
	{
		bxpos = bez_getX(btime/fdf);
		bypos = bez_getY(btime/fdf);
	} else {
		bxpos = 50. - 100.*(btime-fdf)/(1.-fdf);
		bypos = 0.0;
	}

	
	bez_draw();
}

function bez_getLength()
{
	var l, i, bx, by, bxo, byo;
	l = 0.0;
	bxo = bez_getX(0.0);
	byo = bez_getY(0.0);
	for (i=1; i<=100; i++)
	{
		bx = bez_getX(i*0.01);
		by = bez_getY(i*0.01);
		l += Math.sqrt((bx-bxo)*(bx-bxo) + (by-byo)*(by-byo));
		bxo = bx;
		byo = by;
	}
	return l;
}

function bez_getX(t)
{
	var temp = Array(numanc);
	var i, j;
	for (i=0; i<numanc; i++)
		temp[i] = bxanc[i];
	for (i=0; i<numanc-1; i++)
	{
		for (j=0; j<numanc-i-1; j++)
			temp[j] = (1.0-t)*temp[j] + t*temp[j+1];
	}
	return temp[0];
}
function bez_getY(t)
{
	var temp = Array(numanc);
	var i, j;
	for (i=0; i<numanc; i++)
		temp[i] = byanc[i];
	for (i=0; i<numanc-1; i++)
	{
		for (j=0; j<numanc-i-1; j++)
			temp[j] = (1.0-t)*temp[j] + t*temp[j+1];
	}
	return temp[0];
}

function bez_draw()
{
	var i, j, t;
	var tempx = Array(numanc);
	var tempy = Array(numanc);
	var px, py;

	bctx.fillStyle = 'white';
	bctx.fillRect(0, 0, 500, 350);

	// grid
	bctx.lineWidth = 1;
	bctx.strokeStyle = '#dddddd';
	for (var i=0; i<10; i++)
	{
		bctx.beginPath();
		bctx.moveTo(i*50,0);
		bctx.lineTo(i*50,350);
		bctx.stroke();
		bctx.beginPath();
		bctx.moveTo(0,i*50);
		bctx.lineTo(500,i*50);
		bctx.stroke();
	}

	// draw borders
	bctx.fillStyle = '#666666';
	bctx.fillRect(0, 0, 500, 3);
	bctx.fillRect(0, 0, 3, 350);
	bctx.fillRect(0, 347, 500, 3);
	bctx.fillRect(497, 0, 3, 450);

	// hard bottom
	bctx.lineWidth = 4;
	bctx.strokeStyle = '#666666';
	bctx.beginPath();
	bctx.moveTo(250-50,250);
	bctx.lineTo(250+50,250);
	bctx.stroke();

	// helper lines
if (btime < fdf)
{
	t = btime/fdf;
	// load data
	for (i=0; i<numanc; i++)
	{
		tempx[i] = bxanc[i];
		tempy[i] = byanc[i];
	}
	// iterate
	for (i=0; i<numanc-1; i++)
	{
		for (j=0; j<numanc-i-1; j++)
		{
			tempx[j] = (1.0-t)*tempx[j] + t*tempx[j+1];
			tempy[j] = (1.0-t)*tempy[j] + t*tempy[j+1];
		}
		if (blevel > i)
		{
			bctx.lineWidth = 2;
			bctx.strokeStyle = '#dddddd';
			bctx.beginPath();
			bctx.moveTo(250+tempx[0],250-tempy[0]);
			for (j=1; j<numanc-i-1; j++)
			{
				bctx.lineTo(250+tempx[j],250-tempy[j]);
			}
			bctx.stroke();
		}
	}
	/*
	if (blevel > 1)
	{
		bctx.lineWidth = 2;
		bctx.strokeStyle = '#dddddd';
		bctx.beginPath();
		x = bxanc[0] + (bxanc[1]-bxanc[0])*(btime/fdf);
		y = byanc[0] + (byanc[1]-byanc[0])*(btime/fdf);
		bctx.moveTo(250+x,250-y);
		for (var i=2; i<numanc; i++)
		{
			x = bxanc[i-1] + (bxanc[i]-bxanc[i-1])*(btime/fdf);
			y = byanc[i-1] + (byanc[i]-byanc[i-1])*(btime/fdf);
			bctx.lineTo(250+x,250-y);
		}
		bctx.stroke();
		bctx.fillStyle = '#aaaaff';
		x0 = bxanc[0] + (bxanc[1]-bxanc[0])*(btime/fdf);
		y0 = byanc[0] + (byanc[1]-byanc[0])*(btime/fdf);
		for (var i=2; i<numanc; i++)
		{
			x1 = bxanc[i-1] + (bxanc[i]-bxanc[i-1])*(btime/fdf);
			y1 = byanc[i-1] + (byanc[i]-byanc[i-1])*(btime/fdf);
			x = x0 + (x1-x0)*(btime/fdf);
			y = y0 + (y1-y0)*(btime/fdf);
			bctx.beginPath();
			bctx.arc(250+x,250-y,4,0,2*Math.PI,false);
			bctx.fill();
			x0 = x1;
			y0 = y1;
		}
	}
	if (blevel > 0)
	{
	bctx.lineWidth = 2;
	bctx.strokeStyle = '#dddddd';
	bctx.beginPath();
	bctx.moveTo(250+bxanc[0],250-byanc[0]);
	for (var i=1; i<numanc; i++)
		bctx.lineTo(250+bxanc[i], 250-byanc[i]);
	bctx.stroke();
	bctx.fillStyle = '#ffaaff';
	for (var i=1; i<numanc; i++)
	{
			x = bxanc[i-1] + (bxanc[i]-bxanc[i-1])*(btime/fdf);
			y = byanc[i-1] + (byanc[i]-byanc[i-1])*(btime/fdf);
			bctx.beginPath();
			bctx.arc(250+x,250-y,4,0,2*Math.PI,false);
			bctx.fill();
	}
	}
	*/
}
	// anchor points
	bctx.lineWidth = 2;
	bctx.strokeStyle = '#aaaaaa';
	bctx.fillStyle = '#dddddd';
	for (var i=1; i<numanc-1; i++)
	{
		bctx.beginPath();
		bctx.arc(250+bxanc[i], 250-byanc[i], 8,0,2 * Math.PI, false);
		bctx.fill();
		bctx.stroke();
	}

	// bezier curve
	bctx.lineWidth = 2;
	bctx.strokeStyle = '#999999';
	bctx.beginPath();
	bctx.moveTo(250+bxanc[0],250-byanc[0]);
	px = bxanc[0]; py = byanc[0];
	var coll = false;
	for (var i=0; i<=100; i++)
	{
		bx = bez_getX(i*0.01);
		by = bez_getY(i*0.01);
		if (by < 0 && coll == false)
		{
			coll = true;
			bctx.stroke();
			bctx.lineWidth = 4;
			bctx.strokeStyle = '#ff4433';
			bctx.beginPath();
			bctx.moveTo(250+px,250-py);
		} else if (by >= 0 && coll == true) {
			coll = false;
			bctx.stroke();
			bctx.lineWidth = 2;
			bctx.strokeStyle = '#999999';
			bctx.beginPath();
			bctx.moveTo(250+px,250-py);
		}
		bctx.lineTo(250+bx, 250-by);
		px=bx; py=by;
	}
	bctx.stroke();


	// current position
	bctx.fillStyle = '#7996de';
	bctx.beginPath();
	bctx.arc(250+bxpos, 250-bypos, 5,0,2 * Math.PI, false);
	bctx.fill();

	// bottom region
	bctx.fillStyle = '#ffffff';
	bctx.fillRect(3,310,494,37);
	bctx.fillStyle = '#666666';
	bctx.fillRect(0,307,500,3);

	// buttons
	bctx.fillStyle = "#999999";
	bctx.beginPath();
	bctx.arc(35,328, 10, 0, 2 * Math.PI, false);
	bctx.stroke();
	bctx.fillStyle = "#e1aa9d";
	if (bmode == 0) bctx.fillStyle = '#83e171';
	bctx.fill();
	bctx.fillStyle = "#999999";
	bctx.fillText("COSINE",50,333);
	bctx.beginPath();
	bctx.arc(125,328, 10, 0, 2 * Math.PI, false);
	bctx.stroke();
	bctx.fillStyle = "#e1aa9d";
	if (bmode == 1) bctx.fillStyle = '#83e171';
	bctx.fill();
	bctx.fillStyle = "#999999";
	bctx.fillText("BEZIER",140,333);

}



