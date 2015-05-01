
/* WARNING: I haven't seriously used javascript since I was 
 * maybe 17 years old, so in a sense I've never used it
 * seriously. This code is a mess and I couldn't care less.
*/

// everything is a global variable? sure.
var time = 0.0;
var dt;
var canv, ctx;
var timeout;
var step = 0;
var simrun = false; // allows sim to be paused
var dragging = false;
var slider = -1;
var sidedown;
var curvature;
var cx;
var legx0 = Array(6);
var legy0 = Array(6);
var legx = Array(6);
var legy = Array(6);
var sweepangle;
var xpos, ypos, ang;

// display options
var show_ground = true;
var show_turn_center = false;

// this gets called on page load?
init();

function init()
{
	canv = document.getElementById('walk_sim01');
	ctx = canv.getContext('2d');
	ctx.font="Bold 12px Arial";

	dt = 0.01;
	time = 0.0;
	curvature = 0.0;
	cx = 1e8;
	sidedown = 0;
	xpos = 0.0;
	ypos = 0.0;
	ang = 0.0;

	// compute initial positions of legs
	for (var ii=0; ii<6; ii++)
	{
		tht = 2.0*Math.PI*ii/6.0;
		legx0[ii] = 50.*Math.cos(tht);
		legy0[ii] = 50.*Math.sin(tht);
		legx[ii] = legx0[ii];
		legy[ii] = legy0[ii];
	}

	// generic listeners from some other code I wrote
	canv.addEventListener('click', handleclick);
	canv.addEventListener('mousedown', handlemousedown);
	canv.addEventListener('mousemove', handlemousemove);
	canv.addEventListener('mouseup', handlemouseup);
	simrun = true;
	draw();
	run();
}

function handlemousedown(event)
{
	var x = event.pageX - canv.offsetLeft;
	var y = event.pageY - canv.offsetTop;
	dragging = true;
	slider = -1;
	// bounds of the main slider
	if (x >= 100 && x <= 500 && y >= 174 && y <= 186)
	{
		slider = 0;
		moveSlider(x-100);
	}
}
function handlemouseup(event)
{dragging = false;}

function handlemousemove(event)
{
	if (dragging && slider >= 0)
		moveSlider(event.pageX - canv.offsetLeft - 100);
}

function moveSlider(x)
{
	var value, logval;

	value = (x/100.) - 2.0;
	curvature = value;
	if (curvature < -1.999) curvature = -1.999;
	if (curvature > 1.999) curvature = 1.999;

	if (simrun==false) draw();
}

function handleclick(event)
{
  var x = event.pageX - canv.offsetLeft;
  var y = event.pageY - canv.offsetTop;

  if (Math.sqrt(Math.pow(x-20,2) + Math.pow(y-20,2)) <= 10)
	  show_ground = !show_ground;

  if (Math.sqrt(Math.pow(x-490,2) + Math.pow(y-20,2)) <= 10)
	  show_turn_center = !show_turn_center;


  if (simrun==false) draw();
}


// contains the primary calculations of the sim
function run()
{
	// call run again in a moment
	if (simrun==true) timeout = setTimeout('run()', 30); // 30ms delay?
	
	// increment and loop time variable
	time += dt;
	if (time > 1.0) time -= 1.0;

	// which leg grouping is down?
	sidedown = 0;
	if (time >= 0.5) sidedown = 1;

	// mytime is a bad name
	// gives a [0,1] value for each half of the leg movement
	mytime = time*2.0;
	if (time > 0.5) mytime = (time-0.5)*2.0;
	
	// compute radius of curvature based on "curvature" from slider
	if (Math.abs(curvature) < 0.0001) curvature = 0.0001;
	if (curvature > 0.0) cx = Math.tan((2.0-curvature)*3.1415/4.0)*50.;
	if (curvature < 0.0) cx = Math.tan((2.0-curvature)*3.1415/4.0)*50.;

	if (curvature < 0.0) sidedown = ((sidedown+1)%2);


	// length = radius * angle
	// everyone needs same angle, capped by leg with farthest length to step
	maxdist = 0.0;
	sweepangle = 0.0;
	for (var ii=0; ii<6; ii++)
	{
		dist = Math.sqrt((cx-legx0[ii])*(cx-legx0[ii]) + legy0[ii]*legy0[ii]);
		if (dist > maxdist)
			maxdist = dist;
	}
	sweepangle = Math.sign(cx)*30.0/maxdist;
	
	// center of body rotates and moves
	ang -= sweepangle*dt*2; // rotation is easy
	// shift body in direction of angle
	xpos -= sweepangle*cx*dt*2*Math.sin(ang);
	ypos -= sweepangle*cx*dt*2*Math.cos(ang);

	// compute where each leg is
	for (var ii=0; ii<6; ii++)
	{
		dist = Math.sqrt((cx-legx0[ii])*(cx-legx0[ii]) + legy0[ii]*legy0[ii]);
		tht0 = Math.atan2(legy0[ii], -(cx-legx0[ii]));
		dtht = sweepangle*Math.sign(cx);
		if (ii % 2 == sidedown)
		{
			// leg is down
			pos = -dtht*(mytime-0.5) + tht0;
			legx[ii] = cx+dist*Math.cos(pos);
			legy[ii] = dist*Math.sin(pos);
		} else {
			// leg is up
			pos = dtht*(mytime-0.5) + tht0;
			legx[ii] = cx+dist*Math.cos(pos);
			legy[ii] = dist*Math.sin(pos);
		}
	}

	draw();
}

function draw()
{	
	ctx.fillStyle = 'white';
	ctx.fillRect(0, 0, 600, 200);

	// save position and angle
	ctx.save();
	if (show_ground)
	{
	// draw background
	y0 = Math.round((ypos-200.0)/30.)*30. - ypos;
	x0 = Math.round((xpos-200.0)/30.)*30. - xpos;
	ctx.fillStyle = '#eeeeee';
	for (var ix=-10; ix<20; ix++)
	{
		for (var iy=-10; iy<20; iy++)
		{
			ctx.beginPath();
			x1 = ix*30 + x0;
			y1 = iy*30 + y0
			x = x1*Math.cos(ang) - y1*Math.sin(ang);
			y = x1*Math.sin(ang) + y1*Math.cos(ang);
			ctx.arc(x + 300, y + 80, 5, 0, 2.*Math.PI, false);
			ctx.fill();
		}
	}
	}

	if (show_turn_center)
	{
	// long leg path
	ctx.strokeStyle = '#eeeeee';
	ctx.lineWidth=2;
	for (var ii=0; ii<6; ii++)
	{
		dist = Math.sqrt((cx-legx0[ii])*(cx-legx0[ii]) + legy0[ii]*legy0[ii]);
		ctx.beginPath();
		ctx.arc(300+cx,80,dist, 0., 2.*Math.PI, false);
		ctx.stroke();
	}


	// small leg path
	ctx.strokeStyle = '#ccccff';
	ctx.lineWidth=4;
	for (var ii=0; ii<6; ii++)
	{
		dist = Math.sqrt((cx-legx0[ii])*(cx-legx0[ii]) + legy0[ii]*legy0[ii]);
		tht0 = Math.atan2(-legy0[ii], -(cx-legx0[ii]));
		dtht = sweepangle*0.5*Math.sign(cx);
		ctx.beginPath();
		ctx.arc(300+cx,80,dist, tht0-dtht, tht0+dtht, false);
		ctx.stroke();
	}
	}

	// draw border on top
	ctx.fillStyle = '#666666';
	ctx.fillRect(0, 0, 600, 3);
	ctx.fillRect(0, 0, 3, 200);
	ctx.fillRect(0, 197, 600, 3);
	ctx.fillRect(597, 0, 3, 200);
	ctx.fillRect(0, 160, 600, 3);
	ctx.fillStyle = '#ffffff';
	ctx.fillRect(3, 163, 594, 34);

	// draw helper circles
	ctx.lineWidth = 2;
	ctx.strokeStyle = '#fff8f8';

	// draw hex body as circle
	ctx.beginPath();
	ctx.arc(300,80,20, 0, 2 * Math.PI, false);
	ctx.lineWidth = 2;
	ctx.strokeStyle = '#999999';
	ctx.fillStyle = '#999999';
	ctx.stroke();
	ctx.fill();

	

	// draw each leg
	for (var ii=0; ii<6; ii++)
	{
		tht = 2.0*Math.PI*ii/6.;
		x0 = 300 + 20*Math.cos(tht);
		y0 = 80 + 20*Math.sin(tht);
		x1 = 300 + legx[ii];
		y1 = 80 + legy[ii];
		// foot-down dot
		if ((ii%2==sidedown&&cx>0) || (ii%2!=sidedown&&cx<0))
		{
			ctx.fillStyle = '#444444';
			ctx.beginPath();
			ctx.arc(x1,y1, 3, 0, 2 * Math.PI, false);
			ctx.fill();
		}
		ctx.strokeStyle = '#999999';
		ctx.lineWidth = 8;
		ctx.beginPath();
		ctx.moveTo(x0,y0);
		ctx.lineTo(x1,y1);
		ctx.stroke();
	}

	if (show_turn_center)
	{
	// draw circle center
	ctx.lineWidth = 1;
	ctx.fillStyle = '#ce8080';
	ctx.beginPath();
	ctx.arc(cx+300, 80, 5, 0, 2 * Math.PI, false);
	ctx.fill();
	}
  

  // sliders
  ctx.strokeStyle = '#bbbbbb';
  ctx.lineWidth = 5;
	ctx.beginPath();
  ctx.moveTo(100,180);
  ctx.lineTo(500,180);
  ctx.stroke();
  ctx.lineWidth = 1;
  ctx.strokeStyle = '#dddddd';
  for (j=0; j<5; j++)
  {
	ctx.moveTo(100+j*400.0/4.0,170);
	ctx.lineTo(100+j*400.0/4.0,190);
	ctx.stroke();
  }
  
  // the slidey parts
  ctx.fillStyle = '#555555';
  ctx.fillRect(297+(curvature)*400/4+1,172,5,16);

  // display options (green = #83e171, red = #e1aa9d)
  ctx.strokeStyle = "#999999";
  ctx.lineWidth = 3;
  
	ctx.beginPath();
	ctx.arc(20,20, 10, 0, 2 * Math.PI, false);
	ctx.stroke();
	ctx.fillStyle = "#e1aa9d";
	if (show_ground) ctx.fillStyle = "#83e171";
	ctx.fill();
	ctx.fillStyle = "#999999";
	ctx.fillText("BACKGROUND",35,25);

	ctx.beginPath();
	ctx.arc(490,20, 10, 0, 2 * Math.PI, false);
	ctx.stroke();
	ctx.fillStyle = "#e1aa9d";
	if (show_turn_center) ctx.fillStyle = "#83e171";
	ctx.fill();
	ctx.fillStyle = "#999999";
	ctx.fillText("TURN GUIDES",505,25);

	ctx.fillStyle = "#777777";
	ctx.fillText("L",85,184);
	ctx.fillText("R",510,184);
}


