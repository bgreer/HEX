var time = 0.0;
var canv, ctx;
var timeout;
var step = 0;
var simrun = false;
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

init();

function init()
{
  canv = document.getElementById('walk_sim01');
  ctx = canv.getContext('2d');
  clear();

  time = 0.0;
  curvature = 0.0;
  cx = 10000.0;
  sidedown = 0;
	xpos = 0.0;
	ypos = 0.0;
	ang = 0.0;

  for (var ii=0; ii<6; ii++)
  {
	tht = 2.0*Math.PI*ii/6.0;
	legx0[ii] = 50.*Math.cos(tht);
	legy0[ii] = 50.*Math.sin(tht);
	legx[ii] = legx0[ii];
	legy[ii] = legy0[ii];
  }

  draw();
  canv.addEventListener('click', handleclick);
  canv.addEventListener('mousedown', handlemousedown);
  canv.addEventListener('mousemove', handlemousemove);
  canv.addEventListener('mouseup', handlemouseup);
  simrun = true;
  run();
}

function handlemousedown(event)
{
	var x = event.pageX - canv.offsetLeft;
	var y = event.pageY - canv.offsetTop;
	dragging = true;
	slider = -1;
	if (x >= 100 && x <= 500 && y >= 174 && y <= 186)
		slider = 0;

	if (slider >= 0)
		moveSlider(x-100);
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
	if (curvature < -2.0) curvature = -2.0;
	if (curvature > 2.0) curvature = 2.0;

	if (simrun==false) {clear();draw();}
}

function handleclick(event)
{
  var x = event.pageX - canv.offsetLeft;
  var y = event.pageY - canv.offsetTop;

  if (simrun==false) {clear(); draw();}
}

function reset()
{

}


function run()
{
  // call run again in a moment
	time += 0.004;
	if (time > 1.0) time -= 1.0;
	sidedown = 0;
	if (time >= 0.5) sidedown = 1;


	mytime = time*2.0;
	if (time > 0.5) mytime = (time-0.5)*2.0;
//	curvature = 2.0*Math.sin(time/30.);
	cx = Math.tan(curvature*3.14/4.0)*30.;
	cx = 1e8;
	if (curvature > 0.01) cx = Math.tan((2.0-curvature)*3.14/4.0)*30.;
	if (curvature < -0.01) cx = Math.tan((2.0-curvature)*3.14/4.0)*30.;

	if (curvature < 0.01) sidedown = ((sidedown+1)%2);


	// compute position for each leg
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
	sweepangle = 40.0/maxdist;
	
	ang -= sweepangle*0.015;
	xpos -= 80.0*0.004*Math.sin(ang);
	ypos -= 80.0*0.004*Math.cos(ang);
	for (var ii=0; ii<6; ii++)
	{
		dist = Math.sqrt((cx-legx0[ii])*(cx-legx0[ii]) + legy0[ii]*legy0[ii]);
		tht0 = Math.atan2(legy0[ii], -(cx-legx0[ii]));
		dtht = sweepangle;
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


  // redraw every now and then
  step += 1;
  if (step % 5 == 0)
  {
    // draw things
    step = 0;
  	clear();
	draw();
  }

	if (simrun==true) timeout = setTimeout('run()', 1);
}

function draw()
{
  ctx.font="Bold 12px Arial";
	// save position and angle
	ctx.save();

	// draw background
	y0 = Math.round((ypos-200.0)/20.)*20. - ypos;
	x0 = Math.round((xpos-200.0)/20.)*20. - xpos;
	ctx.fillStyle = '#eeeeee';
	for (var ix=-30; ix<30; ix++)
	{
		for (var iy=-30; iy<30; iy++)
		{
			ctx.beginPath();
			x1 = ix*20 + x0;
			y1 = iy*20 + y0
			x = x1*Math.cos(ang) - y1*Math.sin(ang);
			y = x1*Math.sin(ang) + y1*Math.cos(ang);
			ctx.arc(x + 300, y + 80, 5, 0, 2.*Math.PI, false);
			ctx.fill();
		}
	}
	
	// draw circle center
	ctx.lineWidth = 1;
	ctx.fillStyle = '#ae8080';
	ctx.beginPath();
	ctx.arc(cx+300, 80, 5, 0, 2 * Math.PI, false);
	ctx.fill();


	// draw helper circles
	ctx.lineWidth = 2;
	ctx.strokeStyle = '#fff8f8';
	/*
	for (var ii=0; ii<6; ii++)
	{
		ctx.beginPath();
		dist = Math.sqrt((cx-legx0[ii])*(cx-legx0[ii]) + legy0[ii]*legy0[ii]);
		ctx.arc(300+cx,80,dist, 0, 2 * Math.PI, false);
		ctx.stroke();
	}*/

	// small path
	ctx.strokeStyle = '#aaaaaa';
	for (var ii=0; ii<6; ii++)
	{
		dist = Math.sqrt((cx-legx0[ii])*(cx-legx0[ii]) + legy0[ii]*legy0[ii]);
		tht0 = Math.atan2(-legy0[ii], -(cx-legx0[ii]));
		dtht = sweepangle*0.5;
		ctx.beginPath();
		ctx.arc(300+cx,80,dist, tht0-dtht, tht0+dtht, false);
		ctx.stroke();
	}


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
		if (ii % 2 == sidedown) 
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


}

function clear()
{
	ctx.fillStyle = '#666666';
	ctx.fillRect(0, 0, 600, 200);
	ctx.fillStyle = 'white';
	ctx.fillRect(3, 3, 594, 194);
	ctx.fillStyle = '#666666';
	ctx.fillRect(0, 160, 600, 3);
}

function end()
{
	clearTimeout(timeout);
}



