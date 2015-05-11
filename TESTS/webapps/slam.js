
/* WARNING: I haven't seriously used javascript since I was 
 * maybe 17 years old, so in a sense I've never used it
 * seriously. This code is a mess and I couldn't care less.
*/

// everything is a global variable? sure.
var scanv, sctx;
var stimeout;
var sdt = 2.0;
var sstep = 0;
var ssimrun = false; // allows sim to be paused
var sdragging = false;
var sslider;
var smode = 2;
var sxpos, sypos, sang;
var snumangles = 72;
var sscanang;
var sscandist = Array(snumangles);
// collision rectangles
var snumrects = 0;
var scolx1 = [];
var scolx2 = [];
var scoly1 = [];
var scoly2 = [];
// slam stuff
var nx = 60;
var ny = 60;
var map = Array(nx*ny);
var sxguess, syguess, sangguess;
var sxerr, syerr, saerr;
// tuning
var lidar_error = 0.02;
var slam_enable = true;

// this gets called on page load?
slam_init();
function slam_init()
{
	scanv = document.getElementById('slam_sim');
	sctx = scanv.getContext('2d');
	sctx.font="Bold 12px Arial";

	sxpos = 0.0;
	sypos = -60.0;
	sang = 0.0;
	sxguess = 0.0;
	syguess = 0.0;
	sangguess = 0.0;
	sxerr = 0.0;
	syerr = 0.0;
	saerr = 0.0;
	sscanang = 0;

	// create environment
	slam_addrect(-500,-200,-500,500);
	slam_addrect(200,500,-500,500);
	slam_addrect(-500,500,-500,-150);
	slam_addrect(-500,500,150,500);
	slam_addrect(-30,30,-10,40);
	slam_addrect(-210,-70,-160,-100);
	slam_addrect(70,210,-160,-100);

	for (var ix=0; ix<nx; ix++)
	{
		for (var iy=0; iy<ny; iy++)
		{
			map[ix*ny+iy] = 0.5;
		}
	}

	// generic listeners from some other code I wrote
	scanv.addEventListener('click', slam_handleclick);
	scanv.addEventListener('mousedown', slam_handlemousedown);
	scanv.addEventListener('mousemove', slam_handlemousemove);
	scanv.addEventListener('mouseup', slam_handlemouseup);
	ssimrun = true;
	slam_draw();
	slam_run();
}

function slam_disttorect (ind, x0, y0, tht)
{
	var x, y, r, ret;
	ret = 1e6;

	// wall 1
	r = (scolx1[ind]-x0)/Math.cos(tht);
	y = y0+r*Math.sin(tht);
	if (y > scoly1[ind] && y < scoly2[ind] && r > 0.0) ret = Math.min(ret, r);
	// wall 2
	r = (scolx2[ind]-x0)/Math.cos(tht);
	y = y0+r*Math.sin(tht);
	if (y > scoly1[ind] && y < scoly2[ind] && r > 0.0) ret = Math.min(ret, r);
	// wall 3
	r = (scoly1[ind]-y0)/Math.sin(tht);
	x = x0+r*Math.cos(tht);
	if (x > scolx1[ind] && x < scolx2[ind] && r > 0.0) ret = Math.min(ret, r);
	// wall 4
	r = (scoly2[ind]-y0)/Math.sin(tht);
	x = x0+r*Math.cos(tht);
	if (x > scolx1[ind] && x < scolx2[ind] && r > 0.0) ret = Math.min(ret, r);
	return ret;
}

function slam_addrect (x1, x2, y1, y2)
{
	scolx1.push(x1);
	scolx2.push(x2);
	scoly1.push(y1);
	scoly2.push(y2);
	snumrects += 1;
}

function scan ()
{
	for (var ii=0; ii<snumangles; ii++)
	{
		tht = 2*Math.PI*ii/snumangles + (Math.random()-0.5)*lidar_error;
		mindist = 1e6;
		for (var ij=0; ij<snumrects; ij++)
		{
			thisdist = slam_disttorect(ij, sxpos, sypos, tht+sang);
			if (thisdist < mindist) mindist = thisdist;
		}
		sscandist[ii] = mindist*(1.0 + (Math.random()-0.5)*2*lidar_error);
	}
}

function slam_handlemousedown(event)
{
	var x = event.pageX - scanv.offsetLeft;
	var y = event.pageY - scanv.offsetTop;
	sdragging = true;
	sslider = -1;
	if (x >= 430 && x <= 580 && y >= 415 && y <= 435)
	{
		sslider = 0;
		slam_moveSlider(x-430);
	}
}
function slam_handlemouseup(event)
{sdragging = false;}

function slam_handlemousemove(event)
{
	if (sdragging && sslider >= 0)
		slam_moveSlider(event.pageX - scanv.offsetLeft - 430);
}

function slam_moveSlider(x)
{
	var value, logval;

	value = (x/150.)*0.1;
	lidar_error = value;
	if (lidar_error < 0.0) lidar_error = 0.0;
	if (lidar_error > 0.1) lidar_error = 0.1;

	if (ssimrun==false) slam_draw();
}

function slam_handleclick(event)
{
  var x = event.pageX - scanv.offsetLeft;
  var y = event.pageY - scanv.offsetTop;

	if (Math.sqrt(Math.pow(x-85,2) + Math.pow(y-425,2)) <= 10)
	  smode = 0;
	if (Math.sqrt(Math.pow(x-175,2) + Math.pow(y-425,2)) <= 10)
	  smode = 1;
	if (Math.sqrt(Math.pow(x-265,2) + Math.pow(y-425,2)) <= 10)
	  smode = 2;
  if (ssimrun==false) slam_draw();
}


// contains the primary calculations of the sim
function slam_run()
{
	// call run again in a moment
	if (ssimrun==true) stimeout = setTimeout('slam_run()', 60); // 30ms delay?


	sang += 0.006*sdt;
	if (sang < -Math.PI) sang += 2*Math.PI;
	if (sang >= Math.PI) sang -= 2*Math.PI;
	sxpos += 0.5*sdt*Math.cos(sang);
	sypos += 0.5*sdt*Math.sin(sang);
	sscanang += 7 + Math.round(Math.random()*5);
	if (sscanang > snumangles) sscanang = 0;

	// check for collisions


	// get lidar scan
	scan();

	sxerr = 0.95*sxerr + (Math.random()-0.5)*lidar_error*10.;
	syerr = 0.95*syerr + (Math.random()-0.5)*lidar_error*10.;
	saerr = 0.95*saerr + (Math.random()-0.5)*lidar_error;
	sxerr *= 0.99;
	syerr *= 0.99;
	saerr *= 0.95;

	// perform slam steps
	if (slam_enable)
	{
		sxguess = sxpos + sxerr;
		syguess = sypos + syerr;
		sangguess = sang + saerr;
	}
	slam_integrate();
	
	slam_draw();
}

function slam_integrate()
{
	// using current guess of robot position and angle,
	// integrate current scan into map
	
	for (var ii=0; ii<snumangles; ii++)
	{
		tht = 2*Math.PI*ii/snumangles;
		thisx = sxguess + sscandist[ii]*Math.cos(tht+sangguess);
		thisy = syguess + sscandist[ii]*Math.sin(tht+sangguess);
		thisx = Math.round(thisx*0.1 + nx/2);
		thisy = Math.round(thisy*0.1 + ny/2);
		if (thisx >= 0 && thisx < nx && thisy >= 0 && thisy < ny)
			map[thisx*ny+thisy] = 1.0;
	}
	
	for (var ix=0; ix<nx; ix++)
	{
		for (var iy=0; iy<ny; iy++)
		{
			xval = (ix-nx/2)/0.1;
			yval = (iy-ny/2)/0.1;
			thistht = Math.atan2(yval-syguess, xval-sxguess) - sangguess;
			if (thistht < 0.0) thistht += 2*Math.PI;
			thisdist = Math.sqrt(Math.pow(sxguess-xval,2)+Math.pow(syguess-yval,2));
			ind = Math.round(thistht*snumangles/(2*Math.PI));
			if (thisdist < sscandist[ind])
				map[ix*ny+iy] *= 0.90 + 0.1*thisdist/sscandist[ind];
		}
	}
}

function slam_draw()
{	
	sctx.fillStyle = 'white';
	sctx.fillRect(0, 0, 600, 450);

	if (smode == 0) // world-view
	{
		// draw laser scans
		sctx.strokeStyle = '#ffccdd';
		sctx.lineWidth = 1;
		for (var ii=0; ii<snumangles; ii++)
		{
			if (ii == sscanang) sctx.strokeStyle = '#ff7777';
			else sctx.strokeStyle = '#ffaaaa';
			sctx.beginPath();
			sctx.moveTo(300+sxpos,200+sypos);
			tht = 2.*Math.PI*ii/snumangles + sang;
			sctx.lineTo(300+sxpos+sscandist[ii]*Math.cos(tht), 
					200+sypos+sscandist[ii]*Math.sin(tht));
			sctx.stroke();
		}
		// draw body
		sctx.fillStyle = '#333333';
		sctx.beginPath();
		sctx.arc(300+sxpos, 200+sypos, 5, 0, 2.*Math.PI, false);
		sctx.fill();
		sctx.strokeStyle = '#333333';
		sctx.lineWidth = 2;
		sctx.beginPath();
		sctx.moveTo(300+sxpos, 200+sypos);
		sctx.lineTo(300+sxpos+15*Math.cos(sang), 200+sypos+15*Math.sin(sang));
		sctx.stroke();
		// draw walls
		sctx.fillStyle='#dddddd';
		for (var ij=0; ij<snumrects; ij++)
		{
			sctx.fillRect(300+scolx1[ij],200+scoly1[ij],
					scolx2[ij]-scolx1[ij],scoly2[ij]-scoly1[ij]);
		}
	} else if (smode == 1) { // scan-view
		// draw laser scans
		sctx.strokeStyle = '#ffccdd';
		sctx.fillStyle = '#ffaaaa';
		sctx.lineWidth = 1;
		for (var ii=0; ii<snumangles; ii++)
		{
			if (ii == sscanang) sctx.strokeStyle = '#ff7777';
			else sctx.strokeStyle = '#ffaaaa';
			sctx.beginPath();
			sctx.moveTo(300,200);
			tht = 2.*Math.PI*ii/snumangles;
			sctx.lineTo(300+sscandist[ii]*Math.cos(tht), 
					200+sscandist[ii]*Math.sin(tht));
			sctx.stroke();
			sctx.beginPath();
			sctx.arc(300+sscandist[ii]*Math.cos(tht), 
					200+sscandist[ii]*Math.sin(tht), 2, 0, 2*Math.PI, false);
			sctx.fill();
		}
		// draw body
		sctx.fillStyle = '#333333';
		sctx.beginPath();
		sctx.arc(300, 200, 5, 0, 2.*Math.PI, false);
		sctx.fill();
		sctx.strokeStyle = '#333333';
		sctx.lineWidth = 2;
		sctx.beginPath();
		sctx.moveTo(300, 200);
		sctx.lineTo(300+15, 200);
		sctx.stroke();
	} else { // slam-view
		s = 1.0
		// draw slam map
		sctx.strokeStyle='#ffffff';
		sctx.fillStyle='#333333';
		dx = 600./nx;
		dy = 600./ny;
		for (var ix=0; ix<nx; ix++)
		{
			for (var iy=0; iy<ny; iy++)
			{
				rval = Math.round(255 - map[ix*ny+iy]*255);
				gval = Math.round(255*(1 - 0.7*map[ix*ny+iy]));
				bval = Math.round(255*(1 - 0.5*map[ix*ny+iy]));
				sctx.fillStyle = "rgb("+rval+","+gval+","+bval+")";
				sctx.fillRect((ix-0.5)*dx,-100+(iy-0.5)*dy,dx,dy);
			}
		}
		// draw laser scans
		sctx.strokeStyle = '#ffaaaa';
		sctx.lineWidth = 1;
		for (var ii=0; ii<snumangles; ii++)
		{
			if (ii == sscanang) sctx.strokeStyle = '#ff7777';
			else sctx.strokeStyle = '#ffaaaa';
			sctx.beginPath();
			sctx.moveTo(300+sxguess*s,200+syguess*s);
			tht = 2.*Math.PI*ii/snumangles + sangguess;
			sctx.lineTo(300+(sxguess+sscandist[ii]*Math.cos(tht))*s, 
					200+(syguess+sscandist[ii]*Math.sin(tht))*s);
			sctx.stroke();
		}
		// draw body
		sctx.fillStyle = '#333333';
		sctx.beginPath();
		sctx.arc(300+sxguess*s, 200+syguess*s, 5, 0, 2.*Math.PI, false);
		sctx.fill();
		sctx.strokeStyle = '#333333';
		sctx.lineWidth = 2;
		sctx.beginPath();
		sctx.moveTo(300+sxguess*s, 200+syguess*s);
		sctx.lineTo(300+sxguess*s+15*Math.cos(sangguess), 
				200+syguess*s+15*Math.sin(sangguess));
		sctx.stroke();
	}

	// draw borders
	sctx.fillStyle = '#666666';
	sctx.fillRect(0, 0, 600, 3);
	sctx.fillRect(0, 0, 3, 450);
	sctx.fillRect(0, 447, 600, 3);
	sctx.fillRect(597, 0, 3, 600);
	sctx.fillRect(0, 400, 600, 3);
	sctx.fillStyle = '#ffffff';
	sctx.fillRect(3, 403, 594, 44);
	sctx.fillStyle = '#666666';
	sctx.fillRect(360,403,3,44);

	// buttons
	sctx.fillStyle = "#999999";
	sctx.fillText("VIEWS:", 75, 430);
	sctx.beginPath();
	sctx.arc(135,425, 10, 0, 2 * Math.PI, false);
	sctx.stroke();
	sctx.fillStyle = "#e1aa9d";
	if (smode == 0) sctx.fillStyle = "#83e171";
	sctx.fill();
	sctx.fillStyle = "#999999";
	sctx.fillText("GLOBAL",150,430);

	sctx.beginPath();
	sctx.arc(215,425, 10, 0, 2 * Math.PI, false);
	sctx.stroke();
	sctx.fillStyle = "#e1aa9d";
	if (smode == 1) sctx.fillStyle = "#83e171";
	sctx.fill();
	sctx.fillStyle = "#999999";
	sctx.fillText("ROBOT",230,430);

	sctx.beginPath();
	sctx.arc(295,425, 10, 0, 2 * Math.PI, false);
	sctx.stroke();
	sctx.fillStyle = "#e1aa9d";
	if (smode == 2) sctx.fillStyle = "#83e171";
	sctx.fill();
	sctx.fillStyle = "#999999";
	sctx.fillText("SLAM",310,430);

	// reset button

  // slider
	sctx.fillText("ERROR:",370,430);
  sctx.strokeStyle = '#bbbbbb';
  sctx.lineWidth = 5;
	sctx.beginPath();
  sctx.moveTo(430,425);
  sctx.lineTo(580,425);
  sctx.stroke();
  sctx.lineWidth = 1;
  sctx.strokeStyle = '#dddddd';
  for (j=0; j<3; j++)
  {
	sctx.moveTo(430+j*150.0/2.0,415);
	sctx.lineTo(430+j*150.0/2.0,435);
	sctx.stroke();
  }
  sctx.fillStyle = '#555555';
  sctx.fillRect(430-3+(lidar_error)*150/0.1+1,417,5,16);
}


