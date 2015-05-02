#include "hexapod.h"

void hexapod::step (float dt)
{
	int ii, modt;
	float absspeed, speedsgn, ssfrac, ssfrac2;
	float cycletime, xpos, ypos, zpos, target[3];
	float sweepmodifier, speedmodifier, legraise;
	float turn_dist, thtpos, rpos, maxsweep, maxdist;
	float dist, tht0;

	// clamp speed and turning, just in case
	if (speed > MAX_SPEED) speed = MAX_SPEED;
	if (speed < -MAX_SPEED) speed = -MAX_SPEED;
	if (turning > 1.0) turning = 1.0;
	if (turning < -1.0) turning = -1.0;
	
	// make sure speed doesnt change too rapidly
	smoothspeed = 0.95*smoothspeed + 0.05*speed;
	smoothturning = 0.95*smoothturning + 0.05*turning;

	// to control walking, modify speed and turning
	absspeed = fabs(smoothspeed);
	speedsgn = 1.0;
	if (smoothspeed < 0.0) speedsgn = -1.0;

	// walking speed is influenced by leg sweep and movement speed
	legraise = 1.0;
	if (absspeed < 0.05) 
		legraise = absspeed/0.05;
	if (absspeed < 0.2)
	{
		sweepmodifier = absspeed*0.8/0.2;
		speedmodifier = 0.25;
	} else if (absspeed < 0.8) {
		sweepmodifier = 0.8;
		speedmodifier = absspeed/sweepmodifier;
	} else if (absspeed < 1.0) {
		sweepmodifier = absspeed;
		speedmodifier = 1.0;
	} else {
		sweepmodifier = 1.0;
		speedmodifier = absspeed;
	}
	speedmodifier *= speedsgn;

	if (ssrunning)
	{
		sstime += dt;
		ssfrac = sstime/SS_DURATION;
		for (ii=0; ii<6; ii++)
		{
			// compute final target
			target[0] = legpos[ii][0]*1.5;
			if (ii == 0 || ii == 2) target[1] = 14.0;
			if (ii == 1) target[1] = 18.0;
			if (ii == 3 || ii == 5) target[1] = -14.0;
			if (ii == 4) target[1] = -18.0;
			target[0] = legpos1[ii][0];
			target[1] = legpos1[ii][1];
			target[2] = -10.;
			// given final target, turn into current target
			if (ssfrac < 0.5)
			{
				ssfrac2 = ssfrac*2.0;
				if (ii % 2 == 0)
				{
					target[0] = ssx0[ii] + ssfrac2*(target[0]-ssx0[ii]);
					target[1] = ssy0[ii] + ssfrac2*(target[1]-ssy0[ii]);
					target[2] = ssz0[ii] + ssfrac2*(target[2]-ssz0[ii]) + 
						2.0*pow(sin(3.1416*ssfrac2),2);
				} else {
					target[0] = ssx0[ii];
					target[1] = ssy0[ii];
					target[2] = ssz0[ii];
				}
			} else {
				ssfrac2 = (ssfrac-0.5)*2.0;
				if (ii % 2 == 0)
				{
					// don't modify targets
				} else {
					target[0] = ssx0[ii] + ssfrac2*(target[0]-ssx0[ii]);
					target[1] = ssy0[ii] + ssfrac2*(target[1]-ssy0[ii]);
					target[2] = ssz0[ii] + ssfrac2*(target[2]-ssz0[ii]) +
						2.0*pow(sin(3.1416*ssfrac2),2);
				}
			}
			IKSolve(ii,target);
		}
		if (sstime > SS_DURATION)
		{
			ssrunning = false;
			sstime = 0.0;
		}
	} else {
	

	// based on current turning, compute turning math
	turn_dist = 1e5;
	if (fabs(smoothturning) < TURN_TOL)
		turn_dist = tan((1.0-TURN_TOL)*3.1416/2.0)*50.;
	if (fabs(smoothturning) > TURN_TOL)
		turn_dist = tan((1.0-smoothturning)*3.1416/2.0)*50.;
	// compute dist between turn_dist and farthest leg
	maxdist = 0.0;
	for (ii=0; ii<6; ii++)
	{
		dist = sqrt(pow(legpos1[ii][0],2) + pow(legpos1[ii][1]-turn_dist,2));
		if (dist > maxdist) maxdist = dist;
	}
	maxsweep = 8.*sweepmodifier/maxdist;
	if (turn_dist < 0.0) maxsweep = -maxsweep;

	// increment fake time
	time += dt*speedmodifier;
	if (time > 1.0) time -= 1.0;
	if (time < 0.0) time += 1.0;

	// loop through each leg to figure out where it should be right now
	for (ii=0; ii<6; ii++)
	{
		// where is this leg in the cycle of stepping?
		// the 0.5*ii is to completely de-sync legs
		// the other part is to adjust it more
		cycletime = fmod(time + 0.5*ii + (legpos[ii][0]-legpos[0][0])*0.0125, 1.0);
		
		// use bezier curve to either be up or down
		if (cycletime < fdf) b2d_walk_down.getPos(cycletime/fdf, &thtpos, &zpos);
		else b2d_walk_up.getPos((cycletime-fdf)/(1.-fdf), &thtpos, &zpos);
		// convert thtpos into angle?
		thtpos *= maxsweep;
//		if (turn_dist < 0.0) thtpos = -thtpos;
		//cout << ii << " " << time << " " << thtpos << endl;

		// convert rpos to xpos,ypos
		dist = sqrt(pow(legpos1[ii][0],2) + pow(legpos1[ii][1]-turn_dist,2));
		tht0 = atan2(legpos1[ii][1]-turn_dist,legpos1[ii][0]);
		xpos = dist*cos(thtpos+tht0);
		ypos = turn_dist + dist*sin(thtpos+tht0);

	//	cout << ii << " " << xpos << " " << ypos << " " << tht0 << endl;

		// set up the IK target
		target[0] = xpos;
		target[1] = ypos;
		target[2] = -10.0 + zpos*2.0*legraise;


		// perform IK solve
		IKSolve(ii,target);
		// TODO: add error handling if IK fails to converge
	}

	}
	setServoAngles();
}

void hexapod::safeStand ()
{
	int ii;
	float fkangles[3], fkpos[3];

	// initialize
	sstime = 0.0;
	ssrunning = true;
	// store FK leg positions
	for (ii=0; ii<6; ii++)
	{
		fkangles[0] = angle[ii*3+0];
		fkangles[1] = angle[ii*3+1];
		fkangles[2] = angle[ii*3+2];
		FKSolve(ii,fkangles,fkpos);
		ssx0[ii] = fkpos[0];
		ssy0[ii] = fkpos[1];
		ssz0[ii] = fkpos[2];
	}
}

hexapod::hexapod()
{
	int ii;

	// init constants for each leg
	length[0] = 5.04; // in cm
	length[1] = 6.52;
	length[2] = 13.3;
	femurangle = 9.53*DEGTORAD; // old: 11.5
	tibiaangle = 45.0*DEGTORAD; // old_47.3

	anglelb[0] = 75.0*DEGTORAD;
	angleub[0] = 245.*DEGTORAD;
	anglelb[1] = 55.0*DEGTORAD;
	angleub[1] = 250.*DEGTORAD;
	anglelb[2] = 40.0*DEGTORAD;
	angleub[2] = 215.*DEGTORAD;
	for (ii=0; ii<3; ii++)
	{
		anglelb[ii] -= 150.*DEGTORAD; // because servos are straight at 150 degrees
		angleub[ii] -= 150.*DEGTORAD;
		//anglelb[ii] *= 0.75;
		//angleub[ii] *= 0.75;
	}

	legpos[0][0] = 11.98;
	legpos[0][1] = 5.97;
	legpos[0][2] = 1.82;
	legpos[1][0] = 0.0;
	legpos[1][1] = 9.957;
	legpos[1][2] = legpos[0][2];
	// symmetry
	legpos[2][0] = -legpos[0][0];
	legpos[2][1] = legpos[0][1];
	legpos[2][2] = legpos[0][2];
	legpos[3][0] = legpos[0][0];
	legpos[3][1] = -legpos[0][1];
	legpos[3][2] = legpos[0][2];
	legpos[4][0] = legpos[1][0];
	legpos[4][1] = -legpos[1][1];
	legpos[4][2] = legpos[1][2];
	legpos[5][0] = legpos[2][0];
	legpos[5][1] = legpos[3][1];
	legpos[5][2] = legpos[0][2];

	// which way do the coxa servos point
	legang[0] = 45.0*DEGTORAD;
	legang[1] = 90.0*DEGTORAD;
	legang[2] = 135.*DEGTORAD;
	legang[3] =-45.*DEGTORAD;
	legang[4] =-90.0*DEGTORAD;
	legang[5] =-135.0*DEGTORAD;

	// default target for each leg
	for (ii=0; ii<6; ii++)
	{
		legpos1[ii][0] = legpos[ii][0] + 12.0 * cos(legang[ii]);
		legpos1[ii][1] = legpos[ii][1] + 12.0 * sin(legang[ii]);
		legpos1[ii][2] = -10.0;
	}

	// initialize bezier curve gait
	// goes from +-1 in x and 0 to 1 in z
	b2d_walk_up.addPoint(-0.83775,0);
	b2d_walk_up.addPoint(-1.11701,0);
	b2d_walk_up.addPoint(-1.39626,0);
	b2d_walk_up.addPoint(0,3.2);
	b2d_walk_up.addPoint(1.39626,0);
	b2d_walk_up.addPoint(1.11701,0);
	b2d_walk_up.addPoint(0.83775,0);

	b2d_walk_down.addPoint(0.83775,0);
	b2d_walk_down.addPoint(-0.83775,0);

	speed = 0.0;
	smoothspeed = 0.0;
	time = 0.0;
	fdf = 0.55;
}

void hexapod::setAngles ()
{
	int ii;
	for (ii=0; ii<6; ii++)
	{
		angle[ii*3] = (servoangle[ii*3]-150.)*DEGTORAD;
		angle[ii*3+1] = (servoangle[ii*3+1]-150.)*DEGTORAD;
		angle[ii*3+2] = (servoangle[ii*3+2]-150.)*DEGTORAD;
	}
}

void hexapod::setServoAngles ()
{
	int ii;
	for (ii=0; ii<6; ii++)
	{
		servoangle[ii*3] = angle[ii*3]*RADTODEG + 150.;
		servoangle[ii*3+1] = angle[ii*3+1]*RADTODEG + 150.;
		servoangle[ii*3+2] = angle[ii*3+2]*RADTODEG + 150.;
	}
}

// target is a float[3] giving absolute position
// return whether or not the solver was successful
bool hexapod::IKSolve (int leg, float *target)
{
	int iter;
	bool converged;
	float diff;
	float targetr, targetz, targetang;
	float fkpos[3], fkangles[3], J[2][2], inv[2][2], delta[2], p[2];
	float posr, posz, ang1, ang2, det;

	// convert absolute position to polar around leg root
	targetz = target[2] - legpos[leg][2];
	targetr = sqrt(pow(target[0]-legpos[leg][0],2) + pow(target[1]-legpos[leg][1],2));
	targetang = atan2(target[1]-legpos[leg][1],target[0]-legpos[leg][0]) - legang[leg]; // atan2 [-pi:pi]

	//cout << "test " << legang[leg] << " " << target[0]-legpos[leg][0] << " " << target[1]-legpos[leg][1] << endl;
	//cout << "coxa: " << angle[leg*3] << " -> " << targetang << endl;
	// easy part: can the coxa servo get to the right angle?
	if (targetang > angleub[0] || targetang < anglelb[0]) return false;
	// else, go ahead and set coxa servo. One out of three angles done!
	angle[leg*3] = targetang;

	// begin 2-joint IK solver using jacobian pseudo-inverse
	// whenever we call FKSolve, need to convert to polar coords around leg root
	fkangles[0] = angle[leg*3]; // already solved for
	// starting point is influenced by actual current point
	// but this makes it safer in case the leg has somehow gone out of bounds
	fkangles[1] = angle[leg*3+1]*0.5;
	fkangles[2] = angle[leg*3+2]*0.5;
	FKSolve(leg, fkangles, fkpos);
	posz = fkpos[2] - legpos[leg][2];
	posr = sqrt(pow(fkpos[0]-legpos[leg][0],2) + pow(fkpos[1]-legpos[leg][1],2));

	//cout << "init " << fkangles[1] << " " << fkangles[2] << " " << posr << " " << posz << endl;
	//cout << "target: " << targetr << " " << targetz << endl;
	diff = sqrt(pow(targetr-posr,2) + pow(targetz-posz,2));
	//cout << "initdiff: " << diff << endl;
	// ITERATE
	converged = false;
	for (iter=0; iter<MAXITER && !converged; iter++)
	{
		// compute jacobian
		p[0] = targetr - posr;
		p[1] = targetz - posz;
		ang1 = fkangles[1]-femurangle;
		ang2 = fkangles[2]-tibiaangle;
		J[0][0] = -length[1]*sin(ang1) - length[2]*sin(ang1+ang2); // dr/dang1
		J[1][0] = -length[2]*sin(ang1+ang2); // dr/dang2
		J[0][1] = length[1]*cos(ang1) + length[2]*cos(ang1+ang2); // dz/dang2
		J[1][1] = length[2]*cos(ang1+ang2); // dz/dang2
		// compute inverse
		det = 1.0/(J[0][0]*J[1][1]-J[0][1]*J[1][0]);
		inv[0][0] = J[1][1]*det;
		inv[1][0] = -J[1][0]*det;
		inv[0][1] = -J[0][1]*det;
		inv[1][1] = J[0][0]*det;
		//cout << "J: " << J[0][0] << " " << J[0][1] << " " << J[1][0] << " " << J[1][1] << endl;
		//cout << "inv: " << inv[0][0] << " " << inv[0][1] << " " << inv[1][0] << " " << inv[1][1] << endl;
		delta[0] = p[0]*inv[0][0] + p[1]*inv[0][1];
		delta[1] = p[0]*inv[1][0] + p[1]*inv[1][1];
		//cout << "delta: " << delta[0] << " " << delta[1] << " p: " << p[0] << " " << p[1] << endl;
		fkangles[1] += delta[0]*0.5;
		fkangles[2] += delta[1]*0.5;
		// enforce bounds
		if (fkangles[1] >= angleub[1]) 
			{fkangles[1] = angleub[1]-ANGEPS; cout << "ang1ub"<<leg<<" " << angleub[1] << endl;}
		if (fkangles[1] <= anglelb[1]) 
			{fkangles[1] = anglelb[1]+ANGEPS; cout << "ang1lb"<<leg<<" " << anglelb[1] << endl;}
		if (fkangles[2] >= angleub[2]) 
			{fkangles[2] = angleub[2]-ANGEPS; cout << "ang2ub"<<leg<<" " << angleub[2] << endl;}
		if (fkangles[2] <= anglelb[2]) 
			{fkangles[2] = anglelb[2]+ANGEPS; cout << "ang2lb"<<leg<<" " << anglelb[2] << endl;}
		// FK
		FKSolve(leg, fkangles, fkpos);
		posz = fkpos[2] - legpos[leg][2];
		posr = sqrt(pow(fkpos[0]-legpos[leg][0],2) + pow(fkpos[1]-legpos[leg][1],2));
		// convergence criteria
		diff = sqrt(pow(targetr-posr,2) + pow(targetz-posz,2));
		//cout << iter << " " << diff << " " << posr << " " << posz << endl;
		if (diff < TOLERANCE) converged = true; // 1 mm tolerance
	}

	// converged?
	if (converged)
	{
		angle[leg*3+1] = fkangles[1];
		angle[leg*3+2] = fkangles[2];
	}
	
	//if (converged) cout << iter << endl;

	return converged;
}

// forward kinematics in absolute coordinate system
// given a flat[3] angles, compute position and put in float[3] pos
// input angles are in radians and are offset properly
void hexapod::FKSolve (int leg, float *angles, float *pos)
{
	float r, ang0, ang1, ang2;

	ang0 = angles[0]+legang[leg];
	ang1 = angles[1]-femurangle;
	ang2 = angles[2]-tibiaangle;

	r = length[0] + length[1]*cos(ang1) + length[2]*cos(ang1+ang2);
	pos[0] = legpos[leg][0] + r*cos(ang0);
	pos[1] = legpos[leg][1] + r*sin(ang0);
	pos[2] = legpos[leg][2] + length[1]*sin(ang1) + length[2]*sin(ang1+ang2);
}

void hexapod::stand ()
{
	int ii;
	float target[3];
	for (ii=0; ii<6; ii++)
	{
		target[0] = legpos[ii][0] + 10.0*cos(legang[ii]);
		target[1] = legpos[ii][1] + 10.0*sin(legang[ii]);
		target[2] = -10.0;
		IKSolve(ii,target);
	}
	setServoAngles();
}

void hexapod::sit ()
{
	int ii;
	float target[3];
	for (ii=0; ii<6; ii++)
	{
		target[0] = legpos[ii][0] + 10.0*cos(legang[ii]);
		target[1] = legpos[ii][1] + 10.0*sin(legang[ii]);
		target[2] = -5.0;
		IKSolve(ii,target);
	}
	setServoAngles();
}

