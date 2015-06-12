#ifndef SCAN_H
#define SCAN_H
#include <string.h>
#include <stdlib.h>
#include <cmath>

class scan
{
public:
	int num;
	float *angle, *dist, *weight;

	scan (int n)
	{
		num = n;
		angle = (float*) malloc(num*sizeof(float));
		dist = (float*) malloc(num*sizeof(float));
		weight = (float*) malloc(num*sizeof(float));
	}

	scan* copy ()
	{
		scan *ret;
		ret = new scan(num);
		memcpy(ret->angle, angle, num*sizeof(float));
		memcpy(ret->dist, dist, num*sizeof(float));
		memcpy(ret->weight, weight, num*sizeof(float));
		return ret;
	}

	// a way of combining two scans
	// if the new scan has similar angles, update here
	// if the new scan has new angles, add them in order here
	void incorporate (scan *s)
	{
		int ii, ij, numnew, ind;
		float diff, *newangle, *newdist, *newweight;

		// look for unmatched angles
		numnew = s->num;
		for (ii=0; ii<num; ii++)
		{
			diff = 1e4;
			for (ij=0; ij<s->num; ij++)
			{
				diff = fabs(angle[ii] - s->angle[ij]);
				if (diff < 0.004)
					numnew --;
			}
		}
		// we have angles from the new scan that could not be matched!
		// make space
		newangle = new float [numnew+num];
		newdist = new float [numnew+num];
		newweight = new float [numnew+num];
		// copy over data
		ii = 0;
		ij = 0;
		ind = 0;
		while (ii+ij < numnew+num)
		{
			// compare elements
			if (fabs(angle[ii] - s->angle[ij]) < 0.004)
			{
				newangle[ind] = s->angle[ij];
				newdist[ind] = s->dist[ij];
				newweight[ind] = s->weight[ij];
				ii++;
				ij++;
			} else if (angle[ii] < s->angle[ij]) {
				newangle[ind] = angle[ii];
				newdist[ind] = dist[ii];
				newweight[ind] = weight[ii];
				ii++;
			} else {
				newangle[ind] = s->angle[ij];
				newdist[ind] = s->dist[ij];
				newweight[ind] = s->weight[ij];
				ij++;
			}
		ind++;
		}
		// replace arrays
		delete [] dist;
		delete [] angle;
		delete [] weight;
		dist = newdist;
		angle = newangle;
		weight = newweight;
	}

	~scan ()
	{
		free(angle);
		free(dist);
		free(weight);
	}
};
#endif
