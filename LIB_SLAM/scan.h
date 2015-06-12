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

	void incorporate (scan *s)
	{
		int ii, ij;
		float diff;

		for (ii=0; ii<num; ii++)
		{
			diff = 1e4;
			for (ij=0; ij<s->num; ij++)
			{
				diff = fabs(angle[ii] - s->angle[ij]);
				if (diff < 0.005) dist[ii] = s->dist[ij];
			}
		}
	}

	~scan ()
	{
		free(angle);
		free(dist);
		free(weight);
	}
};
#endif
