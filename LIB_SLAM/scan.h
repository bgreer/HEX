#include <string.h>
#include <stdlib.h>

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

	~scan ()
	{
		free(angle);
		free(dist);
		free(weight);
	}
};
