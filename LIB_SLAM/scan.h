
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

	~scan ()
	{
		free(angle);
		free(dist);
		free(weight);
	}
};
