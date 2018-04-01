
#include <vector>

using namespace std;

class bezier2d
{
public:
	vector<float> xpos, ypos;

	bezier2d () {}

	void addPoint (float x, float y)
	{
		xpos.push_back(x);
		ypos.push_back(y);
	}

	// t must be [0:1]
	void getPos (float t, float *xret, float *yret)
	{
		int ii, ij, npoints;
		float *x, *y;

		npoints = xpos.size();
		x = new float [npoints];
		y = new float [npoints];
		// load with current points
		for (ii=0; ii<npoints; ii++)
		{
			x[ii] = xpos[ii];
			y[ii] = ypos[ii];
		}

		// iterate over levels
		for (ii=0; ii<npoints-1; ii++)
		{
			for (ij=0; ij<npoints-ii-1; ij++)
			{
				x[ij] = (1.0-t)*x[ij] + t*x[ij+1];
				y[ij] = (1.0-t)*y[ij] + t*y[ij+1];
			}
		}

		*xret = x[0];
		*yret = y[0];

		delete [] x;
		delete [] y;
	}
};
