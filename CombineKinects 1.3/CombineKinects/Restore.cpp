/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/


#include <cstdio>

#include <iostream>

#include <algorithm>

#include <assert.h>

#include <cstring>

#include "Restore.h"

#include "image.h"

#include "misc.h"

#include "pnmfile.h"


#define ITER 5        // number of BP iterations at each scale
#define LEVELS 1     // number of scales

#define DISC_K 200.0F       // truncation of discontinuity cost
#define DATA_K 10000.0F     // truncation of data cost
#define LAMBDA1 0.05F         // weighting of first data cost
#define LAMBDA2 0.05F		// weighting of the second data cost


#define INF 1E10     // large cost
#define VALUES 4000   // number of possible graylevel values

#define SBIAS 1.0F		//Adjusts smoothness term bias (Decrease to bias towards smoothess)
#define DBIAS 1.0F		//Adjusts data term bias (Decrease to bias towards data)

#define ALPHA 0.0F	//Increases bias towards V2 Data term
#define BETA 0.0F	//Increases bias towards V1 Data term

// dt of 1d function
// min convolution algorithm for quadratic cost
float* Restore::dt(float *f, int n, int &low, int &up) {
	float *d = new float[n];
	int *v = new int[n];
	float *z = new float[n + 1];
	int k = 0; // index of rightmost parabola in lower envelope
	v[0] = 0; // Locations of parabolas in lower envelope
	z[0] = -INF; // Locations of boundaies between parabolas
	z[1] = +INF;

	// Compute lower envelope
	for (int q = low + 1; q <= up - 1; q++) {
		float s = ((f[q] + square(q)) - (f[v[k]] + square(v[k])))
			/ (2 * (q - v[k]));
		while (s <= z[k]) {
			k--;
			s = ((f[q] + square(q)) - (f[v[k]] + square(v[k]))) /
				(2 * (q - v[k]));
		}
		k++;
		v[k] = q;
		z[k] = s;
		z[k + 1] = +INF;
	}

	k = 0;

	for (int q = low; q <= up - 1; q++) {
		while (z[k + 1] < q)
			k++;
		d[q] = square(q - v[k]) + f[v[k]];

	}
	delete[] v;
	delete[] z;
	return d;
}


// compute message
void Restore::msg(float s1[VALUES], float s2[VALUES],
	float s3[VALUES], float s4[VALUES],
	float dst[VALUES], int &low, int &up) {

	// aggregate and find min
	float minimum = INF;

	for (int value = low; value <= up; value++) {
		dst[value] = s1[value] + s2[value] + s3[value] + s4[value];
		if (dst[value] < minimum)
			minimum = dst[value];
	}


	// dt
	float *tmp = dt(dst, VALUES, low, up);

	// truncate and store in destination vector
	minimum += DISC_K;
	for (int value = low; value <= up; value++)
		dst[value] = std::min(tmp[value], minimum);

	// normalize
	float val = 0;
	for (int value = low; value < up; value++)
		val += dst[value];

	val /= VALUES;
	for (int value = low; value < up; value++)
		dst[value] -= val;

	delete tmp;
}

// computation of data costs
image<float[VALUES]>* Restore::comp_data(image<uint16_t> *img1, image<uint16_t> *img2) {
	int width = img1->width();
	int height = img1->height();
	image<float[VALUES]> *data = new image<float[VALUES]>(width, height);

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			for (int value = 0; value < VALUES; value++) {
				//Quadratic model for data cost
				float val = square((float)(imRef(img1, x, y) - value));
				float val2 = square((float)(imRef(img2, x, y) - value));

				// Linear model probably requires Linear dt
				//float val = abs(imRef(img1, x, y) - value);
				//float val2 = abs(imRef(img2, x, y) - value);

				// With two label sets, we take the smallest difference between a label and the observed intensity.
				//imRef(data, x, y)[value] = std::min(LAMBDA1 * std::min(val, DATA_K), LAMBDA2 * std::min(val2, DATA_K));
				imRef(data, x, y)[value] = abs((float)((LAMBDA1 * std::min(val, DATA_K)) - (LAMBDA2 * std::min(val2, DATA_K))));
				//std::cout << imRef(data, x, y)[value] << " ";
				//imRef(data, x, y)[value] = LAMBDA1 * std::min(val, DATA_K);
			}
		}
	}

	return data;
}

// generate output from current messages
image<uint16_t>* Restore::output(image<float[VALUES]> *u, image<float[VALUES]> *d,
	image<float[VALUES]> *l, image<float[VALUES]> *r,
	image<float[VALUES]> *data) {

	int width = data->width();
	int height = data->height();
	image<uint16_t> *out = new image<uint16_t>(width, height);

	for (int y = 1; y < height - 1; y++) {
		for (int x = 1; x < width - 1; x++) {
			// keep track of best value for current pixel
			int best = 0;
			float best_val = INF;
			for (int value = 0; value < VALUES; value++) {
				float val =
					imRef(u, x, y + 1)[value] +
					imRef(d, x, y - 1)[value] +
					imRef(l, x + 1, y)[value] +
					imRef(r, x - 1, y)[value] +
					imRef(data, x, y)[value];
				if (val < best_val) {
					best_val = val;
					best = value;
				}
			}
			imRef(out, x, y) = best;
		}
	}

	return out;
}

// belief propagation using checkerboard update scheme
void Restore::bp_cb(image<float[VALUES]> *u, image<float[VALUES]> *d,
	image<float[VALUES]> *l, image<float[VALUES]> *r,
	image<float[VALUES]> *data,
	int iter, image<uint16_t> *img1, image<uint16_t> *img2) {

	int width = data->width();
	int height = data->height();

	for (int t = 0; t < ITER; t++) {
		std::cout << "iter " << t << "\n";

		for (int y = 1; y < height - 1; y++) {
			for (int x = ((y + t) % 2) + 1; x < width - 1; x += 2) {

				int lowerbound, upperbound, bound0;
				if (imRef(img2, x, y) > imRef(img1, x, y)) {
					bound0 = 1;
					upperbound = imRef(img2, x, y);
					lowerbound = imRef(img1, x, y);
				}
				else {
					bound0 = 2;
					upperbound = imRef(img1, x, y);
					lowerbound = imRef(img2, x, y);
				}

				msg(imRef(u, x, y + 1), imRef(l, x + 1, y), imRef(r, x - 1, y),
					imRef(data, x, y), imRef(u, x, y), lowerbound, upperbound);

				msg(imRef(d, x, y - 1), imRef(l, x + 1, y), imRef(r, x - 1, y),
					imRef(data, x, y), imRef(d, x, y), lowerbound, upperbound);

				msg(imRef(u, x, y + 1), imRef(d, x, y - 1), imRef(r, x - 1, y),
					imRef(data, x, y), imRef(r, x, y), lowerbound, upperbound);

				msg(imRef(u, x, y + 1), imRef(d, x, y - 1), imRef(l, x + 1, y),
					imRef(data, x, y), imRef(l, x, y), lowerbound, upperbound);
			}
		}
	}
}

// multiscale belief propagation for image restoration
image<uint16_t>* Restore::restore_ms(image<uint16_t>* img1, image<uint16_t>* img2)
{
	int width = img1->width();
	int height = img1->height();


	image<float[VALUES]> *u[LEVELS];
	image<float[VALUES]> *d[LEVELS];
	image<float[VALUES]> *l[LEVELS];
	image<float[VALUES]> *r[LEVELS];
	image<float[VALUES]> *data[LEVELS];

	/*
	for (int x = 0; x < img1->width(); ++x) {
		for (int y = 0; y < img1->height(); ++y) {
			uint16_t check = imRef(img1, x, y);
			std::cout << check;
			std::cout << "\n";
		}
	}
	
	for (int x = 0; x < img1->width(); ++x) {
		for (int y = 0; y < img1->height(); ++y) {
			int check = img1->data[x,y];
			std::cout << check;
			std::cout << "\n";
		}
	}*/


	for (int i = 0; i < LEVELS; i++)
		data[i] = new image<float[VALUES]>(width, height);


	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {

			int lowerbound, upperbound, bound0;
			if (imRef(img2, x, y) > imRef(img1, x, y)) {
				bound0 = 1;
				upperbound = imRef(img2, x, y);
				lowerbound = imRef(img1, x, y);
			}
			else {
				bound0 = 2;
				upperbound = imRef(img1, x, y);
				lowerbound = imRef(img2, x, y);
			}
			
			if (lowerbound == 0 && bound0 == 1) {

				for (int value = lowerbound; value <= upperbound; value++) {
					//float val2 = square((float)(imRef(img2, x, y) - value));
					//imRef(data[0], x, y)[value] = (LAMBDA1 * std::min(val2, DATA_K));
					imRef(data[0], x, y)[value] = (LAMBDA1 * DATA_K);
				}
				imRef(data[0], x, y)[imRef(img2, x, y)] = 0.0;
			}

			else if (lowerbound == 0 && bound0 == 2) {

				for (int value = lowerbound; value <= upperbound; value++) {
					//float val = square((float)(imRef(img1, x, y) - value));
					//imRef(data[0], x, y)[value] = (LAMBDA1 * std::min(val, DATA_K));
					imRef(data[0], x, y)[value] = (LAMBDA1 * DATA_K);
				}
				imRef(data[0], x, y)[imRef(img1, x, y)] = 0.0;
			}

			else {
				for (int value = lowerbound; value <= upperbound; value++) {

					//Quadratic model for data cost
					float val = square((float)(imRef(img1, x, y) - value));
					float val2 = square((float)(imRef(img2, x, y) - value));

					//With two label sets, we take the smallest difference between a label and the observed intensity.
					imRef(data[0], x, y)[value] = square((float)(((LAMBDA1 * std::min(val, DATA_K))-ALPHA) - ((LAMBDA2 * std::min(val2, DATA_K))-BETA)));

				}
			}
		}
	}
	//delete data;

	
	// data pyramid
	for (int i = 1; i < LEVELS; i++) {
		int old_width = data[i - 1]->width();
		int old_height = data[i - 1]->height();
		int new_width = (int)ceil(old_width / 2.0);
		int new_height = (int)ceil(old_height / 2.0);

		assert(new_width >= 1);
		assert(new_height >= 1);


		data[i] = new image<float[VALUES]>(new_width, new_height);
		for (int y = 0; y < old_height; y++) {
			for (int x = 0; x < old_width; x++) {

				int lowerbound, upperbound;
				if (imRef(img2, x, y) > imRef(img1, x, y)) {
					upperbound = imRef(img2, x, y);
					lowerbound = imRef(img1, x, y);
				}
				else {
					upperbound = imRef(img1, x, y);
					lowerbound = imRef(img2, x, y);
				}

				for (int value = lowerbound; value <= upperbound; value++) {
					imRef(data[i], x / 2, y / 2)[value] += imRef(data[i - 1], x, y)[value];
				}

			}
		}
	}
	

	
	// run bp from coarse to fine
	for (int i = LEVELS - 1; i >= 0; i--) {
		int width = data[i]->width();
		int height = data[i]->height();

		// allocate & init memory for messages
		if (i == LEVELS - 1) {
			// in the coarsest level messages are initialized to zero
			u[i] = new image<float[VALUES]>(width, height);
			d[i] = new image<float[VALUES]>(width, height);
			l[i] = new image<float[VALUES]>(width, height);
			r[i] = new image<float[VALUES]>(width, height);
		}
		else {
			// initialize messages from values of previous level
			u[i] = new image<float[VALUES]>(width, height, false);
			d[i] = new image<float[VALUES]>(width, height, false);
			l[i] = new image<float[VALUES]>(width, height, false);
			r[i] = new image<float[VALUES]>(width, height, false);

			for (int y = 0; y < height; y++) {
				for (int x = 0; x < width; x++) {

					int lowerbound, upperbound;
					if (imRef(img2, x, y) > imRef(img1, x, y)) {
						upperbound = imRef(img2, x, y);
						lowerbound = imRef(img1, x, y);
					}
					else {
						upperbound = imRef(img1, x, y);
						lowerbound = imRef(img2, x, y);
					}

					for (int value = lowerbound; value <= upperbound; value++) {
						imRef(u[i], x, y)[value] = imRef(u[i + 1], x / 2, y / 2)[value];
						imRef(d[i], x, y)[value] = imRef(d[i + 1], x / 2, y / 2)[value];
						imRef(l[i], x, y)[value] = imRef(l[i + 1], x / 2, y / 2)[value];
						imRef(r[i], x, y)[value] = imRef(r[i + 1], x / 2, y / 2)[value];
					} 
					
				}
			}
			// delete old messages and data
			delete u[i + 1];
			delete d[i + 1];
			delete l[i + 1];
			delete r[i + 1];
			delete data[i + 1];
		}

		// BP
		//bp_cb(u[i], d[i], l[i], r[i], data[i], ITER);

	} 


	int bwidth = data[0]->width();
	int bheight = data[0]->height();

	for (int t = 0; t < ITER; t++) {
		std::cout << "iter " << t << "\n";

		for (int y = 1; y < bheight - 1; y++) {
			for (int x = ((y + t) % 2) + 1; x < bwidth - 1; x += 2) {

				int lowerbound, upperbound;
				if (imRef(img2, x, y) > imRef(img1, x, y)) {
					upperbound = imRef(img2, x, y);
					lowerbound = imRef(img1, x, y);
				}
				else {
					upperbound = imRef(img1, x, y);
					lowerbound = imRef(img2, x, y);
				}

				msg(imRef(u[0], x, y + 1), imRef(l[0], x + 1, y), imRef(r[0], x - 1, y),
					imRef(data[0], x, y), imRef(u[0], x, y), lowerbound, upperbound);

				msg(imRef(d[0], x, y - 1), imRef(l[0], x + 1, y), imRef(r[0], x - 1, y),
					imRef(data[0], x, y), imRef(d[0], x, y), lowerbound, upperbound);

				msg(imRef(u[0], x, y + 1), imRef(d[0], x, y - 1), imRef(r[0], x - 1, y),
					imRef(data[0], x, y), imRef(r[0], x, y), lowerbound, upperbound);

				msg(imRef(u[0], x, y + 1), imRef(d[0], x, y - 1), imRef(l[0], x + 1, y),
					imRef(data[0], x, y), imRef(l[0], x, y), lowerbound, upperbound);
			}
		}
	}



	//image<uint16_t> *out = output(u[0], d[0], l[0], r[0], data[0]);
	image<uint16_t> *out = new image<uint16_t>(width, height);

	for (int y = 1; y < height - 1; y++) {
		for (int x = 1; x < width - 1; x++) {
			// keep track of best value for current pixel
			int best = 0;
			float best_val = INF;

			int lowerbound, upperbound, bound0;
			if (imRef(img2, x, y) > imRef(img1, x, y)) {
				bound0 = 1;
				upperbound = imRef(img2, x, y);
				lowerbound = imRef(img1, x, y);
			}
			else {
				bound0 = 2;
				upperbound = imRef(img1, x, y);
				lowerbound = imRef(img2, x, y);
			}


			for (int value = lowerbound; value <= upperbound; value++) {
				float val =
					SBIAS*
					(imRef(u[0], x, y + 1)[value] +
					imRef(d[0], x, y - 1)[value] +
					imRef(l[0], x + 1, y)[value] +
					imRef(r[0], x - 1, y)[value]) +
					
					(DBIAS*imRef(data[0], x, y)[value]);
				if (val < best_val) {
					best_val = val;
					best = value;
				}
			}

			imRef(out, x, y) = best;
		}
	}


	delete u[0];
	delete d[0];
	delete l[0];
	delete r[0];
	delete data[0];

	return out;

}




Restore::Restore()
{
}


Restore::~Restore()
{
}
