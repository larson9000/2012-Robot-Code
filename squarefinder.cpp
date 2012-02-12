#include <WPILib.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include "squarefinder.h"

SquareFinder::SquareFinder()
{
	lumPlane = imaqCreateImage(IMAQ_IMAGE_U8, 7);
}

SquareFinder::~SquareFinder()
{
	imaqDispose(lumPlane);
}

TargetReport SquareFinder::getBestTarget(HSLImage *img)
{
	TargetReport ret2;
	ret2.x = 1337.0; //error code
	ret2.y = 0;
	if(!img)
		return ret2;
	ret2.x = 1338.0;

	int height = img->GetHeight();
	int width = img->GetWidth();
	Image *image = img->GetImaqImage();
	//Parameter, Lower, Upper, Calibrated?, Exclude?
	//ParticleFilterCriteria2 particleCriteria[1] = { {IMAQ_MT_AREA,500,2000,0,0} };
	ParticleFilterCriteria2 particleCriteria_initial[1] = { {IMAQ_MT_AREA_BY_IMAGE_AREA,25,100,0,1} };
	ParticleFilterCriteria2 particleCriteria[1] = { {IMAQ_MT_RATIO_OF_EQUIVALENT_RECT_SIDES,1,2,0,0} };
	ParticleFilterOptions particleFilterOptions[1] = { {FALSE,0,FALSE} };
	ParticleFilterOptions particleFilterOptions_conn8[1] = { {FALSE,0,TRUE} };
	int numParticles;

	imaqExtractColorPlanes(image, IMAQ_HSL, NULL, NULL, lumPlane);
	image = lumPlane;
	//imaqLocalThreshold(image, image, 32, 32, IMAQ_BACKGROUND_CORRECTION, 0.2, IMAQ_DARK_OBJECTS, 1);
	imaqInverse(image,image,NULL);
	imaqAutoThreshold2(image, image, 2, IMAQ_THRESH_INTERCLASS, NULL);
	imaqParticleFilter3(image, image, particleCriteria_initial, 1, particleFilterOptions, NULL, &numParticles);
	imaqFillHoles(image, image, TRUE);

	int pKernel[9] = {1,1,1,1,1,1,1,1,1};
	StructuringElement structElem[1] = { { 3, 3, FALSE, pKernel } };
	imaqSizeFilter(image, image, TRUE, 2, IMAQ_KEEP_LARGE, structElem);

	imaqParticleFilter3(image, image, particleCriteria, 1, particleFilterOptions_conn8, NULL, &numParticles);
	//imaqWriteJPEGFile(image,"/step7.jpg",1000,NULL);

	vector<TargetReport> reports;

	//Use the most proportional.
	if(imaqCountParticles(image,TRUE,&numParticles)) {
		TargetReport ret;
		if(numParticles >= 1) {
			for(int i = 0; i < numParticles; i++) {
				double h,w,area;
				imaqMeasureParticle(image,i,FALSE,IMAQ_MT_BOUNDING_RECT_HEIGHT,&h);
				imaqMeasureParticle(image,i,FALSE,IMAQ_MT_BOUNDING_RECT_WIDTH,&w);
				imaqMeasureParticle(image,i,FALSE,IMAQ_MT_AREA,&area);
				if(area/(w*h) > 0.0) {
					ret.height = h;
					ret.width  = w;
					ret.size = area;
					imaqMeasureParticle(image,i,FALSE,IMAQ_MT_CENTER_OF_MASS_X,&ret.normalized_x);
					imaqMeasureParticle(image,i,FALSE,IMAQ_MT_CENTER_OF_MASS_Y,&ret.normalized_y);
					imaqMeasureParticle(image,i,FALSE,IMAQ_MT_BOUNDING_RECT_LEFT,&ret.x);
					imaqMeasureParticle(image,i,FALSE,IMAQ_MT_BOUNDING_RECT_TOP,&ret.y);
					ret.normalized_x = (-1.0+2.0*((ret.normalized_x)/width)); //Map to [-1.0,1.0]
					ret.normalized_y = (-1.0+2.0*((ret.normalized_y)/height));
					ret.distance = 732.7079220552562/w; //Approximates distance using lens optics.
					reports.push_back(ret);
				}
			}
		}
	}
	sort(reports.begin(),reports.end());

	/*
	printf("Target report(%i particles):\n",reports.size());
	for(unsigned int i = 0; i < reports.size(); i++) {
		printf("\tTarget %i:\n",i);
		printf("\t\tX: %f\n",reports.at(i).x);
		printf("\t\tY: %f\n",reports.at(i).y);
		printf("\t\tWidth: %f\n",reports.at(i).width);
		printf("\t\tHeight: %f\n",reports.at(i).height);
		printf("\t\tArea: %f\n",reports.at(i).size);
		printf("\t\tDistance: %f\n",reports.at(i).distance);
	}
	 */

	if(reports.size())
		ret2 = reports.at(0);
	return ret2;
}
