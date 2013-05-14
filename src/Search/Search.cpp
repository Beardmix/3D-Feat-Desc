#include "Search.h"

// Constructors
Search::Search()
{
}
Search::Search(PCLCloud cloud, int * parameters, long ind_picked) 
	: cloud(cloud), parameters(parameters), match(new Match(cloud, parameters)), best(ind_picked)
{
	match->setReference(ind_picked);
	max_dist = 5;
}
Search::~Search()
{
	delete match;
}
void Search::setReference(long index)
{
	match->setReference( index );
	std::cout << "Reference Changed: " << index << std::endl; 
}

void Search::clearGrid()
{
	grid.clear();
}
void Search::generateGrid(int type, int density)
{
	grid.clear();
	switch (type)
	{
		case 0: // Uniform
			for (int x = 0; x < density; x ++)
			{
				for (int y = 0; y < density; y ++)
				{
					grid.push_back( floor( x * cloud->width /(float)density + y * cloud->width * cloud->height /(float)density ));
				}
			}
			break;
		case 1: // Random
			grid.push_back(best);
			for (int x = 0; x < density; x ++)
			{
				for (int y = 0; y < density-1; y ++)
				{
					grid.push_back( floor( (x+rnd(0)/2.0) * cloud->width /(float)density + (y+rnd(0)/2.0) * cloud->width * cloud->height /(float)density ));
				}
			}
			break;
		case 2: // Best Centered
			density /= 2;
			grid.push_back(best);
			float base = 2; // Must be superior to 1.5
			long w = best % cloud->width;
			long h = floor(best / (float)cloud->width);
			double aX = w / pow(base, density);
			for (int x = 0; x < density; x ++)
			{
				double aY = h / pow(base, density);
				for (int y = 0; y < density; y ++)
				{
					grid.push_back( floor( (w - aX) + floor(h - aY) * cloud->width ));
					aY *= base; 
				}
				aX *= base; 
			}
			aX = (cloud->width - w) / pow(base, density);
			for (int x = 0; x < density; x ++)
			{
				double aY = h / pow(base, density);
				for (int y = 0; y < density; y ++)
				{
					grid.push_back( floor( (w + aX) + floor(h - aY) * cloud->width ));
					aY *= base; 
				}
				aX *= base; 
			}
			aX = w / pow(base, density);
			for (int x = 0; x < density; x ++)
			{
				double aY = (cloud->height - h) / pow(base, density);
				for (int y = 0; y < density; y ++)
				{
					grid.push_back( floor( (w - aX) + floor(h + aY) * cloud->width ));
					aY *= base; 
				}
				aX *= base; 
			}
			aX = (cloud->width - w) / pow(base, density);
			for (int x = 0; x < density; x ++)
			{
				double aY = (cloud->height - h) / pow(base, density);
				for (int y = 0; y < density; y ++)
				{
					grid.push_back( floor( (w + aX) + floor(h + aY) * cloud->width ));
					aY *= base; 
				}
				aX *= base; 
			}
			break;
	}
}



std::vector<long> Search::runGrid()
{
	min = 101;
	best = -1;
	for (size_t i = 0; i < grid.size(); i ++)
	{
		ind = grid[i];
		diff = 100;
		if( cloud->points[ind].z == cloud->points[ind].z)
		{
			match->setPoint( ind );
			match->compareHue();
			if (match->getDiffHue() < 0.1) 
			{
				match->compareLayer(1);
				diff = match->getDiff(1);
				if (diff != diff)
					diff = 100;
				if (diff < min){
					min = diff;
					best = ind;
				}
			}
		}
		
		cloud->points[ind].rgb = color( 2.5 * diff, (255 - 5 * diff) * (1 - floor(diff / 50.1)), 2.5 * diff );
	}
	if (best == -1){
		std::cout << "Warning: no points studied, filter too strong" << std::endl; 
		clearGrid();
	 	generateGrid(1,30);
	}
	std::vector<long> res;
	res.push_back(best);
	res.push_back((long)floor(min));
	return res;
}


std::vector<long> Search::runSA(long ind_start)
{
	temperature = 100.0;
	ind_old = ind_start;
	ene_old = 100;
	int iter = 0;
	do{
		// Index choice
		ind = ind_old + floor( rnd(1) * temperature * max_dist ); // rnd on X
		ind += cloud->width * floor( rnd(1) * temperature * max_dist ); // rnd on Y
		
		// Index saturation
		if (ind < 0) 
			ind = 0;
		if (ind > (long) cloud->size()-1)
			ind = cloud->size()-1;
			
		if( cloud->points[ind].z == cloud->points[ind].z)
		{
			// Compute the signature
			match->setPoint(ind);
			match->compareHue();
			if (match->getDiffHue() < 0.1) 
			{
				match->compareLayer(1);
				ene = match->getDiff(1);
				if (ene != ene)
					ene = 100;
				cloud->points[ind].rgb = color( 2.5 * ene, (255 - 5 * ene) * (1 - floor(ene / 50.1)), 2.5 * ene );
				deltaE = ene - ene_old;
		
				if( deltaE < 0 || rnd(0) < exp( -1 / temperature * deltaE ))
				{
					ene_old = ene;
					ind_old = ind;
				}
			}
		}
		temperature = 0.95 * temperature;
		iter++;
		
	}while(temperature > 1);
	
	
	std::vector<long> res;
	res.push_back(ind_old);
	res.push_back((long)floor(ene_old));
	return res;
}

