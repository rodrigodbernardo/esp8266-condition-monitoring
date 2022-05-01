#include <iostream>
#include <math.h>       /* pow */
#include <vector>

using namespace std;



double rms = 7932.627919;
const int n_classes = 3;
double threshold[n_classes] = {6449.948355229731, 7038.69402258786, 7750.546329672901};
int result_class = 0;

int main(){
	for (int i = 0; i < (n_classes - 1); i++)
	{
		if(rms >= threshold[i])
		{
			result_class = i+1;
		}
	}
	
	printf("Result class = %i",result_class);
}