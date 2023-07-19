#include "misc.h"

std::string read_filter()
{
	std::string filter;
	std::cout << "Enter your filter \n";
	std::cin >> filter;
	std::cout << "Setting the filter to: " << filter;
	return filter;
}
