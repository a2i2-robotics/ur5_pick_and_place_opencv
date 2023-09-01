#include "misc.h"

std::string read_filter(std::string prompt_text)
{
	std::string filter;
	std::cout << prompt_text;
	std::cin >> filter;
	std::cout << "Setting the filter to: " << filter;
	return filter;
}
